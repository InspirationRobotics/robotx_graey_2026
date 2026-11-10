"""Discovery helpers from InspirationRobotics/rx26_asv main abf7d821.
Copied unchanged from ocs_link.py; GRAEY config supplies its own fallback hosts.
"""
import ipaddress
import json
import socket
import threading
import time

DEFAULT_DISCOVERY_PORT = 37565

#: A COPY of OCS/rx_bridge/beacon.py: change both. OCS/tests/test_beacon.py
#: imports this file and fails if they disagree.
BEACON_KEY = "ocs_beacon"
BEACON_VERSION = 1

#: A beacon older than this is a past address, not a present one. The OCS sends
#: one a second, so this survives four lost datagrams.
BEACON_FRESH_S = 5.0

#: How often a beacon port that could not be bound is tried again. A port taken at
#: start -- a second ocs_client, or the last one still exiting -- must not leave
#: the boat on its fallback hosts for the rest of the run.
LISTEN_RETRY_S = 5.0

#: `ocs_host` value that means "find it from the beacon".
AUTO_HOST = "auto"

#: Where the OCS has been seen on the team networks (lab WiFi, Rocket), tried in
#: turn when no beacon arrives. Only used when the params file omits
#: ocs_fallback_hosts; the YAML is the source of truth.
DEFAULT_FALLBACK_HOSTS = ("192.168.100.107", "192.168.8.107")

#: RoboNation's course networks (Alpha .65 to Delta .68). Vehicles must never
#: reach RoboCommand, so nothing here dials, or believes a beacon from, these.
COURSE_NETWORKS = tuple(ipaddress.ip_network(n) for n in (
    "192.168.65.0/24", "192.168.66.0/24", "192.168.67.0/24", "192.168.68.0/24"))


def on_course_network(host) -> bool:
    """True if `host` is an IPv4 address inside a RoboCommand course network."""
    try:
        ip = ipaddress.IPv4Address(str(host).strip())
    except ValueError:
        return False
    return any(ip in net for net in COURSE_NETWORKS)


def parse_beacon(data) -> dict | None:
    """{"team", "port", "v"} for a well-formed v1 beacon, else None.

    Strict, like the OCS's own parse: anything not exactly this shape is not a
    beacon, and acting on a guess means dialling an address on a stray packet.
    """
    if not isinstance(data, (bytes, bytearray)) or len(data) > 512:
        return None
    try:
        obj = json.loads(bytes(data).decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        return None
    inner = obj.get(BEACON_KEY) if isinstance(obj, dict) else None
    if not isinstance(inner, dict) or len(obj) != 1:
        return None
    team, port, ver = inner.get("team"), inner.get("port"), inner.get("v")
    if not isinstance(team, str) or not team:
        return None
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        return None
    if isinstance(ver, bool) or ver != BEACON_VERSION:
        return None
    return {"team": team, "port": port, "v": ver}


def check_fallback_hosts(hosts) -> tuple:
    """(usable hosts in order, [problem strings]) for a configured fallback list.

    Only IPv4 literals outside the course networks: a hostname would make every
    attempt wait on a resolver. Shared with check_config, so the file and the
    code cannot disagree about what a good list is.
    """
    if isinstance(hosts, str):
        hosts = [hosts]                 # a hand-edited YAML with one bare string
    kept: list = []
    problems: list = []
    for raw in hosts or ():
        host = str(raw).strip()
        try:
            ipaddress.IPv4Address(host)
        except ValueError:
            problems.append("%r is not an IPv4 address" % host)
            continue
        if on_course_network(host):
            problems.append("%s is on a RoboCommand course network -- the boat "
                            "must never be pointed there" % host)
            continue
        if host not in kept:
            kept.append(host)
    return kept, problems


class ChangeLog:
    """Logs a message only when its key differs from the last one logged on the
    same channel, so a state that holds is said once and a state that changes is
    said again. A channel with a constant key is "once ever"."""

    _UNSET = object()

    def __init__(self, log) -> None:
        self._log = log
        self._last: dict = {}

    def __call__(self, channel, key, message: str) -> None:
        if self._last.get(channel, self._UNSET) != key:
            self._last[channel] = key
            self._log(message)


class TargetChooser:
    """Which OCS to dial next. Pure: no sockets, and the clock is injectable.

    `hear()` runs on the listener thread, `choose()` on the link thread and only
    while unconnected -- so a beacon heard while connected is recorded but never
    acted on. That is the no-flapping rule.
    """

    def __init__(self, fallback_hosts, port: int, *,
                 fresh_s: float = BEACON_FRESH_S, clock=time.monotonic) -> None:
        self.fallback = list(fallback_hosts)
        self.port = port
        self.fresh_s = fresh_s
        self._clock = clock
        self._lock = threading.Lock()
        self._heard = None              # (ip, tcp_port, when) of the newest beacon
        self._next = 0                  # round-robin cursor into the fallback list

    def hear(self, ip: str, tcp_port: int) -> None:
        with self._lock:
            self._heard = (ip, tcp_port, self._clock())

    def choose(self):
        """(host, port, why), or None if there is neither a fresh beacon nor a
        fallback host. A fallback choice advances the cursor, so a host that does
        not answer is followed by the next one."""
        with self._lock:
            if self._heard and self._clock() - self._heard[2] <= self.fresh_s:
                ip, tcp_port, _ = self._heard
                return ip, tcp_port, "beacon from %s" % ip
            if not self.fallback:
                return None
            host = self.fallback[self._next % len(self.fallback)]
            self._next += 1
            return host, self.port, "fallback %s" % host

    def note_connected(self, host: str) -> None:
        """A fallback host that answered is tried first next time: the OCS that
        was there a moment ago is most likely still there."""
        with self._lock:
            if host in self.fallback:
                self._next = self.fallback.index(host)


class BeaconListener:
    """Receives the OCS beacons on UDP and feeds the ones that are ours to a
    TargetChooser. `handle()` is the whole decision and takes no socket."""

    def __init__(self, port: int, team: str, chooser: TargetChooser, log=print) -> None:
        self.port = port
        self.team = team
        self.chooser = chooser
        self.log = log
        self._once = ChangeLog(log)
        self.heard = 0                  #: beacons accepted
        self.ignored = 0                #: datagrams that were not ours
        self.listening = False
        self.bind_error = ""            #: why the last start() could not bind; "" once bound
        self._sock = None
        self._stop = threading.Event()
        self._thread = None

    def handle(self, data: bytes, ip: str) -> bool:
        """True if `data` from `ip` was one of our OCS's beacons and was recorded."""
        beacon = parse_beacon(data)
        if beacon is None:
            return self._ignore()
        if beacon["team"] != self.team:
            return self._ignore(
                ("team", ip, beacon["team"]),
                "OCS link: beacon from %s ignored -- its team is %r, ours is %r"
                % (ip, beacon["team"], self.team))
        if on_course_network(ip):
            return self._ignore(
                ("course", ip),
                "OCS link: beacon from %s ignored -- that is a RoboCommand "
                "course network and this boat must never be pointed there" % ip)
        self.chooser.hear(ip, beacon["port"])
        self.heard += 1
        return True

    def _ignore(self, channel=None, message: str = "") -> bool:
        """Count a datagram that was not ours; say why once per channel, if asked."""
        self.ignored += 1
        if message:
            self._once(channel, True, message)
        return False

    def start(self) -> bool:
        """Bind and listen. False if the port is taken: the link then runs on its
        fallback hosts alone rather than not at all, and OcsLink tries again every
        LISTEN_RETRY_S. A failure that holds is logged once, not on every try.

        No SO_REUSEADDR, deliberately: two listeners would split the datagrams,
        and the loser sees silence rather than an error.
        """
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.bind(("0.0.0.0", self.port))
        except OSError as exc:
            sock.close()
            if str(exc) != self.bind_error:
                self.log("OCS link: cannot listen for beacons on UDP %d -- %s -- using "
                         "the fallback hosts only, trying again every %g s"
                         % (self.port, exc, LISTEN_RETRY_S))
            self.bind_error = str(exc)
            return False
        if self.bind_error:
            self.log("OCS link: listening for beacons on UDP %d now" % self.port)
            self.bind_error = ""
        sock.settimeout(0.5)
        self._sock = sock
        self.listening = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="ocs-beacon-rx",
                                        daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()
        if self._thread is not None:
            self._thread.join(2.0)
        self.listening = False

    def _run(self) -> None:
        sock = self._sock
        while not self._stop.is_set() and sock is not None:
            try:
                data, (ip, _port) = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError as exc:
                if not self._stop.is_set():
                    self.log("OCS link: beacon listener stopped -- %s" % exc)
                self.listening = False
                return
            self.handle(data, ip)


