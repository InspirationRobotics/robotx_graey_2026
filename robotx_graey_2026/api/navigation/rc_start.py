"""One-shot RC start edge used by the adapted Chris Simple Dive.

This copies the proven prequal operator gesture, not Chris's radio transport:
SE must first be observed low, then high.  RF loss after that edge is irrelevant;
the mission executor owns the run onboard.
"""


class RcStartEdge:
    def __init__(self, channel=9, high=1700):
        if type(channel) is not int or not 9 <= channel <= 18:
            raise ValueError('RC start channel must be 9..18')
        if type(high) is not int or not 800 < high < 2200:
            raise ValueError('RC high threshold must be 801..2199')
        self.channel = channel
        self.high = high
        self.seen_low = False
        self.started = False
        self.pwm = None

    def update(self, message):
        """Return True exactly once, on a valid low-to-high SE transition."""
        value = getattr(message, f'chan{self.channel}_raw', None)
        if type(value) is not int or not 800 <= value <= 2200:
            return False
        self.pwm = value
        if value < self.high:
            self.seen_low = True
        if not self.started and self.seen_low and value > self.high:
            self.started = True
            return True
        return False

    def snapshot(self):
        return dict(channel=self.channel, high=self.high, pwm=self.pwm,
                    low_seen=self.seen_low, started=self.started)
