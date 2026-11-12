"""GRAEY mission/sensor adapters around the UNCHANGED Crusader laptop app."""
import argparse
import json
import os
import re
import sys
from pathlib import Path
from urllib.parse import urlsplit

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE));sys.path.insert(0,str(HERE.parents[3]))
import gcs_app as common
from robotx_graey_2026.api.gui.gcs_transport import local_asset,ocs_request

factory=common.make_handler
radio_client=None


def make_handler(link,hub,allowed_hosts,missions=None):
    base=factory(link,hub,allowed_hosts,missions)
    class GraeyHandler(base):
        def adaptation(self,method):
            path=urlsplit(self.path).path
            mine=path=='/' or path in ('/graey-adapter.js','/graey-tools','/crusader-theme.css','/graey-theme.js') or path.startswith(('/api/','/vehicle/')) or path in ('/missions','/waypoint','/pool-map','/planner','/crusader-map.js','/sensors','/record','/mission.css','/missions.js','/sensors.js','/record.js','/station.css','/station_theme.js','/vehicle-controls','/calibration')
            if not mine:return False
            if not self._allowed():return True
            origin=self.headers.get('Origin')
            if self.headers.get('Sec-Fetch-Site')=='cross-site' or (origin and urlsplit(origin).netloc!=self.headers.get('Host')):
                self._send(403,b'Origin rejected','text/plain');return True
            try:
                if method=='GET' and path=='/':
                    page=(HERE/'page.html').read_bytes().replace(b'</body>',b'<script src="/graey-adapter.js"></script></body>')
                    self._send(200,page,'text/html; charset=utf-8');return True
                if method=='GET' and path in ('/graey-adapter.js','/graey-tools','/crusader-theme.css','/graey-theme.js'):
                    name={'/graey-adapter.js':'graey_adapter.js','/graey-tools':'graey_tools.html','/crusader-theme.css':'crusader_theme.css','/graey-theme.js':'graey_theme.js'}[path]
                    self._send(200,(HERE/name).read_bytes(),'application/javascript' if path.endswith('.js') else 'text/css' if path.endswith('.css') else 'text/html');return True
                if method=='GET':
                    asset=local_asset(path)
                    if asset is not None:
                        status,kind,data=asset
                        if kind.startswith('text/html'):data=data.replace(b'/station.css',b'/crusader-theme.css').replace(b'/station_theme.js',b'/graey-theme.js')
                        self._send(status,data,kind);return True
                n=int(self.headers.get('Content-Length','0'))
                if not 0<=n<=65536:raise ValueError('Request too large')
                self.connection.settimeout(5)
                body=self.rfile.read(n) if method=='POST' else b''
                if path=='/api/radio/mission':
                    if radio_client is None:raise ValueError('RFD mission client is unavailable')
                    if method=='GET':status,kind,data=200,'application/json',json.dumps(dict(ok=True,**radio_client.status())).encode()
                    else:
                        result=radio_client.request(json.loads(body.decode('utf-8')))
                        status,kind,data=200,'application/json',json.dumps(result).encode()
                elif path.startswith('/api/run/'):
                    status,kind,data=ocs_request(int(os.environ.get('GRAEY_OCS_PORT','8099')),path,method,body)
                else:
                    header,data=link.request('http',dict(method=method,path=self.path,body=body.decode('utf-8'),content_type=self.headers.get('Content-Type','application/json')))
                    if not header.get('ok'):raise ValueError(header.get('msg','GRAEY unavailable'))
                    status,kind=header['status'],header['content_type']
                    if kind.startswith('text/html') and path.startswith('/vehicle/'):
                        prefix='/'.join(path.split('/')[:3])
                        data=re.sub(rb'''(["'])/([A-Za-z0-9_][^"']*)\1''',lambda m:m[1]+prefix.encode()+b'/'+m[2]+m[1],data)
                        script=("<script>const f=window.fetch;window.fetch=(u,o)=>f(typeof u==='string'&&u.startsWith('/')&&!u.startsWith('/vehicle/')?'"+prefix+"'+u:u,o)</script>").encode()
                        data=script+data
                if kind.startswith('text/html'):data=data.replace(b'/station.css',b'/crusader-theme.css').replace(b'/station_theme.js',b'/graey-theme.js')
                self._send(status,data,kind)
            except (ValueError,OSError) as exc:self._json(dict(ok=False,msg=str(exc)),503)
            return True
        def do_GET(self):
            if not self.adaptation('GET'):super().do_GET()
        def do_POST(self):
            if not self.adaptation('POST'):super().do_POST()
    return GraeyHandler


def main(argv=None):
    global radio_client
    common.make_handler=make_handler
    os.environ.setdefault('CRSD_BOAT',os.environ.get('GRAEY_HOST','graey.local'))
    common.DEFAULT_BOAT=os.environ['CRSD_BOAT']
    common.__doc__=common.__doc__.replace('8150','8152').replace('the boat','GRAEY')
    # ADAPTED: submarine-facing names; copied Crusader internals remain unchanged.
    parser=argparse.ArgumentParser(description='GRAEY laptop ground station')
    parser.add_argument('--graey',default=common.DEFAULT_BOAT,help='GRAEY host/IP, or comma-separated hosts (default: %(default)s)')
    parser.add_argument('--graey-port',type=int,default=14559,help='GRAEY GCS bridge TCP port (default: %(default)s)')
    parser.add_argument('--port',type=int,default=8152,help='Laptop page port (default: %(default)s)')
    parser.add_argument('--radio-port',type=int,default=14572,
                        help='fleet_link GRAEY mission output port (default: %(default)s)')
    parser.add_argument('--no-browser',action='store_true')
    # Hidden compatibility aliases for previously documented commands.
    parser.add_argument('--boat',dest='graey',default=argparse.SUPPRESS,help=argparse.SUPPRESS)
    parser.add_argument('--boat-port',dest='graey_port',type=int,default=argparse.SUPPRESS,help=argparse.SUPPRESS)
    args=parser.parse_args(sys.argv[1:] if argv is None else argv)
    from robotx_graey_2026.api.gui.laptop.radio_client import RadioMissionClient
    radio_client=RadioMissionClient(args.radio_port)
    try:
        common.main(['--boat',args.graey,'--boat-port',str(args.graey_port),'--port',str(args.port),
                     *(['--no-browser'] if args.no_browser else [])])
    finally:radio_client.close()


if __name__=='__main__':main()
