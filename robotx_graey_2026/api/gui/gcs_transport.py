"""Crusader gcs_wire transport with a GRAEY HTTP compatibility operation.

Only the vehicle bridge imports/contacts the existing GUI backend. The laptop
needs Python's standard library, one TCP connection, and no ROS/MAVLink.
Mutations are never retried after an ambiguous socket failure.
"""
import argparse
import ast
import http.client
import json
import socket
import socketserver
import select
import logging
from concurrent.futures import ThreadPoolExecutor
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit
from pathlib import Path
from functools import lru_cache

from .gcs_wire import PORT, MAX_BLOB, encode, read_frame, keepalive

SERVICE_PORTS = (8080, 8095, 8096, 8097)


def ocs_request(port,path,method,body=b''):
    """The laptop owns its local OCS; preserve OCS declaration/fleet checks."""
    routes={'/api/run/state':('GET','/api/state'),'/api/run/declare':('POST','/api/declare'),
            '/api/run/end':('POST','/api/end')}
    if path not in routes or routes[path][0]!=method:raise ValueError('Unknown OCS run operation')
    if len(body)>2048:raise ValueError('OCS request too large')
    if method=='POST':
        obj=json.loads(body or b'{}')
        if not isinstance(obj,dict) or set(obj)-{'callsign'}:raise ValueError('Only pilot callsign may accompany run controls')
        if 'callsign' in obj and (not isinstance(obj['callsign'],str) or len(obj['callsign'])>128):raise ValueError('Invalid callsign')
    conn=http.client.HTTPConnection('127.0.0.1',port,timeout=12 if path.startswith(('/api/params','/api/recording')) else 3)
    try:
        conn.request(method,routes[path][1],body if method=='POST' else None,{'Content-Type':'application/json'})
        result=conn.getresponse();data=result.read((1<<20)+1)
        if len(data)>1<<20:raise ValueError('OCS response too large')
        return result.status,'application/json',data
    finally:conn.close()
LINK_BANNER = b'''<script>document.addEventListener('DOMContentLoaded',()=>{const banner=document.createElement('div');banner.style.cssText='padding:6px 12px;background:#243748;color:#fff;font:14px system-ui';document.body.prepend(banner);async function poll(){try{const r=await fetch('/gcs-link',{cache:'no-store'});const s=await r.json();banner.textContent='Laptop to GRAEY: '+(s.connected?'connected':'unavailable')+' | '+(s.host||'waiting for vehicle')+(s.rtt_s==null?'':' | '+Math.round(s.rtt_s*1000)+' ms');}catch(e){banner.textContent='Laptop GCS unavailable'}setTimeout(poll,2000)}poll()})</script>'''


def target(path, backend_port=8090):
    """No arbitrary hosts/URLs/ports in bridge requests."""
    if not isinstance(path, str) or len(path)>4096 or not path.startswith('/') or path.startswith('//'):
        raise ValueError('invalid request path')
    if any(c in path for c in ('\r','\n','\\','#')):
        raise ValueError('invalid request path')
    parsed=urlsplit(path)
    if parsed.scheme or parsed.netloc:raise ValueError('absolute URL refused')
    if path.startswith('/vehicle/'):
        _,_,port,path=path.split('/',3)
        if not port.isdigit() or int(port) not in SERVICE_PORTS:raise ValueError('unknown vehicle service')
        return int(port),'/'+path
    return backend_port,path


def backend_request(header, body, backend_port=8090):
    method=header.get('method')
    if method not in ('GET','POST'):raise ValueError('unsupported HTTP method')
    if len(body)>65536:raise ValueError('request body too large')
    port,path=target(header.get('path'),backend_port)
    conn=http.client.HTTPConnection('127.0.0.1',port,timeout=3)
    headers={'Content-Type':header.get('content_type','application/json'),
             'Origin':f'http://127.0.0.1:{port}'}
    try:
        conn.request(method,path,body=body if method=='POST' else None,headers=headers)
        response=conn.getresponse()
        content_type=response.getheader('Content-Type','application/octet-stream')
        if content_type.startswith('multipart/'):
            # One frame per request; the laptop viewer supplies its own cadence.
            data=bytearray()
            while len(data)<MAX_BLOB:
                block=response.read1(65536)
                if not block:break
                data.extend(block)
                start=data.find(b'\xff\xd8');end=data.find(b'\xff\xd9',max(0,start))
                if start>=0 and end>start:
                    return response.status,'image/jpeg',bytes(data[start:end+2])
            raise ValueError('camera frame unavailable or oversized')
        data=response.read(MAX_BLOB+1)
        if len(data)>MAX_BLOB:raise ValueError('response too large')
        return response.status,content_type,data
    finally:conn.close()


class BridgeHandler(socketserver.BaseRequestHandler):
    def handle(self):
        from .fleet_adapter import FleetAdapter
        from .gcs_connection import Connection
        keepalive(self.request)
        self.adapter=FleetAdapter(lambda header,body:backend_request(header,body,self.server.backend_port))
        self.common=False
        conn=Connection(self.request,self.client_address,lambda c,h,b:self.server.pool.submit(self.dispatch,c,h,b),logging.getLogger('graey_gcs_bridge'))
        try:
            while conn.alive:
                if self.common:
                    for frame in self.adapter.frames():conn.send(frame)
                time.sleep(.5)
        finally:conn.close()

    def dispatch(self,conn,header,blob):
        result=dict(t='result',id=header.get('id'),ok=False)
        try:
            if header.get('t')!='req':raise ValueError('request frame required')
            op=header.get('op')
            if op=='ping':
                self.common=True;result.update(ok=True,data={'vehicle':'GRAEY','protocol':'crusader-gcs'})
            elif op=='http':
                if not blob:blob=header.get('body','').encode('utf-8')
                if header.get('path')=='/vehicle/8080/frame.jpg':header['path']='/vehicle/8080/stream'
                status,kind,data=backend_request(header,blob,self.server.backend_port)
                result.update(ok=True,status=status,content_type=kind);conn.send(result,data);return
            elif op=='record_download':
                self.download(conn,header);return
            else:
                data=self.adapter.operation(header)
                result.update(ok=True,data=data,msg=data.get('msg','') if isinstance(data,dict) else '')
        except (ValueError,OSError,TypeError,AttributeError,http.client.HTTPException) as exc:result['msg']=str(exc)
        conn.send(result)

    def download(self,conn,req):
        from urllib.parse import urlencode
        from .gcs_connection import _ChunkWriter
        connection=http.client.HTTPConnection('127.0.0.1',self.server.backend_port,timeout=10)
        ident=req.get('id');writer=_ChunkWriter(conn,ident)
        try:
            connection.request('GET','/api/recording/archive?'+urlencode(dict(name=req.get('name',''))))
            response=connection.getresponse()
            if response.status!=200:
                conn.send(dict(t='result',id=ident,ok=False,msg=response.read(4096).decode(errors='replace')));return
            while True:
                data=response.read(65536)
                if not data:break
                writer.write(data)
            writer.finish()
            conn.send(dict(t='result',id=ident,ok=True,msg='Recording archive sent',data=dict(name=req.get('name'))))
        except OSError as exc:
            conn.drop_bulk(ident);conn.send(dict(t='result',id=ident,ok=False,msg=str(exc)))
        finally:connection.close()


class BridgeServer(socketserver.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True
    def __init__(self,address,backend_port=8090):
        self.backend_port=backend_port
        self.pool=ThreadPoolExecutor(max_workers=4,thread_name_prefix='graey_gcs_op')
        super().__init__(address,BridgeHandler)
    def server_close(self):
        super().server_close();self.pool.shutdown(wait=False,cancel_futures=True)


class LaptopLink:
    def __init__(self,hosts,port=PORT,clock=time.monotonic):
        self.hosts=[h.strip() for h in hosts.split(',') if h.strip()]
        if not self.hosts:raise ValueError('at least one vehicle hostname/IP required')
        self.port,self.clock=port,clock
        self.lock=threading.Lock();self.sock=None;self.host=None;self.next=0
        self.last_received=None;self.rtt=None;self.error=None

    def request(self,header,body=b''):
        if not self.lock.acquire(timeout=2):raise ConnectionError('vehicle request queue busy')
        try:
            if self.sock is None:
                host=self.hosts[self.next%len(self.hosts)];self.next+=1
                self.sock=socket.create_connection((host,self.port),timeout=1)
                self.sock.settimeout(6);keepalive(self.sock);self.host=host
            ident=uuid.uuid4().hex;start=self.clock()
            self.sock.sendall(encode(dict(header,t='req',id=ident),body))
            result,blob=read_frame(self.sock)
            if result.get('t')!='result' or result.get('id')!=ident:raise ConnectionError('unexpected vehicle response')
            self.last_received=self.clock();self.rtt=self.last_received-start;self.error=None
            if not result.get('ok'):raise ValueError(result.get('msg','vehicle refused request'))
            return result,blob
        except (OSError,ValueError,ConnectionError) as exc:
            if self.sock:self.sock.close()
            self.sock=None;self.error=str(exc)
            raise
        finally:self.lock.release()

    def status(self):
        # Receipt clocks are laptop-local; never subtract a Jetson timestamp.
        age=None if self.last_received is None else self.clock()-self.last_received
        return dict(connected=self.sock is not None and age is not None and age<6,
                    host=self.host,age_s=age,rtt_s=self.rtt,error=self.error)

    def close(self):
        with self.lock:
            if self.sock:self.sock.close()
            self.sock=None


class LaptopHandler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def reply(self,status,kind,data):
        self.send_response(status);self.send_header('Content-Type',kind)
        self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store')
        self.send_header('X-Content-Type-Options','nosniff');self.end_headers();self.wfile.write(data)

    def do_GET(self):self.proxy('GET')
    def do_POST(self):self.proxy('POST')

    def proxy(self,method):
        host=self.headers.get('Host','')
        # Local web pages may operate controls; arbitrary web origins may not.
        if (host not in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')
                or self.headers.get('Sec-Fetch-Site')=='cross-site'
                or (self.headers.get('Origin') and urlsplit(self.headers['Origin']).netloc!=host)):
            self.reply(403,'text/plain',b'Origin or host rejected');return
        try:
            if self.path=='/gcs-link':
                self.reply(200,'application/json',json.dumps(self.server.link.status()).encode());return
            if getattr(self.server,'local_assets',False):
                page=local_asset(urlsplit(self.path).path)
                if page is not None:
                    self.reply(*page);return
            if self.path.startswith('/api/run/'):
                size=int(self.headers.get('Content-Length','0'))
                if not 0<=size<=2048:raise ValueError('OCS request too large')
                if method=='POST' and self.headers.get_content_type()!='application/json':raise ValueError('JSON required')
                self.connection.settimeout(4)
                body=self.rfile.read(size) if method=='POST' else b''
                self.reply(*ocs_request(getattr(self.server,'ocs_port',8099),self.path,method,body));return
            port,path=target(self.path)
            if port==8080 and path in ('/','/stream'):
                if path=='/':
                    page=b'''<!doctype html><title>GRAEY camera</title><style>body{margin:0;background:#111}img{width:100%}</style><img id="frame"><script>let stopped=false;document.addEventListener('visibilitychange',()=>{stopped=document.hidden});async function tick(){try{if(!stopped){let r=await fetch('frame.jpg',{cache:'no-store'});if(r.ok){let u=URL.createObjectURL(await r.blob());let f=document.getElementById('frame');let old=f.src;f.src=u;if(old.startsWith('blob:'))URL.revokeObjectURL(old)}}}finally{setTimeout(tick,333)}}tick()</script>'''
                    self.reply(200,'text/html',page);return
                # Existing overlay viewer expects MJPEG. Fetch only while watched.
                self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.end_headers()
                while True:
                    _,jpeg=self.server.link.request(dict(op='http',method='GET',path='/vehicle/8080/stream'))
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(jpeg)).encode()+b'\r\n\r\n'+jpeg+b'\r\n');self.wfile.flush();time.sleep(.333)
            size=int(self.headers.get('Content-Length','0'))
            if not 0<=size<=65536:raise ValueError('request too large')
            self.connection.settimeout(5)
            body=self.rfile.read(size) if method=='POST' else b''
            requested='/vehicle/8080/stream' if port==8080 and path=='/frame.jpg' else self.path
            result,data=self.server.link.request(dict(op='http',method=method,path=requested,
                content_type=self.headers.get('Content-Type','application/json')),body)
            kind=result['content_type']
            if kind.startswith('text/html'):
                # Keep existing auxiliary service pages on this laptop origin.
                prefix=f'/vehicle/{port}' if self.path.startswith('/vehicle/') else ''
                script=("<script>window.GRAEY_LAPTOP=true;const originalFetch=window.fetch;window.fetch=(url,opts)=>originalFetch(typeof url==='string'&&url.startsWith('/')&&!url.startsWith('/vehicle/')?'"+prefix+"'+url:url,opts);</script>").encode()
                data=data.replace(b'<head>',b'<head>'+script,1) if b'<head>' in data else script+data
                if self.path=='/':data=data.replace(b'</head>',LINK_BANNER+b'</head>',1)
                data=data.replace(b"return 'http://' + location.hostname + ':' + port + '/' + (path || '');",b"return '/vehicle/' + port + '/' + (path || '');")
            self.reply(result['status'],kind,data)
        except (OSError,ValueError,ConnectionError,http.client.HTTPException) as exc:
            try:self.reply(503,'application/json',json.dumps(dict(ok=False,msg='Vehicle outcome unknown; check status before retrying.',detail=str(exc))).encode())
            except OSError:pass


def bridge_main():
    parser=argparse.ArgumentParser();parser.add_argument('--bind',default='0.0.0.0');parser.add_argument('--port',type=int,default=PORT);parser.add_argument('--backend-port',type=int,default=8090)
    args,_=parser.parse_known_args()
    with BridgeServer((args.bind,args.port),args.backend_port) as server:server.serve_forever()


def legacy_laptop_main():
    parser=argparse.ArgumentParser();parser.add_argument('--host',default='graey.local');parser.add_argument('--vehicle-port',type=int,default=PORT);parser.add_argument('--port',type=int,default=8152);parser.add_argument('--ocs-port',type=int,default=8099)
    args=parser.parse_args();link=LaptopLink(args.host,args.vehicle_port)
    try:
        with ThreadingHTTPServer(('127.0.0.1',args.port),LaptopHandler) as server:
            server.link=link;server.ocs_port=args.ocs_port;server.local_assets=True
            print(f'GRAEY GCS http://127.0.0.1:{args.port} -> {args.host}:{args.vehicle_port}',flush=True)
            server.serve_forever()
    finally:link.close()


@lru_cache(maxsize=1)
def local_ui():
    """Load trusted page metadata without importing ROS on the laptop."""
    folder=Path(__file__).resolve().parents[3]
    tree=ast.parse((folder/'robotx_graey_2026/api/gui/gui_node.py').read_text(encoding='utf-8'))
    values={}
    for node in tree.body:
        if isinstance(node,ast.Assign):
            for target in node.targets:
                if isinstance(target,ast.Name) and target.id in ('FILES','TYPES','CONTROL_PANEL'):
                    values[target.id]=ast.literal_eval(node.value)
    namespace={}
    function=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='page_navigation')
    exec(compile(ast.Module(body=[function],type_ignores=[]),'page_navigation','exec'),namespace)
    assets=folder/'tools'
    if not assets.is_dir():assets=Path(__file__).resolve().parents[6]/'share/robotx_graey_2026/tools'
    return assets,values,namespace['page_navigation']


def local_asset(path):
    folder,values,navigation=local_ui();name=values['FILES'].get(path)
    if not name:return None
    file=(folder/name).resolve()
    if folder.resolve() not in file.parents or not file.is_file():return None
    data=file.read_bytes();kind=values['TYPES'].get(file.suffix,'application/octet-stream')
    if file.suffix=='.html':
        text=data.decode('utf-8')
        if path=='/':
            marker='<section id="overview" style="display:none">'
            text=text.replace(marker,marker+values['CONTROL_PANEL'],1)
        text=navigation(text,path)
        text=text.replace("return 'http://' + location.hostname + ':' + port + '/' + (path || '');","return '/vehicle/' + port + '/' + (path || '');")
        if path=='/':text=text.replace('</head>',LINK_BANNER.decode()+'</head>',1)
        data=text.encode('utf-8')
    return 200,kind,data


def laptop_main():
    from .laptop.graey_app import main
    main()
