#!/usr/bin/env python3
"""Loopback entry point for both robots in the same Gazebo world."""
import argparse
import socket
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import urllib.request
import urllib.error


def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8765)
    p.add_argument('--check-ports',action='store_true')
    args=p.parse_args()
    if args.check_ports:
        for port in [args.port,8766,8767]:
            with socket.socket() as sock:
                sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                try:sock.bind(('127.0.0.1',port))
                except OSError:raise SystemExit(f'Port {port} is already in use; stop the previous click navigation instance.')
        return
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*unused):pass
        def dispatch(self):
            if self.path=='/':
                self.send_response(302);self.send_header('Location','/dog/');self.end_headers();return
            parts=self.path.split('/',2)
            if len(parts)!=3 or parts[1] not in ('dog','drone'):
                self.send_error(404);return
            origin=self.headers.get('Origin')
            if self.command=='POST' and origin and origin not in (f'http://localhost:{args.port}',f'http://127.0.0.1:{args.port}'):
                self.send_error(403);return
            size=int(self.headers.get('Content-Length','0'))
            if size<0 or size>4096:self.send_error(413);return
            body=self.rfile.read(size) if self.command=='POST' else None
            url=f"http://127.0.0.1:{8766 if parts[1]=='dog' else 8767}/"+parts[2]
            request=urllib.request.Request(url,data=body,method=self.command,
                headers={'Content-Type':self.headers.get('Content-Type','application/json')})
            try:
                try:r=urllib.request.urlopen(request,timeout=8)
                except urllib.error.HTTPError as e:r=e
                with r:
                    data=r.read();self.send_response(r.code)
                    self.send_header('Content-Type',r.headers.get('Content-Type','application/json'))
                    self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store')
                    self.end_headers();self.wfile.write(data)
            except (OSError,urllib.error.URLError):
                if not parts[2]:
                    data=b'<meta charset="utf-8"><meta http-equiv="refresh" content="2"><title>Starting navigation</title><p>Robot service is starting. This page will refresh automatically.</p>'
                    self.send_response(200);self.send_header('Content-Type','text/html; charset=utf-8')
                    self.end_headers();self.wfile.write(data)
                else:self.send_error(503,'Robot service is starting or unavailable')
        do_GET=dispatch
        do_POST=dispatch
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler);server.daemon_threads=True
    print(f'CLICK_NAV_SHARED http://localhost:{args.port}/dog/ and /drone/',flush=True)
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:server.server_close()


if __name__=='__main__':main()
