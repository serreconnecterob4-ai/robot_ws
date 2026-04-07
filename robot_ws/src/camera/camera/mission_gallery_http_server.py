import argparse
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse


ALLOWED_EXTENSIONS = (
    '.jpg', '.jpeg', '.png', '.bmp', '.gif',
    '.mp4', '.avi', '.mov', '.webm', '.mkv'
)


def _safe_gallery_files(gallery_dir):
    if not os.path.isdir(gallery_dir):
        return []

    files = []
    for name in os.listdir(gallery_dir):
        path = os.path.join(gallery_dir, name)
        if not os.path.isfile(path):
            continue
        if not name.lower().endswith(ALLOWED_EXTENSIONS):
            continue
        files.append(name)

    files.sort(key=lambda x: os.path.getmtime(os.path.join(gallery_dir, x)), reverse=True)
    return files


class MissionGalleryHandler(BaseHTTPRequestHandler):
    gallery_dir = ''

    def log_message(self, format, *args):
        return

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_file(self, path):
        try:
            size = os.path.getsize(path)
            with open(path, 'rb') as f:
                self.send_response(200)
                self.send_header('Content-Type', 'application/octet-stream')
                self.send_header('Content-Length', str(size))
                self.send_header('Cache-Control', 'no-cache')
                self.end_headers()
                while True:
                    chunk = f.read(64 * 1024)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
        except FileNotFoundError:
            self._send_json({'error': 'file not found'}, status=404)
        except Exception as exc:
            self._send_json({'error': f'cannot read file: {exc}'}, status=500)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path or '/'

        if path == '/health':
            self._send_json({'ok': True})
            return

        if path == '/list':
            files = _safe_gallery_files(self.gallery_dir)
            self._send_json({'files': files})
            return

        if path.startswith('/files/'):
            raw_name = unquote(path[len('/files/'):])
            name = os.path.basename(raw_name)
            if not name or name != raw_name:
                self._send_json({'error': 'invalid filename'}, status=400)
                return
            if not name.lower().endswith(ALLOWED_EXTENSIONS):
                self._send_json({'error': 'unsupported extension'}, status=400)
                return

            file_path = os.path.join(self.gallery_dir, name)
            self._send_file(file_path)
            return

        self._send_json({'error': 'not found'}, status=404)


def main():
    parser = argparse.ArgumentParser(description='Expose robot mission gallery over HTTP.')
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=int(os.environ.get('MISSION_GALLERY_HTTP_PORT', '8092')))
    parser.add_argument(
        '--gallery-dir',
        default=os.environ.get('MISSION_GALLERY_DIR', os.path.expanduser('~/mission_gallery')),
    )
    args = parser.parse_args()

    os.makedirs(args.gallery_dir, exist_ok=True)
    MissionGalleryHandler.gallery_dir = args.gallery_dir

    server = ThreadingHTTPServer((args.host, args.port), MissionGalleryHandler)
    print(f'Mission gallery HTTP server listening on {args.host}:{args.port} dir={args.gallery_dir}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
