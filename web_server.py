import os
import sys
import json
import time
import threading
import urllib.parse
import base64
from http import HTTPStatus
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from io import BytesIO

# Optional dependencies used if available
try:
    import mss
    import mss.tools
except Exception:
    mss = None

try:
    import pyautogui
except Exception:
    pyautogui = None

# Optional system process metrics
try:
    import psutil
except Exception:
    psutil = None

# Optional Pillow for JPEG/WebP encoding
try:
    from PIL import Image, features as PIL_features
except Exception:
    Image = None
    PIL_features = None

# Resolve project root for both source and frozen builds
try:
    if getattr(sys, 'frozen', False):
        # In PyInstaller builds, use the executable directory
        PROJECT_ROOT = os.path.dirname(sys.executable)
    else:
        PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))
except Exception:
    PROJECT_ROOT = os.path.abspath('.')
SERVER_CONFIG_PATH = os.path.join(PROJECT_ROOT, 'server_config.json')


def load_server_config():
    cfg = {
        "bind": "127.0.0.1",
        "port": 8765,
        "token": "CHANGE_ME_TOKEN",
        "mjpeg_fps": 6,
    }
    try:
        if os.path.exists(SERVER_CONFIG_PATH):
            with open(SERVER_CONFIG_PATH, 'r') as f:
                user_cfg = json.load(f)
            if isinstance(user_cfg, dict):
                cfg.update(user_cfg)
    except Exception:
        pass
    # Allow env override
    cfg['bind'] = os.environ.get('WEB_BIND', cfg['bind'])
    cfg['port'] = int(os.environ.get('WEB_PORT', cfg['port']))
    cfg['token'] = os.environ.get('WEB_TOKEN', cfg['token'])
    return cfg


def read_config_json():
    path = os.path.join(PROJECT_ROOT, 'config.json')
    try:
        if os.path.exists(path):
            with open(path, 'r') as f:
                return json.load(f)
    except Exception:
        return {}
    return {}


def bring_app_to_foreground(window_title: str = "Image Detection Bot") -> bool:
    """Bring the bot GUI to foreground so F5/F8 are received by the app.
    Returns True if a window was focused, False otherwise.
    """
    try:
        import ctypes
        user32 = ctypes.windll.user32
        # Try FindWindow by title
        FindWindowW = user32.FindWindowW
        SetForegroundWindow = user32.SetForegroundWindow
        ShowWindow = user32.ShowWindow
        SW_RESTORE = 9
        hwnd = FindWindowW(None, window_title)
        if hwnd:
            # Restore if minimized and bring to front
            ShowWindow(hwnd, SW_RESTORE)
            SetForegroundWindow(hwnd)
            return True
        return False
    except Exception:
        return False


def write_config_json(new_cfg: dict) -> None:
    cfg_path = os.path.join(PROJECT_ROOT, 'config.json')
    # Backup current config
    try:
        if os.path.exists(cfg_path):
            import shutil, time as _time
            # Save backups under a dedicated folder: "backup configs"
            backup_dir = os.path.join(PROJECT_ROOT, 'backup configs')
            try:
                os.makedirs(backup_dir, exist_ok=True)
            except Exception:
                pass
            backup = os.path.join(backup_dir, f'config.backup.{int(_time.time())}.json')
            shutil.copyfile(cfg_path, backup)
            # Prune old backups to avoid folder flooding (configurable)
            try:
                # Read retention from server config (default 20)
                MAX_BACKUPS = int(load_server_config().get('backup_retention', 20))
                files = [f for f in os.listdir(backup_dir) if f.startswith('config.backup.') and f.endswith('.json')]
                def _ts(name):
                    try:
                        # config.backup.<ts>.json
                        return int(name.split('.')[2])
                    except Exception:
                        # Fallback to mtime
                        return int(os.path.getmtime(os.path.join(backup_dir, name)))
                files.sort(key=_ts, reverse=True)
                for old in files[MAX_BACKUPS:]:
                    try:
                        os.remove(os.path.join(backup_dir, old))
                    except Exception:
                        pass
            except Exception:
                pass
    except Exception:
        pass
    with open(cfg_path, 'w', encoding='utf-8') as f:
        json.dump(new_cfg, f, indent=2)


def trigger_reload_config_ipc():
    cmd_path = os.path.join(PROJECT_ROOT, 'ipc_command.json')
    try:
        with open(cmd_path, 'w', encoding='utf-8') as f:
            json.dump({"command": "reload_config"}, f)
    except Exception:
        pass


class WebHandler(BaseHTTPRequestHandler):
    server_version = "BotWeb/0.1"

    def _cfg(self):
        return self.server._cfg

    def _write_json(self, obj, status=HTTPStatus.OK):
        data = json.dumps(obj).encode('utf-8')
        try:
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
            # Client disconnected; avoid noisy traceback
            try:
                self.close_connection = True
            except Exception:
                pass
            return

    def _check_auth(self):
        # Accept token via Authorization: Bearer <token> or query param ?token=<token>
        expected = self._cfg().get('token') or ''
        # Header
        auth = self.headers.get('Authorization')
        if auth and auth.startswith('Bearer '):
            if auth.split(' ', 1)[1].strip() == expected:
                return True
        # Query param
        try:
            parsed = urllib.parse.urlparse(self.path)
            qs = urllib.parse.parse_qs(parsed.query)
            if 'token' in qs and qs['token'][0] == expected:
                return True
        except Exception:
            pass
        return False

    def _serve_static_file(self, rel_path):
        # Sanitize
        safe_path = rel_path.replace('..', '').replace('\\', '/').strip('/')
        primary = os.path.join(PROJECT_ROOT, 'web', 'static', safe_path)
        alt = os.path.join(PROJECT_ROOT, '_internal', 'web', 'static', safe_path)
        full_path = primary if os.path.isfile(primary) else alt
        if not os.path.isfile(full_path):
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        ctype = 'text/plain'
        if full_path.endswith('.html'):
            ctype = 'text/html; charset=utf-8'
        elif full_path.endswith('.js'):
            ctype = 'application/javascript'
        elif full_path.endswith('.css'):
            ctype = 'text/css'
        elif full_path.endswith('.png'):
            ctype = 'image/png'
        elif full_path.endswith('.jpg') or full_path.endswith('.jpeg'):
            ctype = 'image/jpeg'
        self.send_response(HTTPStatus.OK)
        self.send_header('Content-Type', ctype)
        fs = os.stat(full_path)
        self.send_header('Content-Length', str(fs.st_size))
        self.end_headers()
        with open(full_path, 'rb') as f:
            self.wfile.write(f.read())

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if path == '/' or path == '/index.html':
            return self._serve_static_file('index.html')
        if path.startswith('/static/'):
            return self._serve_static_file(path[len('/static/'):])

        # Token validation (no auth required): quickly check whether provided token matches server config
        if path == '/api/status/check':
            try:
                expected = self._cfg().get('token') or ''
                # Prefer header token; fallback to query param
                auth = self.headers.get('Authorization') or ''
                tok = ''
                if auth.startswith('Bearer '):
                    tok = auth.split(' ', 1)[1].strip()
                else:
                    qs = urllib.parse.parse_qs(parsed.query)
                    tok = (qs.get('token') or [''])[0]
                return self._write_json({
                    "valid": bool(tok) and (tok == expected),
                    "time": int(time.time())
                })
            except Exception as e:
                return self._write_json({"valid": False, "error": str(e)}, HTTPStatus.OK)

        # Auth-required endpoints
        if path == '/api/config':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            cfg = read_config_json()
            return self._write_json(cfg)

        if path == '/api/sequences':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            cfg = read_config_json()
            names = [s.get('name', 'Unnamed Sequence') for s in cfg.get('sequences', [])]
            return self._write_json({"sequences": names})
        # Break settings
        if path == '/api/break-settings':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            cfg = read_config_json()
            br = cfg.get('break_settings', {})
            return self._write_json({
                'enabled': bool(br.get('enabled', False)),
                'max_runtime_seconds': int(br.get('max_runtime_seconds', 0)),
                'run_final_after_break': bool(br.get('run_final_after_break', False)),
                'final_sequence_name': br.get('final_sequence_name', '(none)')
            })

        # Failsafe settings
        if path == '/api/failsafe':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            cfg = read_config_json()
            fs = cfg.get('failsafe', {}) or {}
            return self._write_json({
                'enabled': bool(fs.get('enabled', False)),
                'template_name': fs.get('template_name') or fs.get('template'),
                'confidence': fs.get('confidence', 0.8),
                'region': fs.get('region')
            })
        # Failsafe sequence (list steps)
        if path == '/api/failsafe/sequence':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            cfg = read_config_json()
            fs = cfg.get('failsafe', {}) or {}
            steps = fs.get('sequence', []) or []
            return self._write_json({ 'steps': steps })
        # Get a specific sequence by name
        if path.startswith('/api/sequences/'):
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            name = urllib.parse.unquote(path[len('/api/sequences/'):])
            cfg = read_config_json()
            seq = next((s for s in cfg.get('sequences', []) if s.get('name') == name), None)
            if not seq:
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            return self._write_json(seq)

        if path == '/api/status':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            # Enriched status including uptime and last-run snapshot
            uptime = 0
            try:
                uptime = int(time.time() - (getattr(self.server, '_start_ts', time.time())))
            except Exception:
                uptime = 0
            return self._write_json({
                "status": "ok",
                "time": int(time.time()),
                "uptime": uptime,
                "last_run": getattr(self.server, '_last_run', {
                    "status": "idle",
                    "sequence": None,
                    "group": None,
                    "last_event_ts": None
                })
            })

        # Lightweight ping for latency measurement
        if path == '/api/ping':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            qs = urllib.parse.parse_qs(parsed.query)
            echo = (qs.get('echo') or [''])[0]
            return self._write_json({"time": int(time.time()), "echo": echo})

        # Metrics endpoint: basic server/process metrics and connection counts
        if path == '/api/metrics':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            try:
                now = int(time.time())
                uptime = int(now - (getattr(self.server, '_start_ts', now)))
                # Aggregate app-only stats: current process + descendants
                app_stats = None
                try:
                    if psutil is not None:
                        proc = getattr(self.server, '_proc', None)
                        if proc is None:
                            proc = psutil.Process(os.getpid())
                        # Non-blocking snapshot; values stabilize over time
                        cpu_main = float(proc.cpu_percent(interval=None) or 0.0)
                        mem_main = int(proc.memory_info().rss or 0)
                        children = proc.children(recursive=True)
                        cpu_children = 0.0
                        mem_children = 0
                        for ch in children:
                            try:
                                cpu_children += float(ch.cpu_percent(interval=None) or 0.0)
                                mem_children += int(ch.memory_info().rss or 0)
                            except Exception:
                                continue
                        app_stats = {
                            "cpu_percent": round(cpu_main + cpu_children, 1),
                            "mem_rss_bytes": int(mem_main + mem_children),
                            "process_count": int(1 + len(children)),
                            # Optional breakdown
                            "cpu_main_percent": round(cpu_main, 1),
                            "cpu_children_percent": round(cpu_children, 1),
                            "mem_main_rss_bytes": int(mem_main),
                            "mem_children_rss_bytes": int(mem_children),
                        }
                except Exception:
                    app_stats = None
                data = {
                    "time": now,
                    "uptime": uptime,
                    "connections": {
                        "status_sse": int(getattr(self.server, '_sse_clients', 0)),
                        "preview_mjpeg": int(getattr(self.server, '_mjpeg_clients', 0)),
                        "log_sse": int(getattr(self.server, '_log_clients', 0)),
                    },
                    "totals": {
                        "status_sse_messages": int(getattr(self.server, '_sse_messages_total', 0)),
                        "preview_mjpeg_frames": int(getattr(self.server, '_mjpeg_frames_total', 0)),
                        "log_sse_messages": int(getattr(self.server, '_log_messages_total', 0)),
                    },
                    "app": app_stats,
                    "capabilities": {
                        "mss_available": bool(mss is not None),
                        "pyautogui_available": bool(pyautogui is not None),
                        "pillow_available": bool('Image' in globals() and Image is not None),
                        "jpeg_supported": bool('PIL_features' in globals() and PIL_features is not None and (
                            bool(PIL_features.check('jpg')) or bool(PIL_features.check('jpeg'))
                        )),
                        "psutil_available": bool(psutil is not None),
                    },
                    "last_run": getattr(self.server, '_last_run', {
                        "status": "idle",
                        "sequence": None,
                        "group": None,
                        "last_event_ts": None
                    })
                }
                return self._write_json(data)
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Server-Sent Events: live status stream (heartbeat + last-run info)
        if path == '/api/status/stream':
            if not self._check_auth():
                self.send_error(HTTPStatus.UNAUTHORIZED)
                return
            # Interval control via query param (milliseconds)
            qs = urllib.parse.parse_qs(parsed.query)
            try:
                interval_ms = int((qs.get('interval_ms') or ['1000'])[0])
            except Exception:
                interval_ms = 1000
            interval_ms = max(250, min(interval_ms, 10000))
            counted = False
            try:
                try:
                    self.server._sse_clients = int(getattr(self.server, '_sse_clients', 0)) + 1
                    counted = True
                except Exception:
                    pass
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                # Stream loop
                while True:
                    try:
                        uptime = int(time.time() - (getattr(self.server, '_start_ts', time.time())))
                    except Exception:
                        uptime = 0
                    payload = {
                        "heartbeat": int(time.time()),
                        "uptime": uptime,
                        "last_run": getattr(self.server, '_last_run', {
                            "status": "idle",
                            "sequence": None,
                            "group": None,
                            "last_event_ts": None
                        })
                    }
                    data = json.dumps(payload)
                    try:
                        self.wfile.write(b'data: ' + data.encode('utf-8') + b'\n\n')
                        # Attempt to flush to client
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                        try:
                            self.server._sse_messages_total = int(getattr(self.server, '_sse_messages_total', 0)) + 1
                        except Exception:
                            pass
                    except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                        # Client disconnected
                        try:
                            self.close_connection = True
                        except Exception:
                            pass
                        return
                    time.sleep(interval_ms / 1000.0)
            except Exception:
                # Any other error: stop streaming silently
                return
            finally:
                if counted:
                    try:
                        self.server._sse_clients = int(getattr(self.server, '_sse_clients', 1)) - 1
                    except Exception:
                        pass

        # Debug log tail
        if path == '/api/debug-log':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            try:
                qs = urllib.parse.parse_qs(parsed.query)
                try:
                    max_lines = int((qs.get('lines') or ['500'])[0])
                except Exception:
                    max_lines = 500
                max_lines = max(1, min(max_lines, 5000))
                log_path = os.path.join(PROJECT_ROOT, 'bot_debug.log')
                if not os.path.exists(log_path):
                    return self._write_json({"exists": False, "lines": []})
                # Read tail lines (simple approach, acceptable for moderate sizes)
                with open(log_path, 'r', encoding='utf-8', errors='ignore') as f:
                    content = f.read()
                lines = content.splitlines()
                tail = lines[-max_lines:]
                return self._write_json({
                    "exists": True,
                    "lines": tail,
                    "count": len(tail),
                    "total": len(lines)
                })
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Debug log SSE stream: periodically send tail lines
        if path == '/api/debug-log/stream':
            if not self._check_auth():
                self.send_error(HTTPStatus.UNAUTHORIZED)
                return
            qs = urllib.parse.parse_qs(parsed.query)
            try:
                interval_ms = int((qs.get('interval_ms') or ['1000'])[0])
            except Exception:
                interval_ms = 1000
            interval_ms = max(250, min(interval_ms, 10000))
            try:
                max_lines = int((qs.get('lines') or ['500'])[0])
            except Exception:
                max_lines = 500
            max_lines = max(1, min(max_lines, 5000))
            log_path = os.path.join(PROJECT_ROOT, 'bot_debug.log')
            counted = False
            try:
                try:
                    self.server._log_clients = int(getattr(self.server, '_log_clients', 0)) + 1
                    counted = True
                except Exception:
                    pass
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/event-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Connection', 'keep-alive')
                self.end_headers()
                while True:
                    exists = os.path.exists(log_path)
                    lines = []
                    total = 0
                    if exists:
                        try:
                            with open(log_path, 'r', encoding='utf-8', errors='ignore') as f:
                                content = f.read()
                            lines = content.splitlines()
                            total = len(lines)
                            lines = lines[-max_lines:]
                        except Exception:
                            exists = False
                            lines = []
                            total = 0
                    payload = {
                        "time": int(time.time()),
                        "exists": bool(exists),
                        "count": len(lines),
                        "total": int(total),
                        "lines": lines,
                    }
                    data = json.dumps(payload)
                    try:
                        self.wfile.write(b'data: ' + data.encode('utf-8') + b'\n\n')
                        try:
                            self.wfile.flush()
                        except Exception:
                            pass
                        try:
                            self.server._log_messages_total = int(getattr(self.server, '_log_messages_total', 0)) + 1
                        except Exception:
                            pass
                    except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                        try:
                            self.close_connection = True
                        except Exception:
                            pass
                        return
                    time.sleep(interval_ms / 1000.0)
            except Exception:
                return
            finally:
                if counted:
                    try:
                        self.server._log_clients = int(getattr(self.server, '_log_clients', 1)) - 1
                    except Exception:
                        pass

        # Scheduled sequences
        if path == '/api/schedules':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            cfg = read_config_json()
            return self._write_json({"schedules": cfg.get('scheduled_sequences', [])})

        # Groups listing
        if path == '/api/groups':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            try:
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                names = sorted(list(groups.keys()))
                return self._write_json({"groups": names})
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Group details
        if path.startswith('/api/groups/'):
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            try:
                name = urllib.parse.unquote(path[len('/api/groups/'):])
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                g = groups[name]
                # Support both dict-based groups and list-based groups (steps-only)
                if isinstance(g, list):
                    out = {
                        'name': name,
                        'steps': g,
                        'loop': False,
                        'loop_count': 1
                    }
                else:
                    out = {
                        'name': name,
                        'steps': (g.get('steps') or []),
                        'loop': bool(g.get('loop', False)),
                        'loop_count': int(g.get('loop_count', 1))
                    }
                return self._write_json(out)
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Monitors listing (0 = All, then 1..N)
        if path == '/api/monitors':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            try:
                mons = []
                if mss is None:
                    return self._write_json({"monitors": mons})
                with mss.mss() as sct:
                    for idx, mon in enumerate(sct.monitors):
                        mons.append({
                            "index": idx,
                            "left": mon.get('left', 0),
                            "top": mon.get('top', 0),
                            "width": mon.get('width', 0),
                            "height": mon.get('height', 0)
                        })
                return self._write_json({"monitors": mons})
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Templates listing
        if path == '/api/templates':
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            cfg = read_config_json()
            tpl_map = cfg.get('templates', {})
            out = []
            for name, p in tpl_map.items():
                full = os.path.join(PROJECT_ROOT, p) if not os.path.isabs(p) else p
                out.append({
                    'name': name,
                    'path': p,
                    'exists': bool(full and os.path.exists(full))
                })
            return self._write_json({'templates': out})

        # Template detail
        if path.startswith('/api/templates/'):
            if not self._check_auth():
                return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)
            name = urllib.parse.unquote(path[len('/api/templates/'):])
            cfg = read_config_json()
            tpl_map = cfg.get('templates', {})
            p = tpl_map.get(name)
            if p is None:
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            full = os.path.join(PROJECT_ROOT, p) if not os.path.isabs(p) else p
            return self._write_json({'name': name, 'path': p, 'exists': bool(full and os.path.exists(full))})

        # Serve template image preview
        if path == '/api/template-image':
            if not self._check_auth():
                self.send_error(HTTPStatus.UNAUTHORIZED)
                return
            qs = urllib.parse.parse_qs(parsed.query)
            name = (qs.get('name') or [None])[0]
            if not name:
                self.send_error(HTTPStatus.BAD_REQUEST, 'name required')
                return
            cfg = read_config_json()
            p = (cfg.get('templates', {}) or {}).get(name)
            if not p:
                self.send_error(HTTPStatus.NOT_FOUND, 'template not found')
                return
            full = os.path.join(PROJECT_ROOT, p) if not os.path.isabs(p) else p
            if not (full and os.path.exists(full)):
                self.send_error(HTTPStatus.NOT_FOUND, 'image file not found')
                return
            ctype = 'image/png'
            if full.lower().endswith('.jpg') or full.lower().endswith('.jpeg'):
                ctype = 'image/jpeg'
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', ctype)
            fs = os.stat(full)
            self.send_header('Content-Length', str(fs.st_size))
            self.end_headers()
            with open(full, 'rb') as f:
                self.wfile.write(f.read())

        if path == '/stream.mjpeg':
            if not self._check_auth():
                self.send_error(HTTPStatus.UNAUTHORIZED)
                return
            # MJPEG multipart stream
            if mss is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "mss not available")
                return
            # FPS target (allow override via query ?fps=<int>, else use config)
            qs_for_fps = urllib.parse.parse_qs(parsed.query)
            try:
                fps_override = int((qs_for_fps.get('fps') or [str(self._cfg().get('mjpeg_fps', 6))])[0])
            except Exception:
                fps_override = int(self._cfg().get('mjpeg_fps', 6))
            fps = max(1, min(fps_override, 60))
            delay = 1.0 / float(fps)
            boundary = 'frameboundary'
            self.send_response(HTTPStatus.OK)
            self.send_header('Content-Type', f'multipart/x-mixed-replace; boundary={boundary}')
            self.end_headers()
            counted = False
            try:
                try:
                    self.server._mjpeg_clients = int(getattr(self.server, '_mjpeg_clients', 0)) + 1
                    counted = True
                except Exception:
                    pass
                with mss.mss() as sct:
                    # monitor index via query param ?monitor=<int>, default 0 (All)
                    qs = urllib.parse.parse_qs(parsed.query)
                    mon_idx = 0
                    try:
                        mon_idx = int((qs.get('monitor') or ['0'])[0])
                    except Exception:
                        mon_idx = 0
                    if mon_idx < 0 or mon_idx >= len(sct.monitors):
                        mon_idx = 0
                    monitor = sct.monitors[mon_idx]
                    # Quality and scale options
                    try:
                        quality = int((qs.get('quality') or ['70'])[0])
                    except Exception:
                        quality = 70
                    # Allow lower qualities for speed; clamp to [10, 95]
                    quality = max(10, min(quality, 95))
                    try:
                        scale = float((qs.get('scale') or ['1.0'])[0])
                    except Exception:
                        scale = 1.0
                    scale = max(0.25, min(scale, 1.0))
                    while True:
                        frame_start = time.time()
                        img = sct.grab(monitor)
                        # Select output format via query (?format=jpeg|png); default PNG for compatibility
                        fmt = (qs.get('format') or ['png'])[0].lower()
                        frame_bytes = None
                        ctype = 'image/png'
                        if fmt == 'jpeg' and Image is not None:
                            try:
                                pil = Image.frombytes('RGB', img.size, img.rgb)
                                if scale != 1.0:
                                    new_w = max(1, int(pil.width * scale))
                                    new_h = max(1, int(pil.height * scale))
                                    pil = pil.resize((new_w, new_h), Image.BILINEAR)
                                buf = BytesIO()
                                # Use faster save settings for higher throughput
                                pil.save(buf, format='JPEG', quality=quality, optimize=False, subsampling='2')
                                frame_bytes = buf.getvalue()
                                ctype = 'image/jpeg'
                            except Exception:
                                frame_bytes = None
                        if frame_bytes is None:
                            # PNG fallback (optionally scaled via Pillow)
                            if scale != 1.0 and Image is not None:
                                try:
                                    pil = Image.frombytes('RGB', img.size, img.rgb)
                                    new_w = max(1, int(pil.width * scale))
                                    new_h = max(1, int(pil.height * scale))
                                    pil = pil.resize((new_w, new_h), Image.BILINEAR)
                                    buf = BytesIO()
                                    # Avoid optimize for speed; default compression is sufficient
                                    pil.save(buf, format='PNG', optimize=False)
                                    frame_bytes = buf.getvalue()
                                except Exception:
                                    frame_bytes = None
                            if frame_bytes is None:
                                frame_bytes = mss.tools.to_png(img.rgb, img.size)
                        self.wfile.write(bytes(f'--{boundary}\r\n', 'utf-8'))
                        self.wfile.write(bytes(f'Content-Type: {ctype}\r\n', 'utf-8'))
                        self.wfile.write(bytes(f'Content-Length: {len(frame_bytes)}\r\n\r\n', 'utf-8'))
                        self.wfile.write(frame_bytes)
                        self.wfile.write(b'\r\n')
                        try:
                            self.server._mjpeg_frames_total = int(getattr(self.server, '_mjpeg_frames_total', 0)) + 1
                        except Exception:
                            pass
                        # Sleep only the remaining time to hit target FPS; don't slow below encode time
                        elapsed = time.time() - frame_start
                        remaining = max(0.0, delay - elapsed)
                        time.sleep(remaining)
            except Exception:
                # client closed or error
                return
            finally:
                if counted:
                    try:
                        self.server._mjpeg_clients = int(getattr(self.server, '_mjpeg_clients', 1)) - 1
                    except Exception:
                        pass

        # Single-frame snapshot fallback
        if path == '/api/snapshot.png':
            if not self._check_auth():
                self.send_error(HTTPStatus.UNAUTHORIZED)
                return
            if mss is None:
                self.send_error(HTTPStatus.SERVICE_UNAVAILABLE, "mss not available")
                return
            try:
                qs = urllib.parse.parse_qs(parsed.query)
                mon_idx = 0
                try:
                    mon_idx = int((qs.get('monitor') or ['0'])[0])
                except Exception:
                    mon_idx = 0
                with mss.mss() as sct:
                    if mon_idx < 0 or mon_idx >= len(sct.monitors):
                        mon_idx = 0
                    monitor = sct.monitors[mon_idx]
                    img = sct.grab(monitor)
                    png = mss.tools.to_png(img.rgb, img.size)
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'image/png')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Content-Length', str(len(png)))
                self.end_headers()
                self.wfile.write(png)
                return
            except Exception:
                self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR)
                return

        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get('Content-Length', '0') or '0')
        body = b''
        if length:
            body = self.rfile.read(length)
        if not self._check_auth():
            return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)

        if path == '/api/config':
            # Replace config.json with provided JSON, then notify GUI to reload
            try:
                if not body:
                    return self._write_json({"error": "empty body"}, HTTPStatus.BAD_REQUEST)
                new_cfg = json.loads(body.decode('utf-8'))
                if not isinstance(new_cfg, dict):
                    return self._write_json({"error": "config must be an object"}, HTTPStatus.BAD_REQUEST)
                write_config_json(new_cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Create a new sequence
        if path == '/api/sequences':
            try:
                payload = {}
                if body:
                    payload = json.loads(body.decode('utf-8'))
                name = payload.get('name')
                if not name:
                    return self._write_json({"error": "name required"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                if any(s.get('name') == name for s in cfg.get('sequences', [])):
                    return self._write_json({"error": "sequence exists"}, HTTPStatus.CONFLICT)
                seq = {
                    'name': name,
                    'steps': payload.get('steps', []),
                    'loop': payload.get('loop', False),
                    'loop_count': payload.get('loop_count', 1),
                }
                cfg.setdefault('sequences', []).append(seq)
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Append a step to failsafe sequence
        if path == '/api/failsafe/sequence':
            try:
                payload = {}
                if body:
                    payload = json.loads(body.decode('utf-8'))
                new_step = payload.get('step')
                if not isinstance(new_step, dict):
                    return self._write_json({"error": "step must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                fs.setdefault('sequence', [])
                fs['sequence'].append(new_step)
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True, "index": len(fs['sequence']) - 1})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Create a new group
        if path == '/api/groups':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                name = (payload.get('name') or '').strip()
                if not name:
                    return self._write_json({"error": "name required"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name in groups:
                    return self._write_json({"error": "exists"}, HTTPStatus.BAD_REQUEST)
                groups[name] = {"name": name, "steps": [], "loop": False, "loop_count": 1}
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Append a step to a group
        if path.startswith('/api/groups/') and path.endswith('/steps'):
            try:
                name = urllib.parse.unquote(path[len('/api/groups/'):-len('/steps')])
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_step = payload.get('step')
                if not isinstance(new_step, dict):
                    return self._write_json({"error": "step must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                grp = groups[name]
                if isinstance(grp, list):
                    grp.append(new_step)
                    groups[name] = grp
                else:
                    steps = grp.setdefault('steps', [])
                    steps.append(new_step)
                    groups[name] = grp
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                # compute index
                idx = len(groups[name]) - 1 if isinstance(groups[name], list) else (len(groups[name].get('steps', [])) - 1)
                return self._write_json({"ok": True, "index": idx})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Append an action to a group step
        if '/steps/' in path and path.endswith('/actions') and path.startswith('/api/groups/'):
            try:
                # path: /api/groups/<name>/steps/<idx>/actions
                base, _, _ = path.partition('/actions')
                head, _, tail = base.partition('/steps/')
                name = urllib.parse.unquote(head[len('/api/groups/'):])
                idx = int(tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_action = payload.get('action')
                if not isinstance(new_action, dict):
                    return self._write_json({"error": "action must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                grp = groups[name]
                steps = grp if isinstance(grp, list) else grp.setdefault('steps', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                actions = steps[idx].setdefault('actions', [])
                actions.append(new_action)
                groups[name] = steps if isinstance(grp, list) else grp
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True, "index": len(actions) - 1})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Reorder an action within a group step
        if '/steps/' in path and path.endswith('/actions/reorder') and path.startswith('/api/groups/'):
            try:
                base, _, _ = path.partition('/actions/reorder')
                head, _, tail = base.partition('/steps/')
                name = urllib.parse.unquote(head[len('/api/groups/'):])
                idx = int(tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                from_idx = int(payload.get('from_index'))
                to_idx = int(payload.get('to_index'))
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                grp = groups[name]
                steps = grp if isinstance(grp, list) else grp.setdefault('steps', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                actions = steps[idx].setdefault('actions', [])
                if (from_idx < 0 or from_idx >= len(actions) or to_idx < 0 or to_idx >= len(actions)):
                    return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                act = actions.pop(from_idx)
                actions.insert(to_idx, act)
                groups[name] = steps if isinstance(grp, list) else grp
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Reorder a step within a group
        if path.startswith('/api/groups/') and path.endswith('/steps/reorder'):
            try:
                base = path[:-len('/steps/reorder')]
                name = urllib.parse.unquote(base[len('/api/groups/'):])
                payload = json.loads(body.decode('utf-8')) if body else {}
                from_idx = int(payload.get('from_index'))
                to_idx = int(payload.get('to_index'))
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                if isinstance(groups[name], list):
                    steps = groups[name]
                else:
                    steps = groups[name].setdefault('steps', [])
                if (from_idx < 0 or from_idx >= len(steps) or to_idx < 0 or to_idx >= len(steps)):
                    return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                st = steps.pop(from_idx)
                steps.insert(to_idx, st)
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Reorder a step within a sequence
        if path.endswith('/steps/reorder') and path.startswith('/api/sequences/'):
            try:
                base = path[:-len('/steps/reorder')]
                name = urllib.parse.unquote(base[len('/api/sequences/'):])
                payload = json.loads(body.decode('utf-8')) if body else {}
                from_idx = int(payload.get('from_index'))
                to_idx = int(payload.get('to_index'))
                cfg = read_config_json()
                for i, s in enumerate(cfg.get('sequences', [])):
                    if s.get('name') == name:
                        steps = s.setdefault('steps', [])
                        if (from_idx < 0 or from_idx >= len(steps) or to_idx < 0 or to_idx >= len(steps)):
                            return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                        step = steps.pop(from_idx)
                        steps.insert(to_idx, step)
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Reorder a step within failsafe sequence
        if path == '/api/failsafe/sequence/reorder':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                from_idx = int(payload.get('from_index'))
                to_idx = int(payload.get('to_index'))
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                steps = fs.setdefault('sequence', [])
                if (from_idx < 0 or from_idx >= len(steps) or to_idx < 0 or to_idx >= len(steps)):
                    return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                step = steps.pop(from_idx)
                steps.insert(to_idx, step)
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Append an action to a sequence step
        if '/steps/' in path and path.endswith('/actions') and path.startswith('/api/sequences/'):
            try:
                # path: /api/sequences/<name>/steps/<idx>/actions
                base, _, _ = path.partition('/actions')
                head, _, tail = base.partition('/steps/')
                name = urllib.parse.unquote(head[len('/api/sequences/'):])
                idx = int(tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_action = payload.get('action')
                if not isinstance(new_action, dict):
                    return self._write_json({"error": "action must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                for s in cfg.get('sequences', []):
                    if s.get('name') == name:
                        steps = s.setdefault('steps', [])
                        if idx < 0 or idx >= len(steps):
                            return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                        st = steps[idx]
                        st.setdefault('actions', []).append(new_action)
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True, "index": len(st['actions']) - 1})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Reorder an action within a sequence step
        if '/steps/' in path and path.endswith('/actions/reorder') and path.startswith('/api/sequences/'):
            try:
                base = path[:-len('/actions/reorder')]
                head, _, tail = base.partition('/steps/')
                name = urllib.parse.unquote(head[len('/api/sequences/'):])
                idx = int(tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                from_idx = int(payload.get('from_index'))
                to_idx = int(payload.get('to_index'))
                cfg = read_config_json()
                for s in cfg.get('sequences', []):
                    if s.get('name') == name:
                        steps = s.setdefault('steps', [])
                        if idx < 0 or idx >= len(steps):
                            return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                        actions = steps[idx].setdefault('actions', [])
                        if (from_idx < 0 or from_idx >= len(actions) or to_idx < 0 or to_idx >= len(actions)):
                            return self._write_json({"error": "action index out of range"}, HTTPStatus.BAD_REQUEST)
                        act = actions.pop(from_idx)
                        actions.insert(to_idx, act)
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Append an action to failsafe step
        if path.endswith('/actions') and path.startswith('/api/failsafe/sequence/'):
            try:
                base = path[:-len('/actions')]
                idx = int(base[len('/api/failsafe/sequence/'):])
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_action = payload.get('action')
                if not isinstance(new_action, dict):
                    return self._write_json({"error": "action must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                steps = fs.setdefault('sequence', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                st = steps[idx]
                st.setdefault('actions', []).append(new_action)
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True, "index": len(st['actions']) - 1})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Capture a screen region and save as a template image
        if path == '/api/templates/capture':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                name = (payload.get('name') or '').strip()
                # Region params relative to the selected monitor's coordinate space
                mon_idx = int(payload.get('monitor', 0))
                x = int(payload.get('x', -1))
                y = int(payload.get('y', -1))
                w = int(payload.get('width', 0))
                h = int(payload.get('height', 0))
                if not name:
                    return self._write_json({"error": "name required"}, HTTPStatus.BAD_REQUEST)
                if x < 0 or y < 0 or w <= 0 or h <= 0:
                    return self._write_json({"error": "invalid region"}, HTTPStatus.BAD_REQUEST)
                if mss is None:
                    return self._write_json({"error": "mss not available"}, HTTPStatus.SERVICE_UNAVAILABLE)
                # Grab the region
                with mss.mss() as sct:
                    if mon_idx < 0 or mon_idx >= len(sct.monitors):
                        mon_idx = 0
                    monitor = sct.monitors[mon_idx]
                    region = {
                        'left': int(monitor.get('left', 0)) + x,
                        'top': int(monitor.get('top', 0)) + y,
                        'width': max(1, int(w)),
                        'height': max(1, int(h))
                    }
                    img = sct.grab(region)
                    png = mss.tools.to_png(img.rgb, img.size)
                # Determine output path (default to images/<name>.png)
                out_rel = (payload.get('path') or '').strip()
                if not out_rel:
                    safe = ''.join([c for c in name if c.isalnum() or c in ('-', '_')])
                    if not safe:
                        safe = f"tpl_{int(time.time())}"
                    out_rel = f"images/{safe}.png"
                out_full = os.path.join(PROJECT_ROOT, out_rel) if not os.path.isabs(out_rel) else out_rel
                os.makedirs(os.path.dirname(out_full), exist_ok=True)
                with open(out_full, 'wb') as f:
                    f.write(png)
                # Update config template mapping
                cfg = read_config_json()
                tpl_map = cfg.get('templates', {})
                tpl_map[name] = out_rel
                cfg['templates'] = tpl_map
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True, "path": out_rel})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Create or update a template mapping (add new)
        if path == '/api/templates':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                name = payload.get('name')
                tpath = payload.get('path')
                if not name or not tpath:
                    return self._write_json({"error": "name and path required"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                tpl_map = cfg.get('templates', {})
                tpl_map[name] = tpath
                cfg['templates'] = tpl_map
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Upload a template image via base64 and map it in config
        if path == '/api/templates/upload':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                name = (payload.get('name') or '').strip()
                data_b64 = (payload.get('image_base64') or '').strip()
                ext = (payload.get('ext') or 'png').lower().strip()
                if not name:
                    return self._write_json({"error": "name required"}, HTTPStatus.BAD_REQUEST)
                if not data_b64:
                    return self._write_json({"error": "image_base64 required"}, HTTPStatus.BAD_REQUEST)
                if ext in ('jpeg',):
                    ext = 'jpg'
                if ext not in ('png', 'jpg'):
                    return self._write_json({"error": "ext must be png or jpg"}, HTTPStatus.BAD_REQUEST)
                # Allow data URL prefix
                if data_b64.startswith('data:'):
                    try:
                        data_b64 = data_b64.split(',', 1)[1]
                    except Exception:
                        pass
                try:
                    raw = base64.b64decode(data_b64, validate=True)
                except Exception:
                    return self._write_json({"error": "invalid base64"}, HTTPStatus.BAD_REQUEST)
                # Determine output path: images/<safe>.ext
                safe = ''.join([c if (c.isalnum() or c in ('-', '_')) else '_' for c in name])
                if not safe:
                    safe = f"tpl_{int(time.time())}"
                out_rel = f"images/{safe}.{ext}"
                out_full = os.path.join(PROJECT_ROOT, out_rel) if not os.path.isabs(out_rel) else out_rel
                os.makedirs(os.path.dirname(out_full), exist_ok=True)
                with open(out_full, 'wb') as f:
                    f.write(raw)
                # Update config mapping
                cfg = read_config_json()
                tpl_map = cfg.get('templates', {}) or {}
                tpl_map[name] = out_rel.replace('\\', '/')
                cfg['templates'] = tpl_map
                write_config_json(cfg)
                try:
                    trigger_reload_config_ipc()
                except Exception:
                    pass
                return self._write_json({"ok": True, "path": tpl_map[name]})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Append a step to a sequence
        if path.endswith('/steps') and path.startswith('/api/sequences/'):
            try:
                name = urllib.parse.unquote(path[len('/api/sequences/'):-len('/steps')])
                payload = {}
                if body:
                    payload = json.loads(body.decode('utf-8'))
                new_step = payload.get('step')
                if not isinstance(new_step, dict):
                    return self._write_json({"error": "step must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                for i, s in enumerate(cfg.get('sequences', [])):
                    if s.get('name') == name:
                        s.setdefault('steps', []).append(new_step)
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True, "index": len(s['steps']) - 1})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Run options (e.g., non_required_wait)
        if path == '/api/run-options':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                nrw = bool(payload.get('non_required_wait', False))
                # Notify GUI to set checkbox
                cmd_path = os.path.join(PROJECT_ROOT, 'ipc_command.json')
                with open(cmd_path, 'w', encoding='utf-8') as f:
                    json.dump({"command": "set_non_required_wait", "value": nrw}, f)
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        if path == '/api/run':
            # Prefer IPC to trigger run reliably.
            try:
                payload = {}
                if body:
                    try:
                        payload = json.loads(body.decode('utf-8'))
                    except Exception:
                        payload = {}
                seq_name = payload.get('sequence')
                cmd_path = os.path.join(PROJECT_ROOT, 'ipc_command.json')
                with open(cmd_path, 'w', encoding='utf-8') as f:
                    json.dump({"command": "run", "sequence_name": seq_name}, f)
                # Update last-run info for SSE consumers
                try:
                    self.server._last_run = {
                        "status": "run_requested",
                        "sequence": seq_name,
                        "group": None,
                        "last_event_ts": int(time.time())
                    }
                except Exception:
                    pass
                return self._write_json({"ok": True, "mode": "ipc", "sequence": seq_name})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Run a group by creating a temporary sequence from its steps and triggering IPC run
        if path == '/api/run-group':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                group_name = (payload.get('group') or '').strip()
                if not group_name:
                    return self._write_json({"error": "group required"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if group_name not in groups:
                    return self._write_json({"error": "group not found"}, HTTPStatus.NOT_FOUND)
                g = groups[group_name]
                steps = g if isinstance(g, list) else (g.get('steps') or [])
                # Create/update a special sequence to run this group's steps
                run_seq_name = "__RunGroup__"
                seqs = cfg.setdefault('sequences', [])
                idx = next((i for i, s in enumerate(seqs) if s.get('name') == run_seq_name), -1)
                seq_obj = { 'name': run_seq_name, 'steps': steps, 'loop': False, 'loop_count': 1 }
                if idx >= 0:
                    seqs[idx] = seq_obj
                else:
                    seqs.append(seq_obj)
                cfg['sequences'] = seqs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                # Trigger IPC run of the temporary sequence
                cmd_path = os.path.join(PROJECT_ROOT, 'ipc_command.json')
                with open(cmd_path, 'w', encoding='utf-8') as f:
                    json.dump({"command": "run", "sequence_name": run_seq_name}, f)
                # Update last-run info for SSE consumers
                try:
                    self.server._last_run = {
                        "status": "run_group_requested",
                        "sequence": run_seq_name,
                        "group": group_name,
                        "last_event_ts": int(time.time())
                    }
                except Exception:
                    pass
                return self._write_json({"ok": True, "mode": "ipc", "sequence": run_seq_name, "group": group_name})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        if path == '/api/stop':
            try:
                cmd_path = os.path.join(PROJECT_ROOT, 'ipc_command.json')
                with open(cmd_path, 'w', encoding='utf-8') as f:
                    json.dump({"command": "stop"}, f)
                # Update last-run info for SSE consumers
                try:
                    self.server._last_run = {
                        "status": "stop_requested",
                        "sequence": (getattr(self.server, '_last_run', {}) or {}).get('sequence'),
                        "group": (getattr(self.server, '_last_run', {}) or {}).get('group'),
                        "last_event_ts": int(time.time())
                    }
                except Exception:
                    pass
                return self._write_json({"ok": True, "mode": "ipc"})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_PUT(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        length = int(self.headers.get('Content-Length', '0') or '0')
        body = b''
        if length:
            body = self.rfile.read(length)
        if not self._check_auth():
            return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)

        # Rename a template and update all references across config
        if path == '/api/templates/rename':
            try:
                payload = {}
                if body:
                    payload = json.loads(body.decode('utf-8'))
                old_name = payload.get('old_name')
                new_name = payload.get('new_name')
                if not old_name or not new_name:
                    return self._write_json({"error": "old_name and new_name required"}, HTTPStatus.BAD_REQUEST)
                if old_name == new_name:
                    return self._write_json({"error": "names must differ"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                tpl_map = cfg.get('templates', {}) or {}
                if old_name not in tpl_map:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                if new_name in tpl_map:
                    return self._write_json({"error": "target exists"}, HTTPStatus.CONFLICT)

                # Move template mapping
                path_value = tpl_map.get(old_name)
                del tpl_map[old_name]
                tpl_map[new_name] = path_value
                cfg['templates'] = tpl_map

                def _rename_in_step(step_obj: dict):
                    try:
                        if step_obj.get('find') == old_name:
                            step_obj['find'] = new_name
                        actions = step_obj.get('actions') or []
                        for a in actions:
                            if a.get('template') == old_name:
                                a['template'] = new_name
                            if a.get('if_template') == old_name:
                                a['if_template'] = new_name
                    except Exception:
                        # Be permissive; continue renaming other references
                        pass

                # Sequences
                for seq in cfg.get('sequences', []) or []:
                    for st in seq.get('steps', []) or []:
                        _rename_in_step(st)

                # Failsafe settings and sequence
                fs = cfg.get('failsafe', {}) or {}
                if fs.get('template') == old_name:
                    fs['template'] = new_name
                for st in fs.get('steps', []) or []:
                    _rename_in_step(st)
                cfg['failsafe'] = fs

                # Groups
                grps = cfg.get('groups', {}) or {}
                for _, g in grps.items():
                    for st in g.get('steps', []) or []:
                        _rename_in_step(st)
                cfg['groups'] = grps

                # Persist and notify GUI to reload
                write_config_json(cfg)
                try:
                    trigger_reload_config_ipc()
                except Exception:
                    # IPC optional; GUI may not be running
                    pass
                return self._write_json({"ok": True, "renamed": {"from": old_name, "to": new_name}})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)
        try:
            payload = json.loads(body.decode('utf-8')) if body else {}
        except Exception:
            payload = {}

        # Update break settings
        if path == '/api/break-settings':
            try:
                cfg = read_config_json()
                br = cfg.get('break_settings', {})
                # Accept either max_runtime_seconds or hours/minutes/seconds triple
                if 'max_runtime_seconds' in payload:
                    br['max_runtime_seconds'] = int(payload.get('max_runtime_seconds', 0))
                else:
                    hrs = int(payload.get('hours', 0))
                    mins = int(payload.get('minutes', 0))
                    secs = int(payload.get('seconds', 0))
                    br['max_runtime_seconds'] = hrs * 3600 + mins * 60 + secs
                if 'enabled' in payload:
                    br['enabled'] = bool(payload.get('enabled'))
                if 'run_final_after_break' in payload:
                    br['run_final_after_break'] = bool(payload.get('run_final_after_break'))
                if 'final_sequence_name' in payload:
                    br['final_sequence_name'] = payload.get('final_sequence_name') or '(none)'
                cfg['break_settings'] = br
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update debug options (e.g., branch traces)
        if path == '/api/debug':
            try:
                cfg = read_config_json()
                dbg = cfg.get('debug', {}) or {}
                if 'branch_traces' in payload:
                    val = bool(payload.get('branch_traces'))
                    dbg['branch_traces'] = val
                    cfg['debug'] = dbg
                    # Mirror legacy top-level flag for compatibility
                    cfg['debug_branch_traces'] = val
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True, "debug": cfg.get('debug', {}), "debug_branch_traces": cfg.get('debug_branch_traces')})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update failsafe settings
        if path == '/api/failsafe':
            try:
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                if 'enabled' in payload:
                    fs['enabled'] = bool(payload.get('enabled'))
                if 'template_name' in payload:
                    tn = payload.get('template_name')
                    # Write both keys so GUI and web stay in sync
                    fs['template_name'] = tn if tn else None
                    fs['template'] = tn if tn else None
                if 'confidence' in payload:
                    fs['confidence'] = float(payload.get('confidence', 0.8))
                if 'region' in payload:
                    fs['region'] = payload.get('region')
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update entire sequence (rename allowed)
        if path.startswith('/api/sequences/') and '/steps/' not in path:
            try:
                name = urllib.parse.unquote(path[len('/api/sequences/'):])
                cfg = read_config_json()
                seqs = cfg.get('sequences', [])
                idx = next((i for i, s in enumerate(seqs) if s.get('name') == name), -1)
                if idx < 0:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                s = seqs[idx]
                if 'name' in payload:
                    s['name'] = payload['name']
                if 'steps' in payload and isinstance(payload['steps'], list):
                    s['steps'] = payload['steps']
                if 'loop' in payload:
                    s['loop'] = bool(payload['loop'])
                if 'loop_count' in payload:
                    s['loop_count'] = int(payload['loop_count'])
                seqs[idx] = s
                cfg['sequences'] = seqs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update single step by index (exclude action paths)
        if '/steps/' in path and path.startswith('/api/sequences/') and ('/actions/' not in path):
            try:
                # path: /api/sequences/<name>/steps/<index>
                # Extract name and index
                base, _, tail = path.partition('/steps/')
                name = urllib.parse.unquote(base[len('/api/sequences/'):])
                try:
                    idx = int(tail)
                except Exception:
                    return self._write_json({"error": "invalid index"}, HTTPStatus.BAD_REQUEST)
                new_step = payload.get('step')
                if not isinstance(new_step, dict):
                    return self._write_json({"error": "step must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                for i, s in enumerate(cfg.get('sequences', [])):
                    if s.get('name') == name:
                        steps = s.setdefault('steps', [])
                        if idx < 0 or idx >= len(steps):
                            return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                        steps[idx] = new_step
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update single action by index within a sequence step
        if '/steps/' in path and '/actions/' in path and path.startswith('/api/sequences/'):
            try:
                # path: /api/sequences/<name>/steps/<idx>/actions/<aidx>
                pre, _, tail = path.partition('/steps/')
                name = urllib.parse.unquote(pre[len('/api/sequences/'):])
                mid, _, a_tail = tail.partition('/actions/')
                idx = int(mid)
                aidx = int(a_tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_action = payload.get('action')
                if not isinstance(new_action, dict):
                    return self._write_json({"error": "action must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                for s in cfg.get('sequences', []):
                    if s.get('name') == name:
                        steps = s.setdefault('steps', [])
                        if idx < 0 or idx >= len(steps):
                            return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                        actions = steps[idx].setdefault('actions', [])
                        if aidx < 0 or aidx >= len(actions):
                            return self._write_json({"error": "action index out of range"}, HTTPStatus.BAD_REQUEST)
                        actions[aidx] = new_action
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Replace entire failsafe sequence
        if path == '/api/failsafe/sequence':
            try:
                payload = json.loads(body.decode('utf-8')) if body else {}
                steps = payload.get('steps')
                if not isinstance(steps, list):
                    return self._write_json({"error": "steps must be array"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                fs['sequence'] = steps
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True, "count": len(steps)})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update failsafe step by index (exclude action paths)
        if path.startswith('/api/failsafe/sequence/') and ('/actions/' not in path):
            try:
                idx_str = path[len('/api/failsafe/sequence/'):]
                try:
                    idx = int(idx_str)
                except Exception:
                    return self._write_json({"error": "invalid index"}, HTTPStatus.BAD_REQUEST)
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_step = payload.get('step')
                if not isinstance(new_step, dict):
                    return self._write_json({"error": "step must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                steps = fs.setdefault('sequence', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                steps[idx] = new_step
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update single action by index within a failsafe step
        if '/api/failsafe/sequence/' in path and '/actions/' in path:
            try:
                # path: /api/failsafe/sequence/<idx>/actions/<aidx>
                pre, _, a_tail = path.partition('/actions/')
                idx = int(pre[len('/api/failsafe/sequence/'):])
                aidx = int(a_tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_action = payload.get('action')
                if not isinstance(new_action, dict):
                    return self._write_json({"error": "action must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                steps = fs.setdefault('sequence', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                actions = steps[idx].setdefault('actions', [])
                if aidx < 0 or aidx >= len(actions):
                    return self._write_json({"error": "action index out of range"}, HTTPStatus.BAD_REQUEST)
                actions[aidx] = new_action
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update template path
        if path.startswith('/api/templates/'):
            try:
                name = urllib.parse.unquote(path[len('/api/templates/'):])
                payload = json.loads(body.decode('utf-8')) if body else {}
                tpath = payload.get('path')
                if tpath is None:
                    return self._write_json({"error": "path required"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                tpl_map = cfg.get('templates', {})
                if name not in tpl_map:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                tpl_map[name] = tpath
                cfg['templates'] = tpl_map
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update entire group (rename allowed) or metadata
        if path.startswith('/api/groups/') and '/steps/' not in path:
            try:
                name = urllib.parse.unquote(path[len('/api/groups/'):])
                payload = json.loads(body.decode('utf-8')) if body else {}
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                g = groups[name]
                # Support both dict and list
                if 'name' in payload:
                    new_name = (payload['name'] or '').strip()
                    if new_name and new_name != name:
                        groups[new_name] = g
                        groups.pop(name)
                        name = new_name
                if isinstance(g, list):
                    if 'steps' in payload and isinstance(payload['steps'], list):
                        groups[name] = payload['steps']
                else:
                    if 'steps' in payload and isinstance(payload['steps'], list):
                        g['steps'] = payload['steps']
                    if 'loop' in payload:
                        g['loop'] = bool(payload['loop'])
                    if 'loop_count' in payload:
                        g['loop_count'] = int(payload['loop_count'])
                    groups[name] = g
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update single step within a group by index
        if '/steps/' in path and path.startswith('/api/groups/'):
            try:
                base, _, tail = path.partition('/steps/')
                name = urllib.parse.unquote(base[len('/api/groups/'):])
                idx = int(tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_step = payload.get('step')
                if not isinstance(new_step, dict):
                    return self._write_json({"error": "step must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                if isinstance(groups[name], list):
                    steps = groups[name]
                else:
                    steps = groups[name].setdefault('steps', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                steps[idx] = new_step
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Update single action by index within a group step
        if '/steps/' in path and '/actions/' in path and path.startswith('/api/groups/'):
            try:
                # path: /api/groups/<name>/steps/<idx>/actions/<aidx>
                pre, _, tail_actions = path.partition('/steps/')
                name = urllib.parse.unquote(pre[len('/api/groups/'):])
                mid, _, a_tail = tail_actions.partition('/actions/')
                idx = int(mid)
                aidx = int(a_tail)
                payload = json.loads(body.decode('utf-8')) if body else {}
                new_action = payload.get('action')
                if not isinstance(new_action, dict):
                    return self._write_json({"error": "action must be object"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                grp = groups[name]
                steps = grp if isinstance(grp, list) else grp.setdefault('steps', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                actions = steps[idx].setdefault('actions', [])
                if aidx < 0 or aidx >= len(actions):
                    return self._write_json({"error": "action index out of range"}, HTTPStatus.BAD_REQUEST)
                actions[aidx] = new_action
                groups[name] = steps if isinstance(grp, list) else grp
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Replace all scheduled sequences
        if path == '/api/schedules':
            try:
                cfg = read_config_json()
                schedules = payload.get('schedules')
                if not isinstance(schedules, list):
                    return self._write_json({"error": "schedules must be an array"}, HTTPStatus.BAD_REQUEST)
                # Basic normalization: ensure keys exist
                norm = []
                for sch in schedules:
                    if not isinstance(sch, dict):
                        continue
                    norm.append({
                        'enabled': bool(sch.get('enabled', False)),
                        'sequence_name': sch.get('sequence_name') or '(none)',
                        'time': str(sch.get('time', '12:00 PM')).strip(),
                        'queue_if_busy': bool(sch.get('queue_if_busy', False)),
                        'preempt_if_busy': bool(sch.get('preempt_if_busy', False)),
                        'resume_previous': bool(sch.get('resume_previous', False)),
                        'last_run_date': sch.get('last_run_date')
                    })
                cfg['scheduled_sequences'] = norm
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True, "count": len(norm)})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)

    def do_DELETE(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path
        if not self._check_auth():
            return self._write_json({"error": "unauthorized"}, HTTPStatus.UNAUTHORIZED)

        # Delete entire sequence
        if path.startswith('/api/sequences/') and '/steps/' not in path:
            try:
                name = urllib.parse.unquote(path[len('/api/sequences/'):])
                cfg = read_config_json()
                before = len(cfg.get('sequences', []))
                cfg['sequences'] = [s for s in cfg.get('sequences', []) if s.get('name') != name]
                if len(cfg['sequences']) == before:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete single step by index (exclude action paths)
        if '/steps/' in path and path.startswith('/api/sequences/') and ('/actions/' not in path):
            try:
                base, _, tail = path.partition('/steps/')
                name = urllib.parse.unquote(base[len('/api/sequences/'):])
                try:
                    idx = int(tail)
                except Exception:
                    return self._write_json({"error": "invalid index"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                for i, s in enumerate(cfg.get('sequences', [])):
                    if s.get('name') == name:
                        steps = s.setdefault('steps', [])
                        if idx < 0 or idx >= len(steps):
                            return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                        steps.pop(idx)
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete single action by index within a sequence step
        if '/steps/' in path and '/actions/' in path and path.startswith('/api/sequences/'):
            try:
                pre, _, tail = path.partition('/steps/')
                name = urllib.parse.unquote(pre[len('/api/sequences/'):])
                mid, _, a_tail = tail.partition('/actions/')
                idx = int(mid)
                aidx = int(a_tail)
                cfg = read_config_json()
                for s in cfg.get('sequences', []):
                    if s.get('name') == name:
                        steps = s.setdefault('steps', [])
                        if idx < 0 or idx >= len(steps):
                            return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                        actions = steps[idx].setdefault('actions', [])
                        if aidx < 0 or aidx >= len(actions):
                            return self._write_json({"error": "action index out of range"}, HTTPStatus.BAD_REQUEST)
                        actions.pop(aidx)
                        write_config_json(cfg)
                        trigger_reload_config_ipc()
                        return self._write_json({"ok": True})
                return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete single action by index within a failsafe step
        if '/api/failsafe/sequence/' in path and '/actions/' in path:
            try:
                pre, _, a_tail = path.partition('/actions/')
                idx = int(pre[len('/api/failsafe/sequence/'):])
                aidx = int(a_tail)
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                steps = fs.setdefault('sequence', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                actions = steps[idx].setdefault('actions', [])
                if aidx < 0 or aidx >= len(actions):
                    return self._write_json({"error": "action index out of range"}, HTTPStatus.BAD_REQUEST)
                actions.pop(aidx)
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete failsafe step by index
        if path.startswith('/api/failsafe/sequence/'):
            try:
                idx_str = path[len('/api/failsafe/sequence/'):]
                try:
                    idx = int(idx_str)
                except Exception:
                    return self._write_json({"error": "invalid index"}, HTTPStatus.BAD_REQUEST)
                cfg = read_config_json()
                fs = cfg.get('failsafe', {}) or {}
                steps = fs.setdefault('sequence', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                steps.pop(idx)
                cfg['failsafe'] = fs
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete template mapping
        if path.startswith('/api/templates/'):
            try:
                name = urllib.parse.unquote(path[len('/api/templates/'):])
                cfg = read_config_json()
                tpl_map = cfg.get('templates', {})
                if name not in tpl_map:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                del tpl_map[name]
                cfg['templates'] = tpl_map
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete group
        if path.startswith('/api/groups/') and '/steps/' not in path:
            try:
                name = urllib.parse.unquote(path[len('/api/groups/'):])
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                groups.pop(name)
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete single step by index from group (exclude action paths)
        if '/steps/' in path and path.startswith('/api/groups/') and ('/actions/' not in path):
            try:
                base, _, tail = path.partition('/steps/')
                name = urllib.parse.unquote(base[len('/api/groups/'):])
                idx = int(tail)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                if isinstance(groups[name], list):
                    steps = groups[name]
                else:
                    steps = groups[name].setdefault('steps', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "index out of range"}, HTTPStatus.BAD_REQUEST)
                steps.pop(idx)
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        # Delete single action by index within a group step
        if '/steps/' in path and '/actions/' in path and path.startswith('/api/groups/'):
            try:
                pre, _, tail = path.partition('/steps/')
                name = urllib.parse.unquote(pre[len('/api/groups/'):])
                mid, _, a_tail = tail.partition('/actions/')
                idx = int(mid)
                aidx = int(a_tail)
                cfg = read_config_json()
                groups = cfg.get('groups', {}) or {}
                if name not in groups:
                    return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
                grp = groups[name]
                steps = grp if isinstance(grp, list) else grp.setdefault('steps', [])
                if idx < 0 or idx >= len(steps):
                    return self._write_json({"error": "step index out of range"}, HTTPStatus.BAD_REQUEST)
                actions = steps[idx].setdefault('actions', [])
                if aidx < 0 or aidx >= len(actions):
                    return self._write_json({"error": "action index out of range"}, HTTPStatus.BAD_REQUEST)
                actions.pop(aidx)
                groups[name] = steps if isinstance(grp, list) else grp
                cfg['groups'] = groups
                write_config_json(cfg)
                trigger_reload_config_ipc()
                return self._write_json({"ok": True})
            except (BrokenPipeError, ConnectionAbortedError, ConnectionResetError, OSError):
                return
            except Exception as e:
                return self._write_json({"error": str(e)}, HTTPStatus.INTERNAL_SERVER_ERROR)

        return self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)


def run_server():
    cfg = load_server_config()
    server = ThreadingHTTPServer((cfg['bind'], cfg['port']), WebHandler)
    server._cfg = cfg
    # Server start timestamp for uptime calculations
    try:
        server._start_ts = time.time()
    except Exception:
        pass
    # Connection counters for monitoring
    try:
        # Attach psutil Process reference when available
        if psutil is not None:
            server._proc = psutil.Process(os.getpid())
        server._sse_clients = 0
        server._mjpeg_clients = 0
        server._log_clients = 0
        server._sse_messages_total = 0
        server._mjpeg_frames_total = 0
        server._log_messages_total = 0
    except Exception:
        pass
    # Initialize last-run info for SSE heartbeat consumers
    try:
        server._last_run = {
            "status": "idle",
            "sequence": None,
            "group": None,
            "last_event_ts": None
        }
    except Exception:
        pass
    print(f"[web] Serving on http://{cfg['bind']}:{cfg['port']} (token required)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

if __name__ == '__main__':
    run_server()