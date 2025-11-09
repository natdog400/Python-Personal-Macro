import os
import sys
import json
import subprocess
import threading
import platform
import webbrowser
import urllib.parse
import shutil
import tkinter as tk
from tkinter import ttk, messagebox


def _base_dir():
    if getattr(sys, 'frozen', False):
        # When frozen (PyInstaller), executables live in this directory
        return os.path.dirname(sys.executable)
    # Running from source
    return os.path.dirname(os.path.abspath(__file__))

BASE_DIR = _base_dir()
SERVER_CONFIG_PATH = os.path.join(BASE_DIR, 'server_config.json')


DEFAULT_SERVER_CFG = {
    'bind': '127.0.0.1',
    'port': 8765,
    'token': 'CHANGE_ME_TOKEN',
    'mjpeg_fps': 6,
    'backup_retention': 20,
}


def load_server_config():
    cfg = DEFAULT_SERVER_CFG.copy()
    try:
        if os.path.exists(SERVER_CONFIG_PATH):
            with open(SERVER_CONFIG_PATH, 'r', encoding='utf-8') as f:
                user_cfg = json.load(f)
            if isinstance(user_cfg, dict):
                cfg.update(user_cfg)
    except Exception:
        pass
    return cfg


def save_server_config(cfg):
    try:
        with open(SERVER_CONFIG_PATH, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=2)
        return True
    except Exception as e:
        messagebox.showerror('Error', f'Failed to save server_config.json: {e}')
        return False


def start_process(args):
    try:
        creationflags = 0
        if platform.system() == 'Windows':
            # Open new console window for each process (Windows only)
            creationflags = subprocess.CREATE_NEW_CONSOLE
        proc = subprocess.Popen(args, cwd=BASE_DIR, creationflags=creationflags)
        return proc
    except Exception as e:
        messagebox.showerror('Error', f'Failed to launch: {" ".join(args)}\n\n{e}')
        return None


class LauncherApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('Image Detection Bot Launcher')
        self.geometry('520x420')
        self.resizable(False, False)

        # In dev, use current Python. In frozen, avoid using sys.executable (points to this EXE).
        self.is_frozen = getattr(sys, 'frozen', False)
        if self.is_frozen:
            # Try to locate a system Python for non-bundled fallbacks (rare). Prefer py launcher.
            py_cmd = shutil.which('py')
            if py_cmd:
                self.python_exe = py_cmd
            else:
                self.python_exe = shutil.which('python') or 'python'
        else:
            self.python_exe = sys.executable
        self.server_cfg = load_server_config()

        self.var_launch_gui = tk.BooleanVar(value=True)
        self.var_launch_web = tk.BooleanVar(value=False)
        self.var_use_last = tk.BooleanVar(value=True)

        self.proc_web = None
        self.proc_gui = None

        self._build_ui()
        self._refresh_fields()

    def _build_ui(self):
        frm = ttk.Frame(self, padding=12)
        frm.pack(fill=tk.BOTH, expand=True)

        # Header
        ttk.Label(frm, text='Choose components to start:').grid(row=0, column=0, sticky='w')
        ttk.Checkbutton(frm, text='Launch GUI', variable=self.var_launch_gui).grid(row=1, column=0, sticky='w')
        ttk.Checkbutton(frm, text='Launch Web Server', variable=self.var_launch_web, command=self._refresh_fields).grid(row=2, column=0, sticky='w')

        ttk.Separator(frm).grid(row=3, column=0, sticky='ew', pady=(10, 10))

        # Server settings section
        self.grp = ttk.LabelFrame(frm, text='Web Server Settings')
        self.grp.grid(row=4, column=0, sticky='ew')

        # Use last vs edit
        ttk.Radiobutton(self.grp, text='Use last settings (server_config.json)', value=True, variable=self.var_use_last, command=self._refresh_fields).grid(row=0, column=0, columnspan=2, sticky='w', pady=(6, 2))
        ttk.Radiobutton(self.grp, text='Edit and save new settings', value=False, variable=self.var_use_last, command=self._refresh_fields).grid(row=1, column=0, columnspan=2, sticky='w')

        # Fields
        self.entry_bind = self._add_field(self.grp, 'Bind', self.server_cfg.get('bind'))
        self.entry_port = self._add_field(self.grp, 'Port', str(self.server_cfg.get('port')))
        self.entry_token = self._add_field(self.grp, 'Token', self.server_cfg.get('token'))
        self.entry_fps = self._add_field(self.grp, 'MJPEG FPS', str(self.server_cfg.get('mjpeg_fps')))
        self.entry_ret = self._add_field(self.grp, 'Backup Retention', str(self.server_cfg.get('backup_retention')))

        # Spacer
        ttk.Separator(frm).grid(row=5, column=0, sticky='ew', pady=(10, 10))

        # Buttons
        btns = ttk.Frame(frm)
        btns.grid(row=6, column=0, sticky='ew')
        ttk.Button(btns, text='Start', command=self.on_start).pack(side=tk.LEFT)
        ttk.Button(btns, text='Open Web Portal', command=self.on_open_portal).pack(side=tk.LEFT, padx=(8,0))
        ttk.Button(btns, text='Stop GUI', command=self.on_stop_gui).pack(side=tk.LEFT, padx=(8,0))
        ttk.Button(btns, text='Stop Web Server', command=self.on_stop_web).pack(side=tk.LEFT, padx=(8,0))
        ttk.Button(btns, text='Exit', command=self.destroy).pack(side=tk.RIGHT)

        # Status
        self.lbl_status = ttk.Label(frm, text='Ready.')
        self.lbl_status.grid(row=7, column=0, sticky='w', pady=(12, 0))

    def _add_field(self, parent, label, value):
        row = len(parent.grid_slaves()) + 1
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky='w', padx=(6, 6), pady=(4, 2))
        entry = ttk.Entry(parent)
        entry.insert(0, value if value is not None else '')
        entry.grid(row=row, column=1, sticky='ew', pady=(4, 2))
        parent.columnconfigure(1, weight=1)
        return entry

    def _refresh_fields(self):
        # Enable/disable server config fields based on choices
        editing = self.var_launch_web.get() and (not self.var_use_last.get())
        state = 'normal' if editing else 'disabled'
        for entry in (self.entry_bind, self.entry_port, self.entry_token, self.entry_fps, self.entry_ret):
            try:
                entry.configure(state=state)
            except Exception:
                pass

    def on_start(self):
        # Optionally save server config
        if self.var_launch_web.get():
            if not self.var_use_last.get():
                try:
                    cfg = {
                        'bind': self.entry_bind.get().strip() or DEFAULT_SERVER_CFG['bind'],
                        'port': int(self.entry_port.get().strip() or DEFAULT_SERVER_CFG['port']),
                        'token': self.entry_token.get().strip() or DEFAULT_SERVER_CFG['token'],
                        'mjpeg_fps': int(self.entry_fps.get().strip() or DEFAULT_SERVER_CFG['mjpeg_fps']),
                        'backup_retention': int(self.entry_ret.get().strip() or DEFAULT_SERVER_CFG['backup_retention']),
                    }
                except Exception:
                    messagebox.showerror('Error', 'Please enter valid numeric values for Port / MJPEG FPS / Backup Retention.')
                    return
                if not save_server_config(cfg):
                    return

        # Launch selected components in background threads
        if not (self.var_launch_web.get() or self.var_launch_gui.get()):
            messagebox.showinfo('Info', 'No components selected.')
            return
        def run_all():
            ok_all = True
            try:
                # Prefer built executables if present; fall back to Python scripts
                exe_web = os.path.join(BASE_DIR, 'WebServer.exe')
                exe_gui = os.path.join(BASE_DIR, 'ImageDetectionBot.exe')
                if self.var_launch_web.get():
                    if os.path.exists(exe_web):
                        self.proc_web = start_process([exe_web])
                    else:
                        if self.is_frozen:
                            messagebox.showerror('Error', 'Bundled WebServer.exe not found. Please rebuild or run from source.')
                            ok_all = False
                        else:
                            self.proc_web = start_process([self.python_exe, 'web_server.py'])
                    ok_all = ok_all and (self.proc_web is not None)
                if self.var_launch_gui.get():
                    if os.path.exists(exe_gui):
                        self.proc_gui = start_process([exe_gui])
                    else:
                        if self.is_frozen:
                            messagebox.showerror('Error', 'Bundled ImageDetectionBot.exe not found. Please rebuild or run from source.')
                            ok_all = False
                        else:
                            self.proc_gui = start_process([self.python_exe, 'bot_gui.py'])
                    ok_all = ok_all and (self.proc_gui is not None)
            finally:
                self.lbl_status.configure(text=('Launched successfully.' if ok_all else 'Launched with errors.'))

        threading.Thread(target=run_all, daemon=True).start()

    def on_open_portal(self):
        # Open the web dashboard in the default browser; prefer localhost for browser access
        cfg = load_server_config()
        port = cfg.get('port', 8765)
        tok = (cfg.get('token') or '').strip()
        q = f'?token={urllib.parse.quote(tok)}' if tok else ''
        url = f'http://localhost:{port}/{q}'
        try:
            webbrowser.open_new_tab(url)
            self.lbl_status.configure(text=f'Opened {url}')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to open web portal: {e}')

    def on_stop_gui(self):
        try:
            # First try via process handle
            if self.proc_gui and (self.proc_gui.poll() is None):
                self.proc_gui.terminate()
                try:
                    self.proc_gui.wait(timeout=3)
                except Exception:
                    pass
            # If still running or no handle, fallback to image name kill (Windows)
            self.proc_gui = None
            if platform.system() == 'Windows':
                exe_gui = os.path.join(BASE_DIR, 'ImageDetectionBot.exe')
                img = 'ImageDetectionBot.exe'
                if os.path.exists(os.path.join(BASE_DIR, 'ImageDetectionBotConsole.exe')):
                    img = 'ImageDetectionBotConsole.exe'
                try:
                    subprocess.run(['taskkill', '/IM', img, '/F'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    # best effort
                    pass
            self.lbl_status.configure(text='GUI stopped.')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to stop GUI: {e}')

    def on_stop_web(self):
        try:
            # First try via process handle
            if self.proc_web and (self.proc_web.poll() is None):
                self.proc_web.terminate()
                try:
                    self.proc_web.wait(timeout=3)
                except Exception:
                    pass
            self.proc_web = None
            # Fallback: kill by image name on Windows
            if platform.system() == 'Windows':
                try:
                    subprocess.run(['taskkill', '/IM', 'WebServer.exe', '/F'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
            self.lbl_status.configure(text='Web server stopped.')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to stop web server: {e}')


if __name__ == '__main__':
    app = LauncherApp()
    app.mainloop()