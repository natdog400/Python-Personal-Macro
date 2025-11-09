# -*- mode: python ; coding: utf-8 -*-

import os

block_cipher = None

proj_root = os.path.dirname(os.path.abspath(__file__))

# Include runtime configuration and assets
_datas = [
    (os.path.join(proj_root, 'server_config.json'), '.'),
    (os.path.join(proj_root, 'config.json'), '.'),
]

# Helper to include whole folders via Tree
from PyInstaller.building.build_main import Tree
_datas += Tree(os.path.join(proj_root, 'images'), prefix='images')
_datas += Tree(os.path.join(proj_root, 'failsafe_images'), prefix='failsafe_images')

# If the GUI needs web static for any embedded views, include it (harmless otherwise)
try:
    _web_static = os.path.join(proj_root, 'web', 'static')
    if os.path.isdir(_web_static):
        _datas += Tree(_web_static, prefix='web/static')
except Exception:
    pass

_pathex = [proj_root]
_hiddenimports = []

# Build the GUI with a visible console window to show logs
analysis = Analysis(
    ['bot_gui.py'],
    pathex=_pathex,
    binaries=[],
    datas=_datas,
    hiddenimports=_hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=[],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)
pyz = PYZ(analysis.pure, analysis.zipped_data, cipher=block_cipher)
exe = EXE(
    pyz,
    analysis.scripts,
    analysis.binaries,
    analysis.zipfiles,
    analysis.datas,
    [],
    name='ImageDetectionBotConsole',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,  # show console window for logs
    icon=None,
)

# One-folder build uses COLLECT; for one-file pass --onefile when invoking PyInstaller
collect = COLLECT(
    exe,
    analysis.binaries,
    analysis.zipfiles,
    analysis.datas,
    strip=False,
    upx=True,
    upx_exclude=[],
    name='ImageDetectionBotConsole'
)