# -*- mode: python ; coding: utf-8 -*-

import os

block_cipher = None

# In spec context, __file__ may not be defined. Use current working directory.
proj_root = os.path.abspath('.')

# Data files to include in the bundle
_datas = [
    (os.path.join(proj_root, 'server_config.json'), '.'),
    (os.path.join(proj_root, 'config.json'), '.'),
]

# Helper to include whole folders
from PyInstaller.building.datastruct import Tree

# Common analysis options
_pathex = [proj_root]
_hiddenimports = []  # Add modules if PyInstaller misses any

# GUI EXE (bot_gui.py)
a1 = Analysis(
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
pyz1 = PYZ(a1.pure, a1.zipped_data, cipher=block_cipher)
exe1 = EXE(
    pyz1,
    a1.scripts,
    a1.binaries,
    a1.zipfiles,
    a1.datas,
    [],
    name='ImageDetectionBot',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    icon=None,
)

# Console EXE (web_server.py)
a2 = Analysis(
    ['web_server.py'],
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
pyz2 = PYZ(a2.pure, a2.zipped_data, cipher=block_cipher)
exe2 = EXE(
    pyz2,
    a2.scripts,
    a2.binaries,
    a2.zipfiles,
    a2.datas,
    [],
    name='WebServer',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=True,
    icon=None,
)

# GUI EXE (launcher.py)
a3 = Analysis(
    ['launcher.py'],
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
pyz3 = PYZ(a3.pure, a3.zipped_data, cipher=block_cipher)
exe3 = EXE(
    pyz3,
    a3.scripts,
    a3.binaries,
    a3.zipfiles,
    a3.datas,
    [],
    name='Launcher',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=True,
    upx_exclude=[],
    runtime_tmpdir=None,
    console=False,
    icon=None,
)

collect = COLLECT(
    exe1,
    exe2,
    exe3,
    a1.binaries + a2.binaries + a3.binaries,
    a1.zipfiles + a2.zipfiles + a3.zipfiles,
    a1.datas + a2.datas + a3.datas,
    # Include asset directories as Trees at COLLECT stage (expects TOC-like objects here)
    Tree(os.path.join(proj_root, 'web', 'static'), prefix='web/static'),
    Tree(os.path.join(proj_root, 'images'), prefix='images'),
    Tree(os.path.join(proj_root, 'failsafe_images'), prefix='failsafe_images'),
    strip=False,
    upx=True,
    upx_exclude=[],
    name='ProjectBundle'
)