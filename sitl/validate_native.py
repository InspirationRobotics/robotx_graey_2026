#!/usr/bin/env python3
"""Validate an unchanged source snapshot on native Linux storage, retain evidence."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--modes', nargs='+', default=['nominal', 'drop-ack', 'drop-source', 'nominal'],
                    choices=['nominal', 'drop-ack', 'drop-source'])
args = parser.parse_args()
stage = Path(tempfile.mkdtemp(prefix='graey-switch-', dir='/tmp'))
source = stage/'source'
shutil.copytree(root, source, ignore=shutil.ignore_patterns(
    '.git', 'build', 'install', 'log', 'logs', 'run', '__pycache__', '*.pyc'))
binary = Path(os.environ.get('SITL_BIN', str(root.parent/'output/gps-sitl-2026-10-02/ardusub')))
shutil.copy2(binary, stage/'ardusub')
(stage/'ardusub').chmod(0o700)
hashes = {str(p.relative_to(source)): hashlib.sha256(p.read_bytes()).hexdigest()
          for p in source.rglob('*') if p.is_file() and p.suffix in ('.py', '.sh', '.parm', '.lua')}
manifest = dict(stage=str(stage), binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
                source_sha256=hashes, results=[])
env = dict(os.environ, SITL_BIN=str(stage/'ardusub'))
evidence = root/'sitl/logs'/stage.name
evidence.mkdir(parents=True)
print('Native snapshot: '+str(stage), flush=True)
try:
    subprocess.run(['python3', '-m', 'unittest', 'discover', '-s', 'test'], cwd=source,
                   env=env, check=True)
    for mode in args.modes:
        result = subprocess.run(['python3', 'sitl/validate_switching.py', '--mode', mode],
                                cwd=source, env=env)
        manifest['results'].append(dict(mode=mode, returncode=result.returncode))
        shutil.copytree(source/'sitl/logs', evidence, dirs_exist_ok=True)
        (evidence/'manifest.json').write_text(json.dumps(manifest, indent=2))
finally:
    (evidence/'manifest.json').write_text(json.dumps(manifest, indent=2))
print('Retained evidence: '+str(evidence), flush=True)
raise SystemExit(0 if manifest['results'] and all(r['returncode'] == 0 for r in manifest['results']) else 1)
