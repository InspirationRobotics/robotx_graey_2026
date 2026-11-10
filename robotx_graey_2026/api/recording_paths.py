"""ADAPTED: camera/sonar native files inside Crusader-style session folders."""
import re
from pathlib import Path


def session_record_path(root,session,kind,owner):
    if not isinstance(session,str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',session):raise ValueError('Invalid recording session')
    if kind not in ('camera','sonar') or not re.fullmatch(r'graey-[a-f0-9]{16}',owner):raise ValueError('Invalid recorder owner')
    base=Path(root).expanduser().resolve()
    path=(base/session/kind/(owner+('.mjpg' if kind=='camera' else ''))).resolve()
    if base not in path.parents:raise ValueError('Recording target escapes session root')
    return path
