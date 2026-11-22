"""Install the reviewed grasp/place overlay into a clean, pinned Robot Lab checkout."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time


ROOT = Path(__file__).resolve().parent
PIN = '1ed97b03530d00d6a53e6e4f4f97f8a0d394e9c9'


def git(host, *args):
    return subprocess.check_output(['git', '-C', str(host), *args], text=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', type=Path, required=True,
                        help='Clean Robot Lab checkout at the pinned base commit')
    parser.add_argument('--apply', action='store_true', help='Copy files after validation')
    args = parser.parse_args()
    host = args.host.resolve()
    if not (host/'bin/lab').is_file() or git(host, 'rev-parse', 'HEAD') != PIN:
        raise SystemExit(f'Host must be Robot Lab at {PIN}')
    if git(host, 'status', '--porcelain'):
        raise SystemExit('Host has local changes; preserve them before applying this overlay')
    manifest = json.loads((ROOT/'SOURCE_MANIFEST.json').read_text())
    files = []
    for item in manifest['files']:
        rel = Path(item['path'])
        source = ROOT/rel
        if rel.is_absolute() or '..' in rel.parts or not source.is_file():
            raise SystemExit(f'Invalid release path: {rel}')
        actual = hashlib.sha256(source.read_bytes()).hexdigest()
        if actual != item['sha256']:
            raise SystemExit(f'Release hash mismatch: {rel}')
        if item['overlay_to_robot_lab']:
            files.append(str(rel))
    print(json.dumps({'host': str(host), 'file_count': len(files), 'apply': args.apply}, indent=2))
    if not args.apply:
        return
    backup = host.parent/(host.name+'-grasp-backup-'+time.strftime('%Y%m%d-%H%M%S'))
    if backup.exists():
        raise SystemExit('Backup destination already exists')
    backup.mkdir()
    replaced = []
    for rel in files:
        target = host/rel
        if target.exists():
            saved = backup/rel
            saved.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(target, saved)
            replaced.append(rel)
    (backup/'replaced.json').write_text(json.dumps(replaced, indent=2)+'\n')
    for rel in files:
        source = ROOT/rel
        target = host/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
    print(json.dumps({'installed': len(files), 'backup': str(backup)}, indent=2))


if __name__ == '__main__':
    main()
