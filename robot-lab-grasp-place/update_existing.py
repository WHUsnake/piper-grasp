"""Apply only the verified grasp timing and trajectory update to an existing Lab."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import time


ROOT = Path(__file__).resolve().parent
FILES = {
    'integration/integration/arm_trajectory.py': 'b6bfa9b9681a7261e0d8d5ff8855a0f3a23ff66849dd2d03f54853b571bbc450',
    'integration/integration/go2_grasp.py': '7f1c368a297b38db569b6258124fbcfe56de75c049e624a2057289127b7f6c7e',
    'integration/integration/go2_grasp_ros.py': '3ed30be9a885b70278ab269304af8195960032477e4e7055de7eae72233388e9',
}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', type=Path, required=True, help='Existing Robot Lab checkout')
    parser.add_argument('--apply', action='store_true', help='Back up and update the three files')
    args = parser.parse_args()
    host = args.host.resolve()
    if not (host/'bin/lab').is_file():
        raise SystemExit('Host is not a Robot Lab checkout')
    manifest = {item['path']: item['sha256'] for item in
                json.loads((ROOT/'SOURCE_MANIFEST.json').read_text())['files']}
    update = []
    for rel, old_hash in FILES.items():
        source, target = ROOT/rel, host/rel
        if not source.is_file() or not target.is_file() or target.is_symlink():
            raise SystemExit(f'Missing or linked grasp file: {rel}')
        if digest(source) != manifest[rel]:
            raise SystemExit(f'Published source hash mismatch: {rel}')
        current_hash = digest(target)
        if current_hash not in (old_hash, manifest[rel]):
            raise SystemExit(f'Existing file differs from the published base: {rel}; review its diff before updating')
        if current_hash != manifest[rel]:
            update.append(rel)
    print(json.dumps({'host': str(host), 'files_to_update': update, 'apply': args.apply}, indent=2))
    if not args.apply or not update:
        return
    backup = host.parent/(host.name+'-grasp-update-backup-'+time.strftime('%Y%m%d-%H%M%S'))
    backup.mkdir(exist_ok=False)
    for rel in update:
        saved = backup/rel
        saved.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(host/rel, saved)
    (backup/'source-hashes.json').write_text(json.dumps({rel: FILES[rel] for rel in update}, indent=2)+'\n')
    for rel in update:
        shutil.copy2(ROOT/rel, host/rel)
    print(json.dumps({'updated': update, 'backup': str(backup)}, indent=2))


if __name__ == '__main__':
    main()
