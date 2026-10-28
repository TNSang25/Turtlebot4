#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Download a pinned multilingual Whisper model; inference stays offline."""

import argparse
import hashlib
from pathlib import Path
import tempfile
import urllib.request

REPOSITORY = 'Systran/faster-whisper-small'
REVISION = '536b0662742c02347bc0e980a01041f333bce120'
FILES = {
    'config.json': ('git', 'e5047537059bd8f182d9ca64c470201585015187'),
    'model.bin': ('sha256', '3e305921506d8872816023e4c273e75d2419fb89b24da97b4fe7bce14170d671'),
    'tokenizer.json': ('git', '7818adb6de9fa3064d3ff81226fdd675be1f6344'),
    'vocabulary.txt': ('git', 'c9074644d9d1205686f16d411564729461324b75'),
}


def verified(path, kind, expected):
    if not path.is_file():
        return False
    digest = hashlib.sha256() if kind == 'sha256' else hashlib.sha1()
    if kind == 'git':
        digest.update(f'blob {path.stat().st_size}\0'.encode())
    with path.open('rb') as source:
        while chunk := source.read(1024*1024):
            digest.update(chunk)
    return digest.hexdigest() == expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', default=str(Path(__file__).resolve().parents[1]/'models'))
    options = parser.parse_args()
    directory = Path(options.directory).expanduser().resolve()
    destination = directory/'faster-whisper-small'
    if destination.exists():
        if all(verified(destination/name, *spec) for name, spec in FILES.items()):
            print(f'Verified model: {destination}')
            return
        raise SystemExit(f'Incomplete/modified model: {destination}; remove it and download again')
    directory.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.whisper-download-', dir=directory) as temp:
        work = Path(temp)
        for name, spec in FILES.items():
            print(f'Downloading {name}...', flush=True)
            url = f'https://huggingface.co/{REPOSITORY}/resolve/{REVISION}/{name}'
            with urllib.request.urlopen(url, timeout=60) as source, (work/name).open('wb') as output:
                while chunk := source.read(1024*1024):
                    output.write(chunk)
            if not verified(work/name, *spec):
                raise SystemExit(f'Checksum mismatch: {name}')
        work.rename(destination)
    print(f'Installed: {destination}\nRevision: {REVISION}')


if __name__ == '__main__':
    main()
