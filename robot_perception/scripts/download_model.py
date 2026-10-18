#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Download the official YOLOv8n weights once, before launching perception."""
import argparse
import hashlib
from pathlib import Path
import urllib.request

SHA256 = 'f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path(__file__).resolve().parents[1] / 'models/yolov8n.pt')
    args = parser.parse_args()
    output = args.output.expanduser()
    if not output.exists():
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix('.pt.part')
        url = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt'
        try:
            with urllib.request.urlopen(url, timeout=90) as response, temporary.open('wb') as target:
                while chunk := response.read(1024 * 1024):
                    target.write(chunk)
            if hashlib.sha256(temporary.read_bytes()).hexdigest() != SHA256:
                raise RuntimeError('YOLOv8n checkpoint checksum mismatch')
            temporary.replace(output)
        finally:
            temporary.unlink(missing_ok=True)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    if digest != SHA256:
        raise RuntimeError(f'Existing {output} has an unexpected checksum')
    print(f'{output}\nSHA256: {digest}')


if __name__ == '__main__':
    main()
