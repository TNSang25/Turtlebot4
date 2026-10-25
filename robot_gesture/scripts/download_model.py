#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fetch the official, pinned MediaPipe Gesture Recognizer model before launch."""

import argparse
import hashlib
from pathlib import Path
import urllib.request

URL = 'https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task'
SHA256 = '97952348cf6a6a4915c2ea1496b4b37ebabc50cbbf80571435643c455f2b0482'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
                        default=Path(__file__).resolve().parents[1]/'models/gesture_recognizer.task')
    path = parser.parse_args().output.expanduser()
    if not path.exists():
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.task.part')
        try:
            with urllib.request.urlopen(URL, timeout=90) as response, temporary.open('wb') as output:
                while chunk := response.read(1024*1024):
                    output.write(chunk)
            if hashlib.sha256(temporary.read_bytes()).hexdigest() != SHA256:
                raise RuntimeError('Gesture model checksum mismatch')
            temporary.replace(path)
        finally:
            temporary.unlink(missing_ok=True)
    if hashlib.sha256(path.read_bytes()).hexdigest() != SHA256:
        raise RuntimeError(f'Unexpected checksum: {path}')
    print(f'{path}\nSHA256: {SHA256}')


if __name__ == '__main__':
    main()
