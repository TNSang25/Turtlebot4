#!/usr/bin/env python3
"""Install the FER Haar/TFLite runtime in an isolated ROS-compatible venv."""
import argparse
import os
from pathlib import Path
import subprocess
import venv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--venv', default='.venv-emotion')
    args = parser.parse_args()
    target = Path(args.venv).expanduser().resolve()
    package = Path(__file__).resolve().parents[1]
    config = target/'pyvenv.cfg'
    if not config.is_file() or 'include-system-site-packages = true' not in config.read_text():
        # Use distro python3-pip through system-site-packages. This also works
        # on ROS installations whose Python does not provide ensurepip.
        venv.EnvBuilder(with_pip=False, system_site_packages=True).create(target)
    python = str(target/'bin/python')
    pip = [python, '-m', 'pip', 'install', '--disable-pip-version-check', '--no-cache-dir']
    environment = {**os.environ, 'PYTHONNOUSERSITE': '1'}
    subprocess.run(pip+['-r', str(package/'requirements.txt')], check=True, env=environment)
    # FER declares dependencies for MTCNN, plots and video/audio export as well.
    # Importing fer.fer with mtcnn=False only needs the runtime installed above.
    subprocess.run(pip+['--no-deps', '-r', str(package/'requirements-fer.txt')], check=True, env=environment)
    subprocess.run([python, '-c',
                    'from fer.fer import FER; FER(mtcnn=False, use_tflite=True); '
                    'print("FER Haar/TFLite model ready (CPU, offline).")'], check=True, env=environment)
    print(f'Use python_executable:={python}')


if __name__ == '__main__':
    main()
