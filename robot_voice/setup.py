from pathlib import Path
from setuptools import find_packages, setup

assets = {}
for folder in ('config', 'launch', 'maps', 'models', 'licenses'):
    for path in Path(folder).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            assets.setdefault(f'share/robot_voice/{path.parent}', []).append(str(path))

setup(
    name='robot_voice', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_voice']),
        ('share/robot_voice', ['package.xml'] + (['requirements.txt'] if Path('requirements.txt').exists() else [])),
        *assets.items(),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace Maintainer', maintainer_email='maintainer@example.com',
    description='Offline Vietnamese speech recognition and semantic Nav2 navigation.', license='Apache-2.0',
    entry_points={'console_scripts': ['speech_recognition_node = robot_voice.speech_recognition_node:main', 'voice_navigation_node = robot_voice.voice_navigation_node:main']},
)
