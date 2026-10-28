from pathlib import Path
from setuptools import find_packages, setup

assets = {}
for folder in ('config', 'launch', 'maps', 'models', 'licenses'):
    for path in Path(folder).rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            assets.setdefault(f'share/robot_semantic/{path.parent}', []).append(str(path))

setup(
    name='robot_semantic', version='0.1.0', packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_semantic']),
        ('share/robot_semantic', ['package.xml'] + (['requirements.txt'] if Path('requirements.txt').exists() else [])),
        *assets.items(),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Workspace Maintainer', maintainer_email='maintainer@example.com',
    description='Occupancy-map segmentation and semantic region visualization.', license='Apache-2.0',
    entry_points={'console_scripts': ['segment_map = robot_semantic.segmentation:main', 'semantic_map_node = robot_semantic.semantic_map_node:main']},
)
