import os
from typing import List, Tuple

from setuptools import setup

package_name = 'traffic_signal_recognizer'


def list_data_files(target_dir: str) -> List[Tuple[str, List[str]]]:
    """指定ディレクトリ配下のファイルを構造を保ったままインストールする。"""

    data_entries: List[Tuple[str, List[str]]] = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue

        rel_path = os.path.relpath(root, '.')
        install_dir = os.path.join('share', package_name, rel_path)
        src_files = [os.path.join(root, file_name) for file_name in files]
        data_entries.append((install_dir, src_files))

    return data_entries


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for subdir in ['params', 'launch', 'docs']:
    if os.path.isdir(subdir):
        data_files.extend(list_data_files(subdir))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Traffic signal GO/STOP recognizer based on YOLO detections',
    license='MIT',
    entry_points={
        'console_scripts': [
            'traffic_signal_recognizer = '
            'traffic_signal_recognizer.traffic_signal_recognizer_node:main',
        ],
    },
)
