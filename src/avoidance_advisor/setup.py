import os

from setuptools import setup
from typing import List, Tuple

package_name = 'avoidance_advisor'


def list_data_files(target_dir: str) -> List[Tuple[str, List[str]]]:
    entries = []
    for root, _, files in os.walk(target_dir):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, os.path.relpath(root, '.'))
        src_files = [os.path.join(root, file_name) for file_name in files]
        entries.append((install_dir, src_files))
    return entries


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

for directory in ('launch', 'params'):
    if os.path.isdir(directory):
        data_files.extend(list_data_files(directory))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kazuki Ogata',
    maintainer_email='kaz.ogata1988@gmail.com',
    description='AIエージェントへ回避の助言を求めるノード',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'avoidance_advisor = avoidance_advisor.avoidance_advisor_node:main',
        ],
    },
)
