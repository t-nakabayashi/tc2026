from glob import glob
from setuptools import setup

setup(name='icart_mapping', version='0.1.0', packages=['icart_mapping'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/icart_mapping']),
                  ('share/icart_mapping', ['package.xml', 'README.md']),
                  ('share/icart_mapping/pipeline', glob('pipeline/*.py'))],
      install_requires=['setuptools'], zip_safe=False,
      maintainer='nkb', maintainer_email='nkb@example.com', license='Apache-2.0',
      description='Isolated offline FIX and ICP graph map creation',
      entry_points={'console_scripts': ['build_map = icart_mapping.job:main']})
