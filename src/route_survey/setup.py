from glob import glob
from os.path import isfile
from setuptools import setup
setup(name='route_survey', version='0.1.0', packages=['route_survey'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/route_survey']),
                  ('share/route_survey', ['package.xml', 'README.md']),
                  *[('share/route_survey/'+d, [p for p in glob(d+'/*') if isfile(p)]) for d in ['launch','params','docs','web','results']]],
      scripts=glob('tools/*.py'), install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
      maintainer='nkb', maintainer_email='nkb@example.com', license='Apache-2.0',
      description='手動経路採取・地面検査・地図編集',
      entry_points={'console_scripts':['recorder = route_survey.recorder_node:main']})
