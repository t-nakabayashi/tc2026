from setuptools import setup

setup(name='icp_localization', version='0.1.0', packages=['icp_localization'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/icp_localization']),
                  ('share/icp_localization', ['package.xml', 'README.md'])],
      install_requires=['setuptools', 'numpy', 'scipy'], zip_safe=True,
      maintainer='Kazuki Ogata', maintainer_email='kaz.ogata1988@gmail.com',
      description='Fixed-map ICP localization with the common ENU pose interface',
      license='Apache-2.0',
      entry_points={'console_scripts': [
          'localization_node = icp_localization.node:main']})
