from setuptools import setup, find_packages

setup(name='mdde',
      version='0.0.1',
      description='Multi-Agent Goal-Driven Drone Environment',
      author='Mouhamed Naby Ndiaye',
      packages=find_packages(),
      include_package_data=True,
      zip_safe=False,
      install_requires=['gym']
      )
