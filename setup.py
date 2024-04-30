# numpydoc ignore=GL08
from pathlib import Path

from setuptools import setup

PACKAGE_NAME = 'pandaset2bag'

with (Path(__file__).absolute().parents[0] / PACKAGE_NAME / 'VERSION').open('r') as f:
    __version__ = f.read().strip()

with Path(
    'README.md',
).open('r', encoding='utf-8') as f:
    long_description = f.read()

with Path('requirements.txt').open('r') as f:
    requirements = f.read().splitlines()

setup(
    name=PACKAGE_NAME,
    version=__version__,
    description=('Command-line utility (and Python library) for converting the PandaSet Dataset to ROS2 bag files'),
    keywords='pandaset, ros, ros2, rosbag, dataset, pypi, package',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Alexander Hilgart',
    author_email='alexander.hilgart@b-plus.com',
    url='https://github.com/bplus-group/pandaset2bag',
    license='MIT',
    packages=[PACKAGE_NAME],
    package_dir={PACKAGE_NAME: PACKAGE_NAME},
    python_requires='>=3.10',
    install_requires=requirements,
    include_package_data=True,
    entry_points={
        'console_scripts': [f'{PACKAGE_NAME} = {PACKAGE_NAME}.__main__:main'],
    },
)
