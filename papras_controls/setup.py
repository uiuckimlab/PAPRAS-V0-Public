from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['papras_controls'],
    package_dir={'': 'scripts'}
)
setup(**d)