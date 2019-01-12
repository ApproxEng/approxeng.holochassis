__author__ = 'tom'
from setuptools import setup, find_packages

# Makes use of the sphinx and sphinx-pypi-upload packages. To build for local development
# use 'python setup.py develop'. To upload a version to pypi use 'python setup.py clean sdist upload'.
# To build docs use 'python setup.py build_sphinx' and to upload docs to pythonhosted.org use
# 'python setup.py upload_sphinx'. Both uploads require 'python setup.py register' to be run, and will
# only work for Tom as they need the pypi account credentials.

setup(
    name='approxeng.holochassis',
    version='0.2',
    description='Python code for holonomic chassis dynamics',
    classifiers=['Programming Language :: Python :: 3.4'],
    url='https://github.com/ApproxEng/approxeng.holochassis/',
    author='Tom Oinn',
    author_email='tomoinn@gmail.com',
    license='ASL2.0',
    packages=find_packages(),
    install_requires=['numpy==1.12.1'],
    include_package_data=True,
    test_suite='nose.collector',
    tests_require=['nose'],
    dependency_links=[],
    zip_safe=False)
