# -*- coding: utf-8 -*-
from setuptools import setup, find_packages
import sys, os
import re

here = os.path.abspath(os.path.dirname(__file__))

v = open(os.path.join(here, 'src', 'plex', '__init__.py'))
version = re.compile(r".*__version__ = '(.*?)'", re.S).match(v.read()).group(1)
v.close()

try:
    README = open(os.path.join(here, 'README.txt')).read()
    CHANGES = open(os.path.join(here, 'CHANGES.txt')).read()
except IOError:
    README = CHANGES = ''

setup(name='plex',
      version=version,
      description="Plex is a library building lexical analysers",
      long_description=README + '\n\n' +  CHANGES,
      classifiers=[], # Get strings from http://pypi.python.org/pypi?%3Aaction=list_classifiers
      keywords='',
      author='Greg Ewing',
      author_email='greg@cosc.canterbury.ac.nz',
      maintainer= 'Stephane Klein',
      maintainer_email= 'stephane@harobed.org',
      url='http://www.cosc.canterbury.ac.nz/greg.ewing/python/Plex/',

      license='LGPL',
      packages=find_packages('src'),
      package_dir = {'': 'src'},include_package_data=True,
      zip_safe=False,
      install_requires=[
          # -*- Extra requirements: -*-
          'nose',
          'sphinx'
      ],
      entry_points="""
      # -*- Entry points: -*-
      """,
      test_suite = 'nose.collector'
      )
