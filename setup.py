#!/usr/bin/env python3

from setuptools import setup, find_packages

long_description = """
A python module for the pololu tic stepper motor drivers
"""

setup(name='tic-python',
      version='0.0.0',
      description='A python module for the pololu tic stepper motor drivers',
      long_description=long_description,
      long_description_content_type='text/markdown',
      author='Jacob Walser',
      author_email='jwalser90@gmail.com',
      url='https://github.com/jaxxzer/tic-python',
      packages=find_packages(), install_requires=['pyserial', 'future'],
      classifiers=[
          "Programming Language :: Python",
          "License :: OSI Approved :: MIT License",
          "Operating System :: OS Independent",
      ],
      scripts=[]
      )
