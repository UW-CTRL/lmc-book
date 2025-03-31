import os
from glob import glob
from setuptools import setup

setup(
    name="lmc-book",
    version="0.0.1",
    description="Style and Imports for lmc book",
    author="Karen Leung",
    author_email="kymleung@uw.edu",
    license="MIT",
    packages=["dmol"],
    install_requires=[
        "jupyter-book==0.13.1",
        "matplotlib",
        "numpy",
        "ipython!=8.7.0",
        "jax",
        "jaxlib",
        "pillow>=8.3.2",
        "linkify-it-py",
        "cvxpy",
        "lxml_html_clean",
    ],
    test_suite="tests",
    long_description="""
# Style and Imports for lmc Book

This is based on the style and imports for deep learning for molecules and materials
written by Andrew White. Please see the [dmol book](https://dmol.pub) or
root package at [github](https://github.com/whitead/dmol-book).
""",
    long_description_content_type="text/markdown",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Topic :: Scientific/Engineering :: Controls",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Typing :: Typed",
    ],
)
