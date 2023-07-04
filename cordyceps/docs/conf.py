# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys

sys.path.insert(0, os.path.abspath('..'))

project = 'Cordyceps'
copyright = '2023, Mart Coppelmans, Sara van Eersel, Jordi Espina Font, Thomas Udo, Cas Leeflang'
author = 'Mart Coppelmans, Sara van Eersel, Jordi Espina Font, Thomas Udo, Cas Leeflang'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ["sphinx.ext.todo", "sphinx.ext.githubpages", "sphinx.ext.autodoc", "sphinx.ext.viewcode"]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

autodoc_mock_imports = ["cordyceps_interfaces","numpy","rclpy","geometry_msgs","nav_msgs"]

latex_engine = 'xelatex'

latex_elements = {
    'papersize': 'letterpaper',
    'extraclassoptions': 'openany,oneside',
    'pointsize': '12pt',
    'preamble': '',
    'figure_align': 'htbp',
    'maketitle': r'''
        \pagenumbering{roman}
        \begin{titlepage}
            \centering
            \vspace*{2cm}
            {\Huge\bfseries Design Document}
            \par\vspace{2cm}
            {\Large Mart Coppelmans,}
            {\Large Sara van Eersel,}
            {\Large Jordi Espina Font,}
            {\Large Thomas Udo and}
            {\Large Cas Leeflang}
            \par\vfill
            {\Large July 2023}
        \end{titlepage}
        \pagenumbering{arabic}
    '''
}


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_logo = 'images/cordyceps_logo_transparent.png'
