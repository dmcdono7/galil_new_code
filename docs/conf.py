# Configuration file for the Sphinx documentation builder.

project = 'Galil ROS 2 Controllers'
copyright = '2024, Douglas McDonough'
author = 'Douglas McDonough'
release = 'main'

# Add myst_parser to read .md files
extensions = [
  'myst_parser',
  'sphinx_copybutton'
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# Theme settings
html_theme = "sphinx_rtd_theme"
