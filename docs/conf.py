from importlib.metadata import version as package_version

project = "zero-franky"
author = "Nick Walker"
release = package_version("zero-franky")

extensions = [
    "myst_parser",
    "sphinx.ext.autodoc",
    "sphinx.ext.intersphinx",
    "sphinx.ext.napoleon",
]

autodoc_member_order = "groupwise"
autodoc_typehints = "description"
toc_object_entries_show_parents = "hide"
myst_heading_anchors = 3

exclude_patterns = ["_build"]
html_theme = "furo"

intersphinx_mapping = {
    "franky": ("https://timschneider42.github.io/franky/", None),
}
