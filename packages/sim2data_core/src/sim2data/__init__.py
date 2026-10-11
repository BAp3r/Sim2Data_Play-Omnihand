"""Shared namespace for the independently installed Sim2Data packages."""
from pkgutil import extend_path

__path__ = extend_path(__path__, __name__)
__version__ = "0.1.0"
