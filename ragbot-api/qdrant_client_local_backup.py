"""Shim loader to expose the installed `qdrant_client` package when
this repository contains a file with the same import name.

This module programmatically loads the installed package from the
virtualenv site-packages and inserts it into sys.modules under the
`qdrant_client` name so other modules can import it normally.

If your environment does not have the package installed, this shim will
raise an informative ImportError.
"""

import sys
import os
import importlib.machinery
import importlib.util

_this_dir = os.path.dirname(__file__)
# Try multiple possible venv site-packages locations (lib/lib64 and site.getsitepackages())
_venv_site_candidates = []
for _libdir in ("lib", "lib64"):
    _base = os.path.join(_this_dir, "venv", _libdir)
    if os.path.isdir(_base):
        for entry in os.listdir(_base):
            if entry.startswith("python"):
                _venv_site_candidates.append(os.path.join(_base, entry, "site-packages"))

_pkg_init = None
_venv_site = None
for p in _venv_site_candidates:
    if os.path.isdir(os.path.join(p, "qdrant_client")):
        _venv_site = p
        break

if not _venv_site:
    try:
        import site as _site_mod
        for p in _site_mod.getsitepackages():
            if os.path.isdir(os.path.join(p, "qdrant_client")):
                _venv_site = p
                break
    except Exception:
        _venv_site = None

if not _venv_site:
    # Last-resort fallback (common lib64 path)
    _venv_site = os.path.join(_this_dir, "venv", "lib64", "python3.12", "site-packages")

_pkg_init = os.path.join(_venv_site, "qdrant_client", "__init__.py")

if not os.path.exists(_pkg_init):
	raise ImportError("Cannot locate installed 'qdrant_client' package in venv site-packages.\n"
					  "Install the package in the venv or remove this shim.")

# Load the installed package directly from the venv site-packages

# Load the package using the canonical name so relative imports inside
# the package resolve correctly.
loader = importlib.machinery.SourceFileLoader("qdrant_client", _pkg_init)
spec = importlib.util.spec_from_loader("qdrant_client", loader)

module = importlib.util.module_from_spec(spec)
# Ensure the module is treated as a package so relative imports work
module.__path__ = [os.path.dirname(_pkg_init)]
loader.exec_module(module)

# Expose it as 'qdrant_client' for normal imports
sys.modules["qdrant_client"] = module

# Re-export public names
from qdrant_client import *  # noqa: F401,F403
