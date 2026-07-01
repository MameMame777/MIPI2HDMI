"""Runs at interpreter startup for the cocotb *simulation* process (the Verilated exe with
embedded Python), which finds this file via the ``PYTHONPATH`` cocotb's runner exports.

Windows resolves an extension module's (``.pyd``) dependent DLLs from the *executable's*
directory + AddDllDirectory dirs + System32 -- NOT from ``PATH``. The Verilated exe lives
in the per-block build dir, so ucrt64's runtime DLLs (libpython, libgcc_s, libwinpthread,
libstdc++, ...) that every stdlib ``.pyd`` needs are invisible, and even ``import binascii``
fails with "DLL load failed". :func:`os.add_dll_directory` registers those dirs so the
loader can find them. The dirs are passed in via ``COCOTB_DLL_DIRS`` (set by
``runner_support``) to avoid importing project modules this early.
"""
import os

for _d in os.environ.get("COCOTB_DLL_DIRS", "").split(os.pathsep):
    if _d and os.path.isdir(_d):
        try:
            os.add_dll_directory(_d)
        except OSError:
            pass
