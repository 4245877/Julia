"""Compatibility entry point: three CTest builds share one isolated output root.

All builds are prepared before any test starts. Actual writes are monitored by the
shared runner, so parallel builds do not mistake each other's outputs for leaks.
"""
import sys
sys.dont_write_bytecode = True
from stage7k_ctest import main

if __name__ == '__main__':
    main()
