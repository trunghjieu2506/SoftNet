import sys, os
def createScene(root):
    print("exe:", sys.executable)
    print("sys.path[:8]:", sys.path[:8])
    try:
        import numpy as np
        print("numpy from:", np.__file__, "ver:", np.__version__)
        import numpy.core._multiarray_umath as m
        print("core from:", m.__file__)
    except Exception as e:
        print("FAILED numpy import:", e)