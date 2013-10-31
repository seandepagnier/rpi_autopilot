import ctypes
c_float = ctypes.c_float

inertial_sensors_init = platform.createBaseFunction( 
    'inertial_sensors_init', dll=
