import numpy as np

a=np.load('orbslam3_mono_unros_tum_room4_512_16/error_array.npz')
b=np.load('orbslam3_mono_unros_tum_room4_512_16/seconds_from_start.npz')
c=np.load('orbslam3_mono_unros_tum_room4_512_16/timestamps.npz')
print(a)
print(b)
print(c)