# Evo evaluate ORB-SLAM3
In this example, I use EuRoc MH_01_easy, the address is:
>http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip

The MH_01_easy_data.csv comes from MH_01_easy/mav0/state_groundtruth_estimate0/data.csv
The MH_01_easy_data.tum comes from
```shell script
evo_traj euroc data.csv --save_as_tum
```

The MH_01_easy_FrameTrajectory_TUM_Format.txt comes from the result:
```shell script
cd shells
./euroc.sh
```

Evo evaluation:

```shell script
evo_ape tum MH_01_easy_data.tum MH_01_easy_f_dataset-MH01_monoi.txt -va --plot --plot_mode xyz --save_results orbslam_MH_01_easy.zip
```

Correct Steps:
```shell script
evo_traj tum f_dataset-room4_512_mono.tum -p
```

change readtxt.py to get data_tum_room4_512_16_calib.csv from data_tum_room4_512_16.csv
```
evo_traj tum data_tum_room4_512_16_calib.csv --save_as_tum
```
change readtxt.py to get f_dataset-room4_512_mono_calib.csv from f_dataset-room4_512_mono.txt
```
evo_traj tum f_dataset-room4_512_mono_calib.csv --save_as_tum
evo_traj tum data_tum_room4_512_16_calib.tum --ref=f_dataset-room4_512_mono_calib.tum -va -p -s
evo_ape tum data_tum_room4_512_16_calib.tum f_dataset-room4_512_mono_calib.tum -p -s -a --save_results orbslam3_mono_unros_tum_room4_512_16.zip
```



References:

>EVO Estimate SLAM 5 --- ORB-SLAM3 Evaluation
>
>https://blog.csdn.net/shanpenghui/article/details/109361766