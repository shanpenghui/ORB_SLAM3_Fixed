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

References:

>EVO Estimate SLAM 5 --- ORB-SLAM3 Evaluation
>
>https://blog.csdn.net/shanpenghui/article/details/109361766