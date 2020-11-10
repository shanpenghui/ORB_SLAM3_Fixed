# Evo evaluate ORB-SLAM3
Use evo tool to evaluate ORB-SLAM3

# EVO Unros Monocular ORB-SLAM3 TUM
## 1 - TUM Visual-Inertial Dataset
The address is:
>https://vision.in.tum.de/data/datasets/visual-inertial-dataset

In this example, I use dataset-room4_512_16.

### Usage Steps

#### 1.Run ORB-SLAM3
```shell script
cd shells
./tum_vi.sh
```
Get the result file as : 
>f_dataset-room4_512_mono.txt
>kf_dataset-room4_512_mono.txt
>


#### 2.Process the ORB-SLAM3 result file
Move the result files above to the results folder
```shell script
cd <path_to_save_result>
evo_traj tum <result_files> -p
python process_orbslam_result.py
```
Change the file name in process_orbslam_result.py Line 6 and Line 25
```python
Line 6
with open('dataset-room4_512_16/f_dataset-room4_512_mono.txt','r') as f:

Line 25
csvfile_write = open('dataset-room4_512_16/f_dataset-room4_512_mono_calib.csv','w')
```
Process calibration files
```shell script
evo_traj tum <calibration_files> -p
evo_traj tum <calibration_files> --save_as_tum
evo_traj tum <tum_result> --ref=<gt_file> -v -a -p -s
```
#### 3.Process the TUM groundtruth
```shell script
cp <raw_gt> <dst_path>
```
Change the file name in process_orbslam_result.py Line 6 and Line 25
```python
Line 6
with open('dataset-room4_512_16/data_tum_room4_512_16.csv','r') as f:

Line 25
csvfile_write = open('dataset-room4_512_16/data_tum_room4_512_16_calib.csv','w')
```
Process:
```shell script
python process_dataset_gt.py
```
Test if trajectory correct
```shell script
evo_traj tum <calibration_gt> -p
evo_traj tum <calibration_gt> --save_as_tum
```

#### 4.Compare the TUM groundtruth and the ORB-SLAM3 result
```shell script
evo_traj tum <tum_result> --ref=<tum_gt> -v -a -p -s
```
Or
```shell script
evo_traj tum <tum_result_1> <tum_result_2> --ref=<tum_gt> -v -a -p -s
```

#### 5.Calculate ATE of ORB-SLAM3 Monocular
Gt files must be in the first place
```shell script
evo_ape tum <tum_gt> <tum_result> -v -a -p -s --save_results <results>.zip
```

![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/Screenshot%20from%202020-11-10%2010-28-55.png)
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/Screenshot%20from%202020-11-10%2010-29-05.png)

#### 6.Process multiple results from a metric
```shell script
evo_res *.zip -p --save_table table.csv
```
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/image.png)
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/image1.png)
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/image2.png)
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/image3.png)
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/image4.png)



# References:

>EVO Estimate SLAM 5 --- ORB-SLAM3 Evaluation
>
>https://blog.csdn.net/shanpenghui/article/details/109361766