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
cd ORB_SLAM3_Fixed
cp f_dataset-room4_512_mono.txt results
cp kf_dataset-room4_512_mono.txt results
cd results
python process_orbslam_result.py
```
Change the file name in process_orbslam_result.py Line 6 and Line 25
```python
Line 6
with open('f_dataset-room4_512_mono.txt','r') as f:

Line 25
csvfile_write = open('f_dataset-room4_512_mono_calib.csv','w')
```
Process calibration files
```
evo_traj tum f_dataset-room4_512_mono_calib.csv -p
evo_traj tum f_dataset-room4_512_mono_calib.csv --save_as_tum
evo_traj tum data_tum_room4_512_16_calib.tum --ref=f_dataset-room4_512_mono_calib.tum -va -p -s
```
#### 3.Process the TUM groundtruth
```shell script
cp <path_to_tum_dataset/dataset-room4_512_16/mav0/mocap0/data.csv> results
```
Change the file name in process_orbslam_result.py Line 6 and Line 25
```python
Line 6
with open('data_tum_room4_512_16.csv','r') as f:

Line 25
csvfile_write = open('data_tum_room4_512_16_calib.csv','w')
```
Process:
```shell script
python process_dataset_gt.py
```
Test if trajectory correct
```shell script
evo_traj tum data_tum_room4_512_16_calib.csv -p
evo_traj tum data_tum_room4_512_16_calib.csv --save_as_tum
```

#### 4.Compare the TUM groundtruth and the ORB-SLAM3 result
```shell script
evo_traj tum f_dataset-room4_512_mono_calib.tum --ref=data_tum_room4_512_16_calib.tum -v -a -p -s
```

#### 5.Calculate ATE of ORB-SLAM3 Monocular
```shell script
evo_ape tum f_dataset-room4_512_mono_calib.tum data_tum_room4_512_16_calib.tum -v -a -p -s --save_results orbslam3_mono_unros_tum_room4_512_16.zip
```
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/Screenshot%20from%202020-11-10%2010-28-55.png)
![Result](https://github.com/shanpenghui/ORB_SLAM3_Fixed/blob/master/pics/Screenshot%20from%202020-11-10%2010-29-05.png)

#### 6.Process multiple results from a metric
```shell script
evo_res orbslam3_mono_unros_tum_room4_512_16.zip -p --save_table table.csv
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