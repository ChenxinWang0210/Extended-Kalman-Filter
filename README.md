# Extended Kalman Filter Project 
Udacity Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./Images_for_README/LR_dataset1.png "dataset1"
[image2]: ./Images_for_README/LR_dataset2.png  "dataset2"

The goal of this project  is to build a kalman filter with C++ and utilize it  to estimate the state of a moving object of interest with noisy lidar and radar measurements. The evaluation metric is root mean squared error (RMSE) between the estimations and ground truth for the position and velocity of the moving object.

This project involves a simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). The C++ kalman filter uses  [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) to communicate with the simulator.

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

# Results
Two datasets were given by Udacity. A visulization of the path that the moving object takes for the two datasets is shown below.

![alt text][image1]

![alt text][image2]

Dataset 1 starts with a lidar measurement while dataset 2 starts with a radar measurement.

RMSE of the position (px, py) and velocity (vx, vy) of the moving object using both lidar and radar measurements

| RMSE 	| Dataset 1 	| Dataset 2 	|
|------	|-----------	|-----------	|
| px   	| 0.0974    	| 0.0726    	|
| py   	| 0.0855    	| 0.0967    	|
| vx   	| 0.4517    	| 0.4582    	|
| vy   	| 0.4404    	| 0.4971    	|

Udacity requires that RMSE should be less than or equal to the values \[0.11,0.11,0.52,0.52 \]. It is obvious that my results satisfy the requirement.

Look at the RMSE with both lidar and radar (LR), only lidar (L) and only radar (R) measurement for dataset 1

| RMSE 	| LR     	| L      	| R      	|
|------	|--------	|--------	|--------	|
| px   	| 0.0974 	| 0.1474 	| 0.2304 	|
| py   	| 0.0855 	| 0.1154 	| 0.3467 	|
| vx   	| 0.4517 	| 0.6390 	| 0.5840 	|
| vy   	| 0.4404 	| 0.5351 	| 0.8048 	|


Look at the RMSE with both lidar and radar (LR), only lidar (L) and only radar (R) measurement for dataset 2

| RMSE 	| LR     	| L      	| R      	|
|------	|--------	|--------	|--------	|
| px   	| 0.0726 	| 0.1170 	| 0.2693 	|
| py   	| 0.0967 	| 0.1263 	| 0.3848 	|
| vx   	| 0.4582 	| 0.6506 	| 0.6534 	|
| vy   	| 0.4971 	| 0.6114 	| 0.8638 	|

For both datasets, using both lidar and radar measurements gives a better RSME. Using only lidar measurements shows a lower RMSE than using only radar measurements, which indicates radar measurements are noisier than lidar measurements.

# Authors

* Chenxin Wang

Thanks for the contributions from Udacity













