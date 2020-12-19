
# Point Registration for 3D Point Clouds #
## Input Point Clouds ##
Model Set - Fountain\
![ModelSetFntn](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/ModelSet_Fountain.png)\
Data Set - Fountain\
![DSFntn](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/DataSet_Fountain.png)\
Point cloud - Cow\
![PointCloudCow](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/CowDataSet00.png)\
Point cloud - Cow and noise added to it to rotate the point cloud by 30 degrees\
![PointCloudNoisyCow](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/CowNoisy00.png)

## Iterative Closest Point (ICP) Algorithm for 3D point registration ##
Registered point set using ICP for fountain point cloud\
![RegisFntnICP1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Fountain_ICP_SVD01.png)\
![RegisFntnICP2](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Fountain_ICP_SVD00.png)\
Registered point cloud using ICP for cow point cloud\
![RegisICPCow](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Cow_ICP_SVD00.png)


## Trimmed Iterative Closest Point (TrICP) Algorithm for 3D point registration ##
Registered point set using TrICP for fountain point cloud\
![RegisteredSetICP](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Fountain_SVD_TrICP01.png)\
![RegisteredSetICP1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Fountain_SVD_TrICP00.png)\
Registered point cloud using TrICP for cow point cloud\
![RegisteredSetICPCow](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Cow_TrICP_SVD00.png)


## Performance Comparison - ICP and TrICP ##

For rotation angle of 30 degrees, comparison between ICP and TrICP is as follows:
|    Parameter  |      ICP      |  TrICP  |
| ------------- |:-------------:| -------:|
| Mean sq error | 0.0000813     | 0.00009 |
| time for convergence     | 0.111016 min      |  0.10811 min   |

| | |
|:-------------------------:|:-------------------------:|
|<img width="1203" alt="BMF_1_7" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Teapot_ICP_30degs00.png">  ICP, rot angle : 30 |<img width="1203" alt="BMF_1_14" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Teapot_TrICP_30deg00.png">  TrICP, rot angle : 30 |

For rotation angle of 20 degrees, comparison between ICP and TrICP is as follows:
|    Parameter  |      ICP      |  TrICP  |
| ------------- |:-------------:| -------:|
| Mean sq error | 0.00006287     | 0.00007299 |
| time for convergence     | 0.06355 min      |  0.060677 min   |

| | |
|:-------------------------:|:-------------------------:|
|<img width="1203" alt="BMF_1_7" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Teapot_ICP_20degs00.png">  ICP, rot angle : 20 |<img width="1203" alt="BMF_1_14" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/PointRegistration/Output/Teapot_TrICP_20degs00.png">  TrICP, rot angle : 20 |

As can be seen from the results, for a small dataset of 1177 points, TrICP is faster than ICP and produces better result for rotation angles < 30 degrees






