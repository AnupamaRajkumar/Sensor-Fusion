#Task 1 --> Bilateral Filtering
Original Image\
![im](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Input/brucke.jpg)\
Bilateral Filtering\
![BilateralFilter](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralFilter.png)\
Bilateral Median Filtering\
![BilateralMedianFilter](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFilter.png)\
| | | | |
|:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:|
|<img width="1203" alt="BMF_1_7" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_1.000000_7.000000.png">  spat:1, rad:7 |<img width="1203" alt="BMF_1_14" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_1.000000_14.000000.png">  spat:1, rad:14 |<img width="1203" alt="BMF_1_21" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_1.000000_21.000000.png"> spat:1, rad:21|<img width="1203" alt="BMF_1_28" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_1.000000_28.000000.png"> spat:1, rad:28|
|<img width="1203" alt="BMF_2_7" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_2.000000_7.000000.png">  spat:2, rad:7 |<img width="1203" alt="BMF_2_14" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_2.000000_14.000000.png">  spat:2, rad:14|<img width="1203" alt="BMF_2_21" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_2.000000_21.000000.png"> spat:2, rad:21|<img width="1203" alt="BMF_2_28" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_2.000000_28.000000.png"> spat:2, rad:28|
|<img width="1203" alt="BMF_3_7" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_3.000000_7.000000.png">  spat:3, rad:7 |<img width="1203" alt="BMF_3_14" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_3.000000_14.000000.png">  spat:3, rad:14 |<img width="1203" alt="BMF_3_21" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_3.000000_21.000000.png"> spat:3, rad:21|<img width="1203" alt="BMF_3_28" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_3.000000_28.000000.png"> spat:3, rad:28|
|<img width="1203" alt="BMF_7_7" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_7.000000_7.000000.png">  spat:7, rad:7 |<img width="1203" alt="BMF_7_14" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_7.000000_14.000000.png">  spat:7, rad:14 |<img width="1203" alt="BMF_7_21" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_7.000000_21.000000.png"> spat:7, rad:21|<img width="1203" alt="BMF_7_28" src="https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/BilateralMedianFiltering_7.000000_28.000000.png"> spat:7, rad:28|


#Task2 --> Guided Joint Bilateral Upsampling
Original Image\
![im1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Input/im1.png)\
Downsampled image before straightforward upsampling\
![Straightforward_Downsampled_im1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/Straightforward_Downsampled_im1.png)\
Straightforward Upsampling\
![Straightforward_Upsampled_im1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/Straightforward_Upsampled_im1.png)\
Downsampled image before iterative upsampling\
![Iterative_Downsampled_im1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/Iterative_Downsampled_im1.png)\
Iterative Upsampling\
![Iterative_Upsampled_im1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Output/Iterative_Upsampled_im1.png)\
Ground truth\
![GroundTruth_im1](https://github.com/AnupamaRajkumar/3DSensing_SensorFusion/blob/master/GuidedImageUpsampling/Input/disp1.png)
