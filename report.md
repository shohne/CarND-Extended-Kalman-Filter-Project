# **Vehicle Detection and Tracking**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road

### Pipeline Overview

Before processing each image, it is necessary to calibrate the camera to correct from distortions. This happens in **getCalibrationParameters** method where it is possibleto get parameters to adjust every image processed in the pipeline.

Each image or video frame is processed in **pipeline** method that following the steps:

1. Undistort the image (with calibration parameters);
2. Apply birds eye viewer (method **warp**) where points (700,445) - (1280,720) - (0,720) - (580-445) are mapped to rectangule (1280, 0) -(1280, 720) - (0, 720) - (0,0);
3. Convert image to HLS colorspace;
4. Consider only S channel in image;
5. Apply Sobel transformation in X direction to get only vertical lines in image;
6. Apply threshold to image;
7. Run method findmaskline (provided by Udacity) to isolate lane pixels;
8. Adjust a second order function to trace left lane (polynomial fit) and find its coefficients;
9. Adjust a second order function to trace right lane;
10. Update coefficients history (with last 20 values);
11. Verify if values on left and right lanes are compatible with previous values;
12. Calculate new value for coefficients (as a mean over historical values);
13. Find **curvature** (method **calculate_curvature**) and position in lane (method calculate_center_dist).
14. Draw green region to indicate detected lane area;
15. Return unwarped image.

### Detailed Steps
Visualize the pipeline showing the result for each step. Consider the original image:

![](test_images/straight_lines1.jpg)

Following pipeline steps, We obtain:

![](output/straight_lines1_0.jpg)

![](output/straight_lines1_0.jpg)

![](output/straight_lines1_1.jpg)

![](output/straight_lines1_2.jpg)

![](output/straight_lines1_3.jpg)

![](output/straight_lines1_4.jpg)

![](output/straight_lines1_5.jpg)

![](output/straight_lines1_6.jpg)

![](output/straight_lines1_F.jpg)

### Video
There is a important step in pipeline that checks if prediction values makes sense and to protect against suddenly changes calculate the mean of historical values for curvature. Only for a temporal sequence of images it is possible to see this behavior. In:

[project_video.mp4](output/project_video.mp4)

We can see the pipeline applied in video:

[original project_video.mp4](project_video.mp4).

### Shortcomings
I believe that the main shortcoming it is related as specific the solution is to a certain type of road. I am almost sure that in other weather, light or road situations, this pipeline will fail. A better approch could be use **Deep Learning** that is able to adjust to different cenarios (since enough data is provided).
