# Write Up
#### FP.1 Match 3D Objects: 
The matchBoundingBoxes function is implemented with std::multimap<int,int> which stores pairs of bounding box IDs. The number of matches in the multimap are counted and we find which pair of current and previos IDs have the most matches. 

#### FP.2 Compute Lidar-based TTC:
Here, we take the average of the x values of the previous and current frame's lidar points. The average of those is d0 and d1 respectivly. We then find the time delta which is 1 / framerate. Then we calculate the TTC using TTC = d1 * dt / (d0 - d1).

#### FP.3 Associate Keypoint Correspondences with Bounding Boxes
Here we loop through all keypoint matches and add the match to boundingBox if the keypoint is in the ROI.

#### FP.4 Compute Camera-based TTC:
Here we compute the camera TTC. This is done using the code already provided earlier in the section. It does this using the equation TTC = (-1.0 / frameRate) / (1 - medianDistRatio).

#### FP.5 Performance Evaluation 1:
The highest TTC error (while using the best combination as determined in the midterm - FAST/ORB) is in frame 5 where theTTC for the camera jumped to 25 seconds, then returned to normal values in the next frame. This could be due to outliers in the keypoints or too large of a ROI as there seem to be several keypoints outside the car. Another example is in frame 4 using FAST|BRISK where the camera TTC is 43 seconds. As far as the lidar TTC, there do not seem to be any significant outliers. It was consistantly around 13 seconds until the distance shrunk and the TTC was reduced to around 8 seconds. Since the average of the lidar points was taken, this might be avoiding outliers in the data that would be in ther eotherwise. 

#### FP.6 Performance Evaluation 2:
The combinations of FAST|ORB, FAST|BRIEF, FAST | BRISK were tested. Around the 5th frame, the model tends to over esitmate the TTC. But by the next frame, it is back to normal. This can be seen when compared to the lidar TTC which remains consistent. This might be an error caused by too many keypoints.

<img src="images/final camera spreadsheet.jpg" width="569" height="417" />


# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
