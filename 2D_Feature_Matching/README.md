# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. I have built a feature tracking part and tested various detector / descriptor combinations to see which ones perform best.

* For loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
  * Implemented a linked list where each node stores a dataframe and link to the next node. 
  * While inserting a new node, it checks for the length of the linked list. If it’s less than the buffersize, it adds a new node. If it is equal to buffersize, the head      pointer points to head-> next and the initial head node is removed. Then a new node is inserted.
  * Code for linkedlist is in dataStructures.cpp

* Integrated several keypoint detectors and compared them with regard to number of keypoints and speed. 
  * **Keypoint Detection:** 
    * detectorType variable can be SHITOMASI, HARRIS, SIFT, FAST, ORB, BRISK, AKAZE. 
    * Based on the detectorType, variable the corresponding detector is used to keypoint detection.

  * **Keypoint Removal:**
    * Keypoints inside the defined rectangle are only stored.
    *  The number of keypoints in the rectangle is reported in [2D_Feature_Tracking_Log.csv](https://github.com/ajaynarasimha/Sensor-Fusion/blob/main/2D_Feature_Matching/2D_Feature_tracking_Log.csv) 

  * **Detector Algorithm Evaluation:**
    * The number of keypoints on preceding vehicle for all the 10 images reported in [2D_Feature_Tracking_Log.csv](https://github.com/ajaynarasimha/Sensor-Fusion/blob/main/2D_Feature_Matching/2D_Feature_tracking_Log.csv). This has been repeated for all the keypoint detection techniques (SHITOMASI, HARRIS, SIFT, FAST, ORB, BRISK and AKAZE).
    * Although BRISK generates around 250 keypoints of the front vehicle, it takes 40s for computing.
    * Considering speed and performance, Fast Detector can generate around 150 keypoints of the front vehicle in 1s. 

* Added descriptor extraction and matching using brute force and also the FLANN. 
  * **Keypoint Descriptors:**
    * descriptorType variable can be BRISK, BRIEF, SIFT, FREAK, ORB, AKAZE.
    * Based on the descriptorType variable, the corresponding extractor is used.
    * If detectorType is AKAZE, then descriptorType should be AKAZE.

  * **Descriptor Matching:**
    * matcherType variable can be MAT_BF , MA_FLANN       
      * For Keypoint matching, both Brute force matching(MAT_BF) and Flann matching(MAT_FLANN) are implemented.
      * Flann matching works with only HOG descriptors, but brute force matching works for both HOG and binary descriptors. 
    * selectorType variable can be MAT_NN, MAT_KNN
      * Implemented both Nearest Neighbour approach k-Nearest Neighbour approach (with k=2) to obtain matches.
      * For k-NN approach, a match is considered as a good match only if distance ratio of the two matches is less than 0.8 

  * **Descriptor Algorithm Evaluation:**
    * In [2D_Feature_Tracking_Log.csv](https://github.com/ajaynarasimha/Sensor-Fusion/blob/main/2D_Feature_Matching/2D_Feature_tracking_Log.csv), time taken for keypoint detection, descriptor extraction and matching for all the 10 images with all the possible detector-descriptor combinations are reported.
    * Considering speed and performance, BRISK, BRIEF, ORB descriptors can be generated in 1s, with matches more than 75% of the keypoints detected.
    * Although SIFT Descriptor performance is close to 75%, it takes around 40s for generating descriptors.


* Different combinations of detector and descriptor algorithms are tested to compare the speed and performance.        
  * **Final Evaluation:**
    * In autonomous vehicles, the execution time (for keypoint detection, descriptor extraction and matching) should be very crucial. 
      These 3 combinations are executed in the shortest time while maintaining good matching features:
      * **_FAST + BRISK_** : the execution time for 10 images is 39ms
      * **_FAST + BRIEF_** : the execution time for 10 images is 44ms
      * **_FAST + ORB_** : the execution time for 10 images is 68ms



## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
