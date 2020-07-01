# Sensor Fusion Nanodegree

## 3D Object Tracking Project
Luciano Silveira
July, 2020

### Specification

Track an Object in 3D Space

<img src="images/course_code_structure.png" width="779" height="414" />

The objective is to detect objects in an image using the YOLO deep-learning framework, associate regions in a camera image with Lidar points in 3D space and calculate the TTC (Time to Crash).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`.
3. Download the [yolov3.weights](https://pjreddie.com/media/files/yolov3.weights) file to `/dat/yolo/`.
4. Compile: `cmake .. && make`.
5. Run it: `./3D_object_tracking`.

### Match 3D Objects

`FP.1` implements the method `matchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property); the matches are the ones with the highest number of keypoint correspondences.

### Compute Lidar-based TTC

`FP.2` compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

### Associate Keypoint Correspondences with Bounding Boxes

`FP.3` prepares the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition are added to a vector in the respective bounding box.

Notice that outlier matches have been removed based on the euclidean distance between them in relation to all the matches in the bounding box.

### Compute Camera-based TTC

`FP.4` compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

### Performance

The algorithms were tested in different combinations and compared with regard to some performance measures.

#### Evaluation 1

`FP.5` Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

Several examples (2-3) have been identified and described in detail. The assertion that the TTC is off has been based on manually estimating the distance to the rear of the preceding vehicle from a top view perspective of the Lidar points.

#### Evaluation 2

`FP.6` Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

All detector / descriptor combinations implemented in previous chapters have been compared with regard to the TTC estimate on a frame-by-frame basis. To facilitate comparison, a spreadsheet and graph should be used to represent the different TTCs.

### Links

 * [Original Repository](https://github.com/udacity/SFND_3D_Object_Tracking)
 * [Project Rubric](https://review.udacity.com/#!/rubrics/2550/view)

### Appendix

Detailed execution by descriptor / detector can be checked on the [images folder](./data/).

|Detector/Descriptor|Camera TTC|Lidar TTC|Image sample|
|---|---|---|---|
|SHITOMASI/BRISK| | |[sample](./images/SHITOMASI_BRISK_0016.png)|
|SHITOMASI/BRIEF| | |[sample](./images/SHITOMASI_BRIEF_0016.png)|
|SHITOMASI/ORB| | |[sample](./images/SHITOMASI_ORB_0016.png)|
|SHITOMASI/FREAK| | |[sample](./images/SHITOMASI_FREAK_0016.png)|
|SHITOMASI/SIFT| | |[sample](./images/SHITOMASI_SIFT_0016.png)|
|HARRIS/BRISK| | |[sample](./images/HARRIS_BRISK_0016.png)|
|HARRIS/BRIEF| | |[sample](./images/HARRIS_BRIEF_0016.png)|
|HARRIS/ORB| | |[sample](./images/HARRIS_ORB_0016.png)|
|HARRIS/FREAK| | |[sample](./images/HARRIS_FREAK_0016.png)|
|HARRIS/SIFT| | |[sample](./images/HARRIS_SIFT_0016.png)|
|FAST/BRISK| | |[sample](./images/FAST_BRISK_0016.png)|
|FAST/BRIEF| | |[sample](./images/FAST_BRIEF_0016.png)|
|FAST/ORB| | |[sample](./images/FAST_ORB_0016.png)|
|FAST/FREAK| | |[sample](./images/FAST_FREAK_0016.png)|
|FAST/SIFT| | |[sample](./images/FAST_SIFT_0016.png)|
|BRISK/BRISK| | |[sample](./images/BRISK_BRISK_0016.png)|
|BRISK/BRIEF| | |[sample](./images/BRISK_BRIEF_0016.png)|
|BRISK/ORB| | |[sample](./images/BRISK_ORB_0016.png)|
|BRISK/FREAK| | |[sample](./images/BRISK_FREAK_0016.png)|
|BRISK/SIFT| | |[sample](./images/BRISK_SIFT_0016.png)|
|ORB/BRISK| | |[sample](./images/ORB_BRISK_0016.png)|
|ORB/BRIEF| | |[sample](./images/ORB_BRIEF_0016.png)|
|ORB/ORB| | |[sample](./images/ORB_ORB_0016.png)|
|ORB/FREAK| | |[sample](./images/ORB_FREAK_0016.png)|
|ORB/SIFT| | |[sample](./images/ORB_SIFT_0016.png)|
|AKAZE/AKAZE| | |[sample](./images/AKAZE_AKAZE_0016.png)|
|SIFT/BRISK| | |[sample](./images/SIFT_BRISK_0016.png)|
|SIFT/BRIEF| | |[sample](./images/SIFT_BRIEF_0016.png)|
|SIFT/FREAK| | |[sample](./images/SIFT_FREAK_0016.png)|
|SIFT/SIFT| | |[sample](./images/SIFT_SIFT_0016.png)|
