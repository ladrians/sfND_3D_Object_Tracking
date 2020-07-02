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
2. Check [dependencies for running locally](https://github.com/udacity/SFND_3D_Object_Tracking#dependencies-for-running-locally).
3. Make a `build` directory in the top level project directory: `mkdir build && cd build`.
4. Download the [yolov3.weights](https://pjreddie.com/media/files/yolov3.weights) file to the `/dat/yolo/` folder.
5. Compile: `cmake .. && make`.
6. Run it: `./3D_object_tracking`.

### Match 3D Objects

Initially the YOLO deep-learning framework will classify the detections on the scene:

![YOLO sample](./images/./images/objectclassification01.png)

`FP.1` implements the method `matchBoundingBoxes`, which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property); the matches are the ones with the highest number of keypoint correspondences.

![Detection sample](./images/detect01.png)

### Compute Lidar-based TTC

`FP.2` compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

![Birds eye view sample](./images/birdseyeview01.png)

### Associate Keypoint Correspondences with Bounding Boxes

`FP.3` prepares the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition are added to a vector in the respective bounding box.

Analyzing the images, the detection ranges starting from [7.97m](./images/3d_object_detect01.png) to [6.81m](./images/3d_object_detect03.png) as they approach the semaphore with red light, for example:

![3D object detection](./images/3d_object_detect02.png)

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
|SHITOMASI/BRISK|8.898673|10.845162|[sample](./images/SHITOMASI_BRISK_0016.png)|
|SHITOMASI/BRIEF|8.898673|11.795735|[sample](./images/SHITOMASI_BRIEF_0016.png)|
|SHITOMASI/ORB|8.898673|11.203624|[sample](./images/SHITOMASI_ORB_0016.png)|
|SHITOMASI/FREAK|8.898673|10.716109|[sample](./images/SHITOMASI_FREAK_0016.png)|
|SHITOMASI/SIFT|8.898673|10.210745|[sample](./images/SHITOMASI_SIFT_0016.png)|
|HARRIS/BRISK|8.898673|6.338662|[sample](./images/HARRIS_BRISK_0016.png)|
|HARRIS/BRIEF|8.898673|7.291746|[sample](./images/HARRIS_BRIEF_0016.png)|
|HARRIS/ORB|8.898673|6.603384|[sample](./images/HARRIS_ORB_0016.png)|
|HARRIS/FREAK|8.898673|6.457490|[sample](./images/HARRIS_FREAK_0016.png)|
|HARRIS/SIFT|8.898673|7.291746|[sample](./images/HARRIS_SIFT_0016.png)|
|FAST/BRISK|8.898673|11.487338|[sample](./images/FAST_BRISK_0016.png)|
|FAST/BRIEF|8.898673|11.218021|[sample](./images/FAST_BRIEF_0016.png)|
|FAST/ORB|8.898673|11.220639|[sample](./images/FAST_ORB_0016.png)|
|FAST/FREAK|8.898673|12.261320|[sample](./images/FAST_FREAK_0016.png)|
|FAST/SIFT|8.898673|11.214223|[sample](./images/FAST_SIFT_0016.png)|
|BRISK/BRISK|8.898673|10.804131|[sample](./images/BRISK_BRISK_0016.png)|
|BRISK/BRIEF|8.898673|9.468606|[sample](./images/BRISK_BRIEF_0016.png)|
|BRISK/ORB|8.898673|10.458415|[sample](./images/BRISK_ORB_0016.png)|
|BRISK/FREAK|8.898673|9.851232|[sample](./images/BRISK_FREAK_0016.png)|
|BRISK/SIFT|8.898673|10.956527|[sample](./images/BRISK_SIFT_0016.png)|
|ORB/BRISK|8.898673|9.760500|[sample](./images/ORB_BRISK_0016.png)|
|ORB/BRIEF|8.898673|12.709675|[sample](./images/ORB_BRIEF_0016.png)|
|ORB/ORB|8.898673|9.370819|[sample](./images/ORB_ORB_0016.png)|
|ORB/FREAK|8.898673|7.318464|[sample](./images/ORB_FREAK_0016.png)|
|ORB/SIFT|8.898673|8.345127|[sample](./images/ORB_SIFT_0016.png)|
|AKAZE/AKAZE|8.898673|9.101661|[sample](./images/AKAZE_AKAZE_0016.png)|
|SIFT/BRISK|8.898673|8.710034|[sample](./images/SIFT_BRISK_0016.png)|
|SIFT/BRIEF|8.898673|9.090199|[sample](./images/SIFT_BRIEF_0016.png)|
|SIFT/FREAK|8.898673|8.520908|[sample](./images/SIFT_FREAK_0016.png)|
|SIFT/SIFT|8.898673|8.944497|[sample](./images/SIFT_SIFT_0016.png)|

TODO: check values again...
