# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1
 * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
 * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Rubrics

This section contains an explanation about what have done in this project.

### 1. MP.1 - Data Buffer Optimization

This is accomplished by directly assessing the buffer size after inserting new data. If it exceeds the defined maximum size, the first element in the buffer is removed using the `erase()` method in the vector library.

```C++
// ...
if (dataBuffer.size() > dataBufferSize) {
    // remove the first element 
    dataBuffer.erase(dataBuffer.begin());
}
// ...
```

### 2. MP.2 - Keypoint Detection

Utilizing detectors such as HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT from the OpenCV library, these are made selectable by setting a corresponding string. The code also measures the execution time of the detection algorithm and displays, on the console, the number of keypoints detected.

```C++
detectorType = "SHITOMASI"; // -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT, SHITOMASI
descriptorType = "BRISK"; // -> BRIEF, ORB, FREAK, AKAZE, SIFT, BRISK

// ... Some code here

if (detectorType.compare("SHITOMASI") == 0) {
            // SHITOMASI
            detKeypointsShiTomasi(keypoints, imgGray, false);
        } else if (detectorType.compare("HARRIS") == 0) {
            // HARRIS
            detKeypointsHarris(keypoints, imgGray, false);
        } else {
            // FAST, BRISK, ORB, AKAZE, SIFT
            detKeypointsModern(keypoints, imgGray, detectorType, false);
        }
// ...
```

### 3. MP.3 - Keypoint Removal

Keypoints outside of a pre-defined rectangle are removed, and only those within the rectangle are used for further processing. This is achieved through the use of "cv::Rect" and the "contains()" method, which checks points inside the defined rectangle.

```C++
// only keep keypoints on the preceding vehicle
bool bFocusOnVehicle = true;
cv::Rect vehicleRect(535, 180, 180, 150);
if (bFocusOnVehicle) {

    // remove the keypoints that are not in the vehicleRect
    vector<cv::KeyPoint> filtered_keypoints;

    for(int i = 0; i < keypoints.size(); i++) {
        cv::Point2f pt = static_cast<cv::Point2i>(keypoints[i].pt);

        if (vehicleRect.contains(pt)) {
            filtered_keypoints.push_back(keypoints[i]);
        }
    }
    // replace the keypoints with the filtered_keypoints
    keypoints.clear();
    keypoints = filtered_keypoints;
}
```

### 4. MP.4 - Keypoint Descriptors

Descriptors such as BRIEF, ORB, FREAK, AKAZE, and SIFT from OpenCV are implemented and selectable by setting a string accordingly. The code also includes execution time measurement.

```C++
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0) {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    } else if (descriptorType.compare("BRIEF") == 0) {
        // BRIEF is a binary descriptor
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    } else if (descriptorType.compare("ORB") == 0) {
        // ORB is a binary descriptor
        extractor = cv::ORB::create();
    } else if (descriptorType.compare("FREAK") == 0) {
        // FREAK is a binary descriptor
        extractor = cv::xfeatures2d::FREAK::create();
    } else if (descriptorType.compare("AKAZE") == 0) {
        // AKAZE is a binary descriptor
        extractor = cv::AKAZE::create();
    } else if (descriptorType.compare("SIFT") == 0) {
        // SIFT is a floating point descriptor
        extractor = cv::xfeatures2d::SIFT::create();
    } else {
        // invalid descriptor type
        throw std::invalid_argument("Invalid descriptor type: " + descriptorType);
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();

    descriptorTime = 1000 * t / 1.0;
    cout << descriptorType << " descriptor extraction in " << descriptorTime << " ms" << endl;
}
```

### 5. MP.5 - Descriptor Matching

FLANN matching and k-nearest neighbor selection are implemented and selectable using respective strings in the main function. Descriptor types of the current and previous images are overwritten to "CV_32F." Brute Force (BF) Matching includes code to select the norm type between "cv::NORM_HAMMING" and "cv::NORM_L2" based on whether the feature type is binary or not.

```C++
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0) {
        int normType = cv::NORM_HAMMING;
        if (descriptorType.compare("DES_BINARY") == 0) {
            normType = cv::NORM_HAMMING;
        } else if (descriptorType.compare("DES_HOG") == 0) {
            normType = cv::NORM_L2;
        } else {
            throw std::invalid_argument("Invalid descriptor type: " + descriptorType);
        }
        matcher = cv::BFMatcher::create(normType, crossCheck);
        cout << "BF matching";

    } else if (matcherType.compare("MAT_FLANN") == 0) {
        // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
        if (descSource.type() != CV_32F) {
            descSource.convertTo(descSource, CV_32F);
        } 
        
        if (descRef.type() != CV_32F) {
            descRef.convertTo(descRef, CV_32F);
        }

        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        cout << "FLANN matching";
    } else {
        throw std::invalid_argument("Invalid matcher type: " + matcherType);
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0) { 
        // nearest neighbor (best match)
        matcher->match(descSource, descRef, matches); 
    } else if (selectorType.compare("SEL_KNN") == 0){ 
        // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;

        // Finds the best match for each descriptor in desc1
        matcher->knnMatch(descSource, descRef, knn_matches, 2);

        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {
            // if the best match has a distance ratio less than minDescDistRatio, then keep it
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance) {
                matches.push_back((*it)[0]);
            }
        }

        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
    }
}
```

### 6. MP.6 - Descriptor Distance Ratio

The K-Nearest-Neighbor matching is used to implement the descriptor distance ratio test, with a ratio of 0.8, deciding whether to keep an associated pair of keypoints.

```C++
// perform matching task
if (selectorType.compare("SEL_NN") == 0) { 
    // nearest neighbor (best match)
    matcher->match(descSource, descRef, matches); 
} else if (selectorType.compare("SEL_KNN") == 0){ 
    // k nearest neighbors (k=2)
    vector<vector<cv::DMatch>> knn_matches;

    // Finds the best match for each descriptor in desc1
    matcher->knnMatch(descSource, descRef, knn_matches, 2);

    // filter matches using descriptor distance ratio test
    double minDescDistRatio = 0.8;
    for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it) {
        // if the best match has a distance ratio less than minDescDistRatio, then keep it
        if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance) {
            matches.push_back((*it)[0]);
        }
    }

    cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
}
```

### 7. MP.7 - Performance Evaluation 1

Count the number of keypoints on the preceding vehicle for all 10 images and note their neighborhood size distribution for all implemented detectors. CSV files in the doc folder contain this information for all detector combinations.

**[Check the generated csv files for all combination in the doc folder]**

### 8. MP.8 - Performance Evaluation 2

Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. The BF approach is used in the matching step with a descriptor distance ratio set to 0.8. CSV files in the doc folder contain this information for all combinations.

**[Check the generated csv files for all combination in the doc folder]**

### 9. MP.9 - Performance Evaluation 3

Log the time for keypoint detection and descriptor extraction. The results should be entered into a spreadsheet. Based on this data, the top 3 detector/descriptor combinations are recommended for detecting keypoints on vehicles. The AKAZE descriptor works only in combination with the AKAZE detector, and the same applies to SIFT.

**[Check the generated csv files for all combination in the doc folder]**

## Top 3 detector/descriptor combinations

### FAST (Detector) + BRISK (Descriptor)

Detection execution time (Average): 11.36345 ms
Keypoints detected (Average): 1344 points
Description execution time (Average): 1.5916 ms
Good Matched Keypoints (Average): 111 points

### FAST (Detector) + BRIEF (Descriptor)

Detection execution time (Average): 11.62124 ms
Keypoints detected (Average): 1344 points
Description execution time (Average): 1.6232 ms
Good Matched Keypoints (Average): 111 points

### FAST (Detector) + ORB (Descriptor)

Detection execution time (Average): 12.0863 ms
Keypoints detected (Average): 1344 points
Description execution time (Average): 1.64849 ms
Good Matched Keypoints (Average): 111 points
