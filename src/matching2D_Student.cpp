
#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
         //convert to CV_32F
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

        if(descRef.type()!=CV_32F) {
            descRef.convertTo(descRef, CV_32F);
        }

        if(descSource.type()!=CV_32F) {
            descSource.convertTo(descSource, CV_32F);
        }
        cout << "FLANN matching" << endl;
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        int k = 2;
        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, k);
        
        // KNN matching using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for(auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        std::cout << knn_matches.size() - matches.size() << " keypoints removed" << std::endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::xfeatures2d::SIFT::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

std::vector<double> detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, int &numOfKpts, cv::Mat &img, std::vector<double> detectorTimes, bool bVis )
{
    
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;
    
    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cout << "working" << endl;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);
    
    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    numOfKpts += keypoints.size();
    detectorTimes.push_back(t);

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
    return detectorTimes;
}

std::vector<double> detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, int &numOfKpts, cv::Mat &img, std::vector<double> detectorTimes, bool bVis)
{
    int blockSize = 2; // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3; // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04; // Harris parameter (see equation for details)
    double maxOverlap = 0.0;

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1 );
    double t = (double)cv::getTickCount();
    cv::cornerHarris( img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT ); 
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dst_norm, dst_norm_scaled );

     if (bVis)
    {
        // Visualize the results
        string windowName = "Harris Corner Detector Response Matrix";
        cv::namedWindow(windowName);
        cv::imshow(windowName, dst_norm_scaled);
        cv::waitKey(0);
    }
    // Apply non-maximum suppression (NMS)
    for (size_t j = 0; j < dst_norm.rows; j++) {
        for (size_t i = 0; i < dst_norm.cols; i++) {
            int response = (int)dst_norm.at<float>(j, i);

            // Apply the minimum threshold for Harris cornerness response
            if (response < minResponse) continue;

            // Otherwise create a tentative new keypoint
            cv::KeyPoint newKeyPoint;
            newKeyPoint.pt = cv::Point2f(i, j);
            newKeyPoint.size = 2 * apertureSize;
            newKeyPoint.response = response;

            // Perform non-maximum suppression (NMS) in local neighbourhood around the new keypoint
            bool bOverlap = false;
            // Loop over all existing keypoints
            for (auto it = keypoints.begin(); it != keypoints.end(); ++it) {
                double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                // Test if overlap exceeds the maximum percentage allowable
                if (kptOverlap > maxOverlap) {
                    bOverlap = true;
                    // If overlapping, test if new response is the local maximum
                    if (newKeyPoint.response > (*it).response) {
                        *it = newKeyPoint;  // Replace the old keypoint
                        break;  // Exit for loop
                    }
                }
            }

            // If above response threshold and not overlapping any other keypoint
            if (!bOverlap) {
                keypoints.push_back(newKeyPoint);  // Add to keypoints list
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    numOfKpts += keypoints.size();
    detectorTimes.push_back(t);
    

    if (bVis)
    {
        // Visualize the keypoints
        string windowName = "Harris corner detection results";
        cv::namedWindow(windowName);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);
    }

    return detectorTimes;

}

std::vector<double> detKeypointsFast(std::vector<cv::KeyPoint> &keypoints, int &numOfKpts, cv::Mat &img, std::vector<double> detectorTimes, bool bVis)
{
    int threshold = 30;

    cv::Ptr<cv::FeatureDetector> fast = cv::FastFeatureDetector::create(threshold);

    double t = (double)cv::getTickCount();
    fast -> detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "FAST with n= " << keypoints.size() << " keypoints in " << 1000*t/1.0 << " ms" << std::endl;
    numOfKpts += keypoints.size();
    detectorTimes.push_back(t);
    return detectorTimes;
}

std::vector<double> detKeypointsBrisk(std::vector<cv::KeyPoint> &keypoints, int &numOfKpts, cv::Mat &img, std::vector<double> detectorTimes, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> brisk = cv::BRISK::create();

    double t = (double)cv::getTickCount();
    brisk -> detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "BRISK detector with n= " << keypoints.size() << " keypoints in " << 1000*t/1.0 << " ms" << std::endl;
    numOfKpts += keypoints.size();
    detectorTimes.push_back(t);
    return detectorTimes;
}


std::vector<double> detKeypointsOrb(std::vector<cv::KeyPoint> &keypoints, int &numOfKpts, cv::Mat &img, std::vector<double> detectorTimes, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create();

    double t = (double)cv::getTickCount();
    orb -> detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "ORB detector with n= " << keypoints.size() << " keypoints in " << 1000*t/1.0 << " ms" << std::endl;
    numOfKpts += keypoints.size();
    detectorTimes.push_back(t);
    return detectorTimes;

}

std::vector<double> detKeypointsAkaze(std::vector<cv::KeyPoint> &keypoints, int &numOfKpts, cv::Mat &img, std::vector<double> detectorTimes, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> akaze = cv::AKAZE::create();

    double t = (double)cv::getTickCount();
    akaze -> detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "AKAZE detector with n= " << keypoints.size() << " keypoints in " << 1000*t/1.0 << " ms" << std::endl;
    numOfKpts += keypoints.size();
    detectorTimes.push_back(t);
    return detectorTimes;
}
std::vector<double> detKeypointsSift(std::vector<cv::KeyPoint> &keypoints, int &numOfKpts, cv::Mat &img, std::vector<double> detectorTimes, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> sift = cv::xfeatures2d::SIFT::create();

    double t = (double)cv::getTickCount();
    sift -> detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "SIFT detector with n= " << keypoints.size() << " keypoints in " << 1000*t/1.0 << " ms" << std::endl;
    numOfKpts += keypoints.size();
    detectorTimes.push_back(t);
    return detectorTimes;
}