
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{   
    vector<double> distnaces;
    double avgDistance = 0.00;
    int numOfMatches = 0; 
    
    //if the ROI contains our kepypoint, add the euclidean distance to avg distance
    for (auto &match : kptMatches) {
        auto ptCurr = kptsCurr[match.trainIdx].pt;
        auto ptPrev = kptsPrev[match.queryIdx].pt;
        auto dist = cv::norm(ptPrev - ptCurr);

        if (boundingBox.roi.contains(ptCurr)) {
            distnaces.push_back(dist);
            avgDistance += dist;
            numOfMatches ++;
        }
    }

    if (numOfMatches == 0)
    {
        return;
    }

    avgDistance /= numOfMatches;
    std::sort(distnaces.begin(), distnaces.end());
    auto medianDistnace = distnaces[distnaces.size() /2];
    auto count = 0;
    
    for(auto match : kptMatches)
    {
        
        auto ptCurr = kptsCurr[match.trainIdx].pt;
        auto ptPrev = kptsPrev[match.queryIdx].pt;
        auto euclideanDistance = cv::norm(ptPrev - ptCurr); 
         
        //cout << medianDistnace << endl;
        cout << "Euclidian Distance: " << euclideanDistance << "    Med Distance: " << medianDistnace << endl;
        if(boundingBox.roi.contains(ptCurr) && euclideanDistance <  30 * medianDistnace) 
        {
            boundingBox.kptMatches.push_back(match);
            count++;
        }
        
    }
    cout << kptMatches.size()  << " " << count << endl;


}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }


    // STUDENT TASK (replacement for meanDistRatio)
    std::sort(distRatios.begin(), distRatios.end());
    double medDistRatio = distRatios[distRatios.size() / 2];

    TTC = (-1.0 / frameRate) / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // TTC = d1 * dt / (d0 - d1)
    // d0 = distance (prev frame)
    // d1 = distance (curr frame)
    // dt = time delta - 1/framerate

    //take median x value of prev and current frame to get d0 and d1

    std::vector<double> lidarPointsPrevFiltered;
    std::vector<double> lidarPointsCurrFiltered;

    for(auto point : lidarPointsPrev)
    {
        lidarPointsPrevFiltered.push_back(point.x);
    }

     for(auto point : lidarPointsCurr)
    {
        lidarPointsCurrFiltered.push_back(point.x);
    }

    //sort points
    std::sort(lidarPointsPrevFiltered.begin(), lidarPointsPrevFiltered.end());
    std::sort(lidarPointsPrevFiltered.begin(), lidarPointsPrevFiltered.end());

    //find median
    double medianPrev = lidarPointsPrevFiltered[(lidarPointsPrevFiltered.size() / 2)];
    double medianCurr = lidarPointsCurrFiltered[(lidarPointsCurrFiltered.size() / 2)];

    int removedPrev = 0;
    int removedCurr = 0;


    //remove any points that are not withing 3% of median
    std::vector<double> prev;
    double sumPrev = 0;
    for(auto point : lidarPointsPrevFiltered)
    {
        if(point > 0.97*medianPrev && point <  1.03*medianPrev)
        {
           prev.push_back(point);
           sumPrev += point; 
        }
        else{ removedPrev++;}
        
    }
    //for d0 we will use the closest of the filtered points
    double d0 = sumPrev / prev.size();

    std::vector<double> curr;
    double sumCurr = 0;
    for(auto point : lidarPointsCurrFiltered)
    {
       if(point > 0.97*medianCurr && point < 1.03*medianCurr)
        {
           curr.push_back(point); 
           sumCurr += point;
        }
        else{removedCurr++;}
    }
    double d1 = sumCurr / curr.size();

    cout << d0 << ", " << d1 << endl;
    //calculate TTC
    TTC = d1 * (1/ frameRate) / (d0 - d1);

    cout << removedPrev << ", " << removedCurr << endl;

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::multimap<int,int> _map {};

    int maxPrevBoxId = 0;

    for (auto match : matches)
    {
        cv::KeyPoint prevKpt = prevFrame.keypoints[match.queryIdx];
        cv::KeyPoint currKpt = currFrame.keypoints[match.trainIdx];

        int prevBoxId;
        int currBoxId;
        
        for(auto boundingBox : prevFrame.boundingBoxes)
        {
            if(boundingBox.roi.contains(prevKpt.pt))
            {
                prevBoxId = boundingBox.boxID;
            }
        }

        for(auto boundingBox : currFrame.boundingBoxes)
        {
            if(boundingBox.roi.contains(currKpt.pt))
            {
                currBoxId = boundingBox.boxID;
            }
        }

        _map.insert({currBoxId,prevBoxId});

        maxPrevBoxId = std::max(maxPrevBoxId, prevBoxId);

    }

    vector<int> currFrameBoxIds {};

    for (auto box : currFrame.boundingBoxes)
    {
        currFrameBoxIds.push_back(box.boxID);
    }

    for(int i : currFrameBoxIds)
    {
        auto range = _map.equal_range(i);

        std::vector<int> counts(maxPrevBoxId + 1, 0);
    

    for (auto it = range.first; it != range.second; ++it) 
    {
        if (-1 != (*it).second) 
        {
            counts[(*it).second] += 1;
        }
    }

    int modeIndex = std::distance(counts.begin(), std::max_element(counts.begin(), counts.end()));
    bbBestMatches.insert({modeIndex, i});
    }
}
