
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
                //it2->lidarPoints.push_back(*it1);
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
    double mean_distance=0.0;
    
    double n =0;
    vector<int>matchesInBox;
    for(int i=0; i< kptMatches.size(); i++)
    {
        cv::DMatch match = kptMatches[i];
        if (boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            mean_distance = mean_distance +cv::norm(kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt);
            n = n+1;
            matchesInBox.push_back(i);
        }
    }
    mean_distance = mean_distance / n;

    for(auto Idx: matchesInBox)
    {
        cv::DMatch match = kptMatches[Idx];
        double dist = cv::norm(kptsCurr[match.trainIdx].pt - kptsPrev[match.queryIdx].pt);
        if( dist <= 1.5* mean_distance )
        {
            boundingBox.keypoints.push_back(kptsCurr[match.trainIdx]);
            boundingBox.kptMatches.push_back(match);
        }
    }

    
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    double dT = 1.0/ frameRate;
    vector<double> distances;

    for(auto it1 = kptMatches.begin(); it1 != kptMatches.end() -1 ; ++it1)
    {
        cv::KeyPoint kptOuterPrev = kptsPrev.at(it1->queryIdx);
        cv::KeyPoint kptOuterCurr = kptsCurr.at(it1->trainIdx);

        double minDist = 100.0;

        for(auto it2 = kptMatches.begin() +1 ; it2 != kptMatches.end(); ++it2)
        {
            cv::KeyPoint kptInnerPrev = kptsPrev.at(it2->queryIdx);
            cv::KeyPoint kptInnerCurr = kptsCurr.at(it2->trainIdx);

            double prevDist = cv::norm(kptOuterPrev.pt - kptInnerPrev.pt);
            double currDist = cv::norm(kptOuterCurr.pt - kptInnerCurr.pt);

            if(prevDist > std::numeric_limits<double>::epsilon() && currDist > minDist)
            {// to ensure that the denominator is greater than 0
                double distRatio = currDist / prevDist;
                distances.push_back(distRatio);
            }
            
        }
    }

    int d = distances.size();
    if(d==0)
    {
        TTC = NAN;
        return;
    }
    double medianDist;
    auto mid = distances.begin() + d/2;
    nth_element(distances.begin(), mid, distances.end());
    if(d%2)
    {
        medianDist = distances[(int)d/2];
    }
    else
    {
        double temp1 = distances[(int)d/2];
        nth_element(distances.begin(), mid-1, distances.end());
        double temp2 = distances[((int)d/2)-1];
        medianDist = (temp1 + temp2) / 2.0;
    }
    cout<<"Keypoint matches :"<<kptMatches.size()<<endl;
    TTC = -dT / (1 - medianDist);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    //cout<<"Check1 ";
    
    double dT = 1 / frameRate;
    double minXPrev = 1e9, minXCurr = 1e9;
    double avgXPrev =0.0, avgXCurr =0.0;
    
    for(auto pt: lidarPointsPrev)
    {
        avgXPrev = avgXPrev + pt.x;
    }
    
    avgXPrev = avgXPrev / lidarPointsPrev.size();
    //cout<<"Check2 ";
    for(auto pt:lidarPointsCurr)
    {
        avgXCurr = avgXCurr + pt.x;
    }
    
    avgXCurr = avgXCurr / lidarPointsCurr.size();
    //cout<<"check3 ";
    double thresholdPrev = 0.5 * avgXPrev;
    int count1=0, count2=0;
    for (auto pt : lidarPointsPrev)
    {
        if (pt.x > thresholdPrev)
        {
            //count1++;
            minXPrev = pt.x < minXPrev ? pt.x : minXPrev;
        }
    }
    //cout<<"check4 ";

    double thresholdCurr = 0.5 * avgXCurr;
    for(auto pt : lidarPointsCurr)
    {
        if(pt.x > thresholdCurr)
        {
            //count2++;
            minXCurr = pt.x < minXCurr ? pt.x : minXCurr;
        }
    }
    
    //cout<<"check5 ";
    cout<<"minXPrev: "<<minXPrev<<" minXCurr: "<<minXCurr<<endl;
    
    TTC = dT * minXCurr / (minXPrev - minXCurr);

    
}

int findBoundingBox(DataFrame &frame, int kptIdx)
{
    int matchedBoxes = 0;
    int boxID=-1;
    for (int i=0; i < frame.boundingBoxes.size(); i++)
    {
        
        // check wether point is within current bounding box
        if ((frame.boundingBoxes[i].roi).contains(frame.keypoints[kptIdx].pt))
        {
            matchedBoxes++;
            boxID = i;
        }

        if(matchedBoxes>1)
        {
            return -1;
        }

    } 
    return boxID;
}

void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    
    int rows = prevFrame.boundingBoxes.size();
    int cols = currFrame.boundingBoxes.size();
    vector<vector<int>> bbMatches(rows, vector<int>(cols,0));
    //int bbMatches[rows][cols];
    //memset(bbMatches, 0, sizeof(bbMatches[0][0]*rows*cols));
    
    for(int i=0;i<matches.size();i++){
        int queryIdx = matches[i].queryIdx;
        int trainIdx = matches[i].trainIdx;
        int queryBoxIdx = findBoundingBox(prevFrame, queryIdx);
        int trainBoxIdx = findBoundingBox(currFrame, trainIdx);

        if(queryBoxIdx != -1 && trainBoxIdx != -1)
        {
            bbMatches[queryBoxIdx][trainBoxIdx]++;
        }
    }

    for(int i=0; i<rows; i++)
    {
        int maxID=0;
        int maxValue = bbMatches[i][0];
        for(int j=1;j<cols;j++)
        {
            if(bbMatches[i][j]> maxValue)
            {
                maxValue = bbMatches[i][j];
                maxID = j;
            }
        }
        if(maxValue >0)
        {
            bbBestMatches[prevFrame.boundingBoxes[i].boxID] = currFrame.boundingBoxes[maxID].boxID;
            
        }
    }
    
    

}
