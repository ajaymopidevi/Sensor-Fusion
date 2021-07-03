#ifndef dataStructures_h
#define dataStructures_h

//#include <iostream>
#include <vector>
#include <opencv2/core.hpp>

//using namespace std;
struct DataFrame { // represents the available sensor information at the same time instance
    
    cv::Mat cameraImg; // camera image
    
    std::vector<cv::KeyPoint> keypoints; // 2D keypoints within camera image
    cv::Mat descriptors; // keypoint descriptors
    std::vector<cv::DMatch> kptMatches; // keypoint matches between previous and current frame
};

struct Node{
    DataFrame dataFrame;
    struct Node* next;
    int num;
};

void deleteNode(Node **head);
void insert(Node **head, DataFrame &data, int bufferSize);

void display(Node **head);

#endif /* dataStructures_h */
