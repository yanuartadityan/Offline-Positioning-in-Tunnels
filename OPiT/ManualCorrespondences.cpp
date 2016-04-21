#include "FeatureDetection.cpp"

#define EPSILON 0.00001

int areSame(int index, double x, double y, double xx, double yy)
{
    if ((fabs(x - xx) < EPSILON) && (fabs(y - yy) < EPSILON))
    {
        return true;
    }
    else
        return false;
}

void ManualCorrespondences(char *imagename)
{
    FeatureDetection fdet;

    Mat descriptor;
    vector<KeyPoint> kpts;
    vector<KeyPoint> pxIdx;
    vector<Point2f> init2D;

    //load image
    Mat img = imread(imagename);

    //detect keypoints
    fdet.siftDetector(img, kpts);

    //find the 10 correspondences from whole kpts
    init2D.push_back(Point2f(397.210571,145.146866));
    init2D.push_back(Point2f(650.494934,129.172379));
    init2D.push_back(Point2f(519.567688,131.898239));
    init2D.push_back(Point2f(531.834473,267.480103));
    init2D.push_back(Point2f(239.835358,207.141220));
    init2D.push_back(Point2f(834.740051,174.580566));
    init2D.push_back(Point2f(211.190155,510.402740));
    init2D.push_back(Point2f(437.319458,218.244186));
    init2D.push_back(Point2f(845.259948,160.413910));
    init2D.push_back(Point2f(559.729248,170.678528));

    for (int i=0; i<kpts.size(); i++)
    {
        for (int j=0; j<init2D.size(); j++)
        {
            if (areSame(i, init2D[j].x, init2D[j].y, kpts[i].pt.x, kpts[i].pt.y))
            {
                cout << i << endl;
                pxIdx.push_back(kpts[i]);
                init2D.erase(init2D.begin()+j);
            }
        }
    }

    // extract descriptor
    fdet.siftExtraction(img, pxIdx, descriptor);
    
    for (int i = 0; i < pxIdx.size(); i++)
    {
        cout << pxIdx[i].pt << endl;
    }
    
    // save the descriptor to a file
//    cv::FileStorage storage("ManualCorrespondences.yml", cv::FileStorage::WRITE);
//    storage << "img" << descriptor;
//    storage.release();
//    
//    cout << "done" << endl;
}
