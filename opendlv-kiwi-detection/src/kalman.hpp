#ifndef __KALMAN__
#define __KALMAN__

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


const int KALMAN_FILTER_STATE_SIZE = 6;
const int KALMAN_FILTER_MEASUREMENT_SIZE = 4;
const int KALMAN_FILTER_CONTROL_PARAMETER = 0;
const int MAX_PREDICTIONS_WITH_NO_MEASUREMENT = 100;


class KalmanFilter {

public:
	KalmanFilter();
	cv::Rect stepTime(float dt, const cv::Rect box=cv::Rect(0,0,0,0));

private:
	cv::KalmanFilter* kalmanFilter;
    cv::Mat predict (float dt);
    cv::Mat getMeasurementMat(const cv::Rect &box);
    cv::Mat getStateMat(const cv::Rect &box);
    cv::Rect getRectFromState(const cv::Mat &state);


	bool prevUpdate;
    const unsigned int maxConsequitiveUpdatesWithNoMeasurements = MAX_PREDICTIONS_WITH_NO_MEASUREMENT;
    bool kiwiDetected;
    unsigned int consequitiveUpdatesWithNoMeasurements;
};

#endif