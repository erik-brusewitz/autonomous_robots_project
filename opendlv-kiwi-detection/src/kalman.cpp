#include "kalman.hpp"


// (x’(k)): x(k)=A*x(k-1)+B*u(k)
// B: controlMatrix
// A: state transition matrix





KalmanFilter::KalmanFilter() : kiwiDetected(false), consequitiveUpdatesWithNoMeasurements(0) {

    kalmanFilter  = new cv::KalmanFilter(KALMAN_FILTER_STATE_SIZE, KALMAN_FILTER_MEASUREMENT_SIZE, KALMAN_FILTER_CONTROL_PARAMETER, CV_32F);

    cv::Mat state(KALMAN_FILTER_STATE_SIZE, 1, CV_32F);
    cv::Mat measurement(KALMAN_FILTER_MEASUREMENT_SIZE, 1, CV_32F);
    cv::setIdentity(kalmanFilter->transitionMatrix);

    kalmanFilter->measurementMatrix = (cv::Mat_<float>(KALMAN_FILTER_MEASUREMENT_SIZE, KALMAN_FILTER_STATE_SIZE, CV_32F) << 1.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                                                                                                                         0.00, 1.00, 0.00, 0.00, 0.00, 0.00,
                                                                                                                         0.00, 0.00, 0.00, 0.00, 1.00, 0.00,
                                                                                                                         0.00, 0.00, 0.00, 0.00, 0.00, 1.00);

    kalmanFilter->processNoiseCov = (cv::Mat_<float>(KALMAN_FILTER_STATE_SIZE, KALMAN_FILTER_STATE_SIZE, CV_32F) << 0.01, 0.00, 0.00, 0.00, 0.00, 0.00,
                                                                                                                 0.00, 0.01, 0.00, 0.00, 0.00, 0.00,
                                                                                                                 0.00, 0.00, 5.00, 0.00, 0.00, 0.00,
                                                                                                                 0.00, 0.00, 0.00, 5.00, 0.00, 0.00,
                                                                                                                 0.00, 0.00, 0.00, 0.00, 0.01, 0.00,
                                                                                                                 0.00, 0.00, 0.00, 0.00, 0.00, 0.01);

    cv::setIdentity(kalmanFilter->measurementNoiseCov, cv::Scalar(0.1));
    // std::cout<<kalmanFilter->errorCovPre<<"\n\n\n\n";
}


cv::Rect KalmanFilter::stepTime(float dt, const cv::Rect box) {

    cv::Mat bestAproximation = predict(dt);
        // std::cout<<bestAproximation<<'\n';


	if (!box.x && !box.y && !box.width && !box.height) {

        consequitiveUpdatesWithNoMeasurements++;
        kiwiDetected = (consequitiveUpdatesWithNoMeasurements < maxConsequitiveUpdatesWithNoMeasurements);		
	}
	else {
		consequitiveUpdatesWithNoMeasurements = 0;

		if (!kiwiDetected) {

			kalmanFilter->errorCovPre.at<float>(0) = 1;
            kalmanFilter->errorCovPre.at<float>(7) = 1;
            kalmanFilter->errorCovPre.at<float>(14) = 1;
            kalmanFilter->errorCovPre.at<float>(21) = 1;
            kalmanFilter->errorCovPre.at<float>(28) = 1;
            kalmanFilter->errorCovPre.at<float>(35) = 1;

            // std::cout<<"this is the box:  "<<box.x<<'\t'<<box.y<<'\t'<<box.width<<'\t'<<box.height<<'\t'<<"\n\n";
            kalmanFilter->statePost = getStateMat(box);//cv::Mat_<float>(KALMAN_FILTER_STATE_SIZE, 1, CV_32F) << measurement.at(0), measurement.at(1), 0, 0, measurement.at(2), measurement.at(3)
            // std::cout<<"this is init:  "<<kalmanFilter->statePost<<"\n\n";
            kiwiDetected = true;
		}
		else {
			kalmanFilter->correct(getMeasurementMat(box));
		}
	}
    
    bestAproximation = kalmanFilter->statePost;

    return getRectFromState(bestAproximation);
}


cv::Mat KalmanFilter::predict(float dt) {

    cv::Mat prediction = getStateMat(cv::Rect(0, 0, 0, 0));

    if (kiwiDetected) {

        kalmanFilter->transitionMatrix.at<float>(2) = dt;
        kalmanFilter->transitionMatrix.at<float>(9) = dt;
        prediction = kalmanFilter->predict();
    }

    return prediction;         
}


cv::Mat KalmanFilter::getMeasurementMat(const cv::Rect &box) {
    
    return cv::Mat_<float>(KALMAN_FILTER_MEASUREMENT_SIZE, 1, CV_32F) << (float)box.x+(float)box.width/2, 
                                                                         (float)box.y+(float)box.height/2, 
                                                                         (float)box.width, 
                                                                         (float)box.height;
}


cv::Mat KalmanFilter::getStateMat(const cv::Rect &box) {
    
    return cv::Mat_<float>(KALMAN_FILTER_STATE_SIZE, 1, CV_32F) << (float)box.x+(float)box.width/2, 
                                                                   (float)box.y+(float)box.height/2, 
                                                                   0,
                                                                   0,
                                                                   (float)box.width, 
                                                                   (float)box.height;
}


cv::Rect KalmanFilter::getRectFromState(const cv::Mat &state) {
    return cv::Rect(state.at<float>(0)-state.at<float>(4)/2, 
                    state.at<float>(1)-state.at<float>(5)/2, 
                    state.at<float>(4), 
                    state.at<float>(5));
}












