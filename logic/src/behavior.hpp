#ifndef __behavior__
#define __behavior__

#include <chrono>
#include <vector>
#include <tuple>
#include <math.h>
#include <mutex>
#include <iostream>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <numeric>


const float MAX_STEERING = 0.290888f;

bool less_by_ver(cv::Point2f &lhs, cv::Point2f &rhs);
cv::Point2f getAverage(std::vector<cv::Point2f> const& points, int start, int end);

class Behavior {

private:
  Behavior(Behavior const &) = delete;
  Behavior(Behavior &&) = delete;
  Behavior &operator=(Behavior const &) = delete;
  Behavior &operator=(Behavior &&) = delete;

public:
	Behavior();
  ~Behavior() = default;

  void enableStopAtJunction();
  void setMovingAverageCoeff(float coeff);

  void useStrategy1ForSteering(float _maxSteeringAt, int _nPoints);
  void useStrategy1ForPedal(float _maxPedalPosition, float _maxFrontKiwiHeight);
	void useStrategy2ForPedal(float _maxPedalPosition, float _maxFrontKiwiHeight, float _maxFrontKiwiHeightBeforeReducingSpeed);


	void setKiwiPos(float _kiwiX, float _kiwiY, float _kiwiWidth, float _kiwiHeight) noexcept;
	void setBlueConePos(std::vector<cv::Point2f> positions) noexcept;
	void setYellowConePos(std::vector<cv::Point2f> positions) noexcept;
	void setRedConePos(std::vector<cv::Point2f> positions) noexcept;
	void setFrontDistance(float _frontDistance) noexcept;
	void setRearDistance(float _rearDistance) noexcept;
	void setLeftVoltage(float _leftVoltage) noexcept;
	void setRightVoltage(float _rightVoltage) noexcept;

	std::tuple<float, float> stepTime();

private:
	bool firstTime, stopAtJunction;
	int pedalPositionStrategy, steeringAngleStrategy, nPoints;

	std::vector<cv::Point2f> blueConesPositions, 
					   							 yellowConesPositions, 
					   						 	 redConesPositions;

  float maxPedalPosition, 
  			maxSteeringAt,
    		frontDistance, 
    	  rearDistance, 
    	  leftDistance, 
    	  rightDistance, 
    	  kiwiX, 
    	  kiwiY, 
    	  kiwiWidth, 
    	  kiwiHeight,
    	  movingAverageCoeff,
    	  prevSteeringAngle,
    	  prevPedalPosition,
    	  maxFrontKiwiHeight,
    	  maxFrontKiwiHeightBeforeReducingSpeed;

  std::mutex blueConesPositionsMutex,
				yellowConesPositionsMutex,
				redConesPositionsMutex,
				frontDistanceMutex,
				rearDistanceMutex,
				leftDistanceMutex,
				rightDistanceMutex,
				kiwiXMutex,
				kiwiYMutex,
				kiwiWidthMutex,
				kiwiHeightMutex;

	cv::Point2f changeCoordinate(const cv::Point2f &point);
	std::vector<cv::Point2f> changeCoordinates(const std::vector<cv::Point2f> &points);
	float convertIrVoltageToDistance(float voltage) const noexcept;
	
	float getSteeringAngleStrategy1();
	
	float getPedalPositionStrategy1();
	float getPedalPositionStrategy2();

	bool mustStopAtJunction();


    // std::vector<float> nLatestCloseAngles, nLatestFarAngles;
    // float latestSteeringAngle, averageCloseAngle, averageFarAngle;
};


#endif