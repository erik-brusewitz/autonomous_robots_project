#include "behavior.hpp"

bool less_by_ver(cv::Point2f &lhs, cv::Point2f &rhs) {  
    return lhs.x < rhs.x;
}

cv::Point2f getAverage(std::vector<cv::Point2f> const& points, int start, int end) {
    if (points.empty() || start>=end || end<0 || start<0) {return cv::Point2f(0,0);}
    if (end>points.size()) {end = points.size();}
    return std::accumulate(points.begin()+start, points.begin()+end, cv::Point2f(0,0)) / (end-start);
}

Behavior::Behavior() :  blueConesPositions{}, 
						yellowConesPositions{}, 
						redConesPositions{}, 
						frontDistance{float((unsigned int)(-1))}, 
						rearDistance{float((unsigned int)(-1))}, 
						leftDistance{float((unsigned int)(-1))}, 
						rightDistance{float((unsigned int)(-1))}, 
						kiwiX{0}, 
						kiwiY{0}, 
						kiwiWidth{0}, 
						kiwiHeight{0},
						blueConesPositionsMutex{},
						yellowConesPositionsMutex{},
						redConesPositionsMutex{},
						frontDistanceMutex{},
						rearDistanceMutex{},
						leftDistanceMutex{},
						rightDistanceMutex{},
						kiwiXMutex{},
						kiwiYMutex{},
						kiwiWidthMutex{},
						kiwiHeightMutex{},
						maxPedalPosition{},
						maxSteeringAt{},
						nPoints{},
						steeringAngleStrategy{},
						pedalPositionStrategy{},
						movingAverageCoeff{1.0f},
						prevSteeringAngle{},
						prevPedalPosition{},
						maxFrontKiwiHeight{},
						maxFrontKiwiHeightBeforeReducingSpeed{},
						firstTime{true}, 
						stopAtJunction{false} {
}

void Behavior::setMovingAverageCoeff(float coeff) {
	movingAverageCoeff = coeff;
}

void Behavior::enableStopAtJunction() {stopAtJunction = true;}

void Behavior::useStrategy1ForSteering(float _maxSteeringAt, int _nPoints) {
	steeringAngleStrategy = 1;
	maxSteeringAt = _maxSteeringAt;
	nPoints = _nPoints;
}


void Behavior::useStrategy1ForPedal(float _maxPedalPosition, float _maxFrontKiwiHeight) {
	pedalPositionStrategy = 1;
	maxPedalPosition = _maxPedalPosition;
	maxFrontKiwiHeight = _maxFrontKiwiHeight;
}


void Behavior::useStrategy2ForPedal(float _maxPedalPosition, float _maxFrontKiwiHeight, float _maxFrontKiwiHeightBeforeReducingSpeed) {
	pedalPositionStrategy = 2;
	maxPedalPosition = _maxPedalPosition;
	maxFrontKiwiHeight = _maxFrontKiwiHeight;
	maxFrontKiwiHeightBeforeReducingSpeed = _maxFrontKiwiHeightBeforeReducingSpeed;
}
void Behavior::setKiwiPos(float _kiwiX, float _kiwiY, float _kiwiWidth, float _kiwiHeight) noexcept {
	kiwiX = 1-_kiwiY;
	kiwiY = 0.5-_kiwiX;
	kiwiWidth = _kiwiWidth;
	kiwiHeight = _kiwiHeight;
}

void Behavior::setBlueConePos(std::vector<cv::Point2f> positions) noexcept {
  	std::lock_guard<std::mutex> lock(blueConesPositionsMutex);
    blueConesPositions = changeCoordinates(positions);
    std::sort(blueConesPositions.begin(), blueConesPositions.end(), less_by_ver);
}

void Behavior::setYellowConePos(std::vector<cv::Point2f> positions) noexcept {
  	std::lock_guard<std::mutex> lock(yellowConesPositionsMutex);
    yellowConesPositions = changeCoordinates(positions);
    std::sort(yellowConesPositions.begin(), yellowConesPositions.end(), less_by_ver);
}

void Behavior::setRedConePos(std::vector<cv::Point2f> positions) noexcept {
  	std::lock_guard<std::mutex> lock(redConesPositionsMutex);
    redConesPositions = changeCoordinates(positions);
    std::sort(redConesPositions.begin(), redConesPositions.end(), less_by_ver);
}

void Behavior::setFrontDistance(float _frontDistance) noexcept {
  std::lock_guard<std::mutex> lock(frontDistanceMutex);
  frontDistance = _frontDistance;
}

void Behavior::setRearDistance(float _rearDistance) noexcept {
  std::lock_guard<std::mutex> lock(rearDistanceMutex);
  rearDistance = _rearDistance;
}

void Behavior::setLeftVoltage(float _leftVoltage) noexcept {
  std::lock_guard<std::mutex> lock(leftDistanceMutex);
  leftDistance = convertIrVoltageToDistance(_leftVoltage);
}

void Behavior::setRightVoltage(float _rightVoltage) noexcept {
  std::lock_guard<std::mutex> lock(rightDistanceMutex);
  rightDistance = convertIrVoltageToDistance(_rightVoltage);
}

std::tuple<float, float> Behavior::stepTime() {

	float groundSteering = 0;
	float pedalPosition = 0;

	if (!stopAtJunction || (stopAtJunction && !mustStopAtJunction())) {
		switch(pedalPositionStrategy) {
	  		case 1:
	  			pedalPosition = getPedalPositionStrategy1();
		    	break;
		    case 2:
		      	pedalPosition = getPedalPositionStrategy2();
		    	break;
	  		default:
	  			break;
		}

		switch(steeringAngleStrategy) {
	  		case 1:
	  			groundSteering = getSteeringAngleStrategy1();
		    	break;
	  		default:
	  			break;
		}

		if (!firstTime) {
			groundSteering = movingAverageCoeff * groundSteering + (1-movingAverageCoeff) * prevSteeringAngle;
			pedalPosition = movingAverageCoeff * pedalPosition + (1-movingAverageCoeff) * prevPedalPosition;
		}
		else {firstTime = false;}
	}

	prevSteeringAngle = groundSteering;
	prevPedalPosition = pedalPosition;
	
	return {groundSteering, pedalPosition};
}

// The received data is of the format x=[0,1], y=[0,1]. 
// It should be x=[0.5,-0.5], y=[1,0]. (0.5 to the far left, 1 in the bottom of the image).
cv::Point2f Behavior::changeCoordinate(const cv::Point2f &point) {
    return cv::Point2f(1 - point.y, 0.5 - point.x);
}

std::vector<cv::Point2f> Behavior::changeCoordinates(const std::vector<cv::Point2f> &points) {
    std::vector<cv::Point2f> newPoints;
    for (auto &point : points) {newPoints.push_back(changeCoordinate(point));}
    return newPoints;
}

float Behavior::convertIrVoltageToDistance(float voltage) const noexcept
{
  double voltageDividerR1 = 1000.0;
  double voltageDividerR2 = 1000.0;

  double sensorVoltage = (voltageDividerR1 + voltageDividerR2) / voltageDividerR2 * voltage;
  double distance = (2.5 - sensorVoltage) / 0.07;
  return distance;
}

float Behavior::getSteeringAngleStrategy1() {

  	std::lock_guard<std::mutex> lock1(blueConesPositionsMutex);
  	std::lock_guard<std::mutex> lock2(yellowConesPositionsMutex);

    float steeringAngle;
    if (blueConesPositions.size()==0 && yellowConesPositions.size()==0) {steeringAngle = 0;}
    else if (blueConesPositions.size()==0) {steeringAngle =-MAX_STEERING;}
    else if (yellowConesPositions.size()==0) {steeringAngle =MAX_STEERING;}
    else {
		cv::Point2f blueConesAverage = getAverage(blueConesPositions, 0, nPoints);
		cv::Point2f yellowConesAverage = getAverage(yellowConesPositions, 0, nPoints);
		cv::Point2f aimPoint = (blueConesAverage+yellowConesAverage)/2;
		double fpAngle = atan2(aimPoint.y, aimPoint.x) * (MAX_STEERING/maxSteeringAt);
		steeringAngle = static_cast<float>(fpAngle);
	}
	if (fabs(steeringAngle)<0.05) {steeringAngle = 0;}
	return steeringAngle;
}

float Behavior::getPedalPositionStrategy1() {
	
	return (kiwiHeight>maxFrontKiwiHeight) ? 0 : maxPedalPosition;
}

float Behavior::getPedalPositionStrategy2() {
	
	if (kiwiHeight<maxFrontKiwiHeightBeforeReducingSpeed) {return maxPedalPosition;}
	if (kiwiHeight>maxFrontKiwiHeight) {return 0;}
	return maxPedalPosition*(1-(kiwiHeight-maxFrontKiwiHeightBeforeReducingSpeed)/(maxFrontKiwiHeight-maxFrontKiwiHeightBeforeReducingSpeed));
}

bool Behavior::mustStopAtJunction() {

	if (redConesPositions.size()==0) {return false;}
	if (kiwiHeight==0 || kiwiWidth==0) {return false;}
	float rightRedConesAverageYPosition = 0;
	int counter = 0;
	for (auto redConesPosition : redConesPositions) {
		if (redConesPosition.y<0) {
			rightRedConesAverageYPosition+=redConesPosition.y;
			counter ++;
		}
	}
	if (counter==0) {return false;}
	rightRedConesAverageYPosition = rightRedConesAverageYPosition/counter;
	if (std::abs((kiwiY-kiwiWidth/2)-rightRedConesAverageYPosition)<0.6*kiwiWidth) {return true;}
	return false;
}
