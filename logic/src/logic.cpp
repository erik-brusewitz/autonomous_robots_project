#include <chrono>
#include <vector>
#include <math.h>
#include <string>


#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "behavior.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


const int BLUE_CONES_ID = 0;
const int YELLOW_CONES_ID = 1;
const int RED_CONES_ID = 2;

const float MOVING_AVERAGE_COEFF = 1.0f;

const float MAX_ALLOWED_FRONT_KIWI_HEIGHT = 0.6;     // Relative to the height of the image. If the height of the detected Kiwi goes up this value the car will stop.
const float MAX_ALLOWED_FRONT_KIWI_HEIGHT_BEFORE_REDUCING_SPEED = 0.4;   

const std::string TUNNING_WINDOW_NAME = "Tune";

const int DEFAULT_STEERING_STRATEGY = 1;
const int DEFAULT_PEDAL_POSITION_STRATEGY = 1;

// bool less_by_ver(Point &lhs, Point &rhs){
    
//     return lhs.ver < rhs.ver;
// }

/*bool compareDouble(double lhs, double rhs){
    if (fabs(lhs-rhs) < 0.01)
        return true;
    return false;
}*/

//The received data is of the format hor=[0,1], ver=[0,1]. It should be hor=[0.5,-0.5], ver=[1,0]. (0.5 to the far left, 1 in the bottom of the image).
// std::vector<Point> changeCoordinates(const std::vector<Point> &points){
//     std::vector<Point> tmp;
//     for (auto &p : points){
//         auto newP = Point(1 - p.hor, 0.5 - p.ver);
//         tmp.push_back(newP);
//     }
//     return tmp;
// }


std::vector<std::string> getLines(std::string s){
    std::string delimiterSemiColon = ";";
    size_t pos = 0;
    std::vector<std::string> lines;
    std::string line;
    while((pos = s.find(delimiterSemiColon)) != std::string::npos) {
        line = s.substr(0,pos);
        lines.push_back(line);
        s.erase(0,pos+delimiterSemiColon.length());
    }
    return lines;
}

std::vector<cv::Point2f> extractConePosition(std::string s){
    std::vector<cv::Point2f> pos;
    std::string delimiterComma = ",";
    std::string y, x;
    auto delInd = s.find(delimiterComma);
    auto lines = getLines(s);
    for (auto &l : lines){
        x = l.substr(0,delInd);
        y = l.substr(delInd + 2, l.size());
        pos.emplace_back(std::stof(x), std::stof(y));
    }
    return pos;
}

// float getVectorFloatAverage(std::vector<float> &v) {
//     float avg(0);
//     if (v.size() != 0) {
//         for (size_t i = 0; i < v.size(); i++) {
//             avg += v[i];
//         }
//         avg /= v.size();
//     }
//     return avg;
// }

// Point getAverage(std::vector<Point> &v, size_t firstPoint, size_t lastPoint){
//     Point avg{0,0};
//     //for (auto &p : v){
//     size_t len = v.size();
//     if (len > lastPoint)
//         len = lastPoint;
//     for(size_t i = firstPoint - 1; i < len; i++){
//         avg.ver += v[i].ver;
//         avg.hor += v[i].hor;
//         //std::cout << "v.ver: " << v[i].ver << std::endl;
//         //std::cout << "v.hor: " << v[i].hor << std::endl;
//     }
//     avg.ver /= v.size();
//     avg.hor /= v.size();
//     return avg;
// }

//Used in getSteeringAngle_old.
// Point averageFp(std::vector<Point> &v1, std::vector<Point> &v2, size_t firstPoint, size_t lastPoint){
//     Point avg1{0,0};
//     Point avg2{0,0};
//     bool zero = false;
//     if (v1.size() > 0)
//         avg1 = getAverage(v1, firstPoint, lastPoint);
//     else
//         zero = true;
//     if (v2.size() > 0)
//         avg2 = getAverage(v2, firstPoint, lastPoint);
//     else
//         zero = true;
//     Point avg{0,0};
//     if (!zero){
//         avg.ver = (avg1.ver + avg2.ver)/2;
//         avg.hor = (avg1.hor + avg2.hor)/2;
//     }
//     return avg;
// }

//The old steeringangle function, uses the average of the nPoints closest cones as the angle to steer towards.
// float getSteeringAngle_old(std::vector<Point> &bluePosition, std::vector<Point> &yellowPosition, size_t &nPoints, float &turnSpeed) {
//     Point fp = averageFp(bluePosition, yellowPosition, 1, nPoints); //works just like before if firstPoint=1.

//     double fpAngle = atan2(fp.hor, fp.ver);
//     float steeringAngle = static_cast<float>(fpAngle);
//     if (bluePosition.size() == 0 && yellowPosition.size() == 0)
//         steeringAngle = 0;
//     else if (bluePosition.size() == 0)
//         steeringAngle = - turnSpeed;
//     else if ( yellowPosition.size() == 0)
//         steeringAngle = turnSpeed;

//     std::cout << steeringAngle << std::endl;
//     if (fabs(steeringAngle) < 0.05)
//         steeringAngle = 0;
    
//     return steeringAngle;
// }


// Point averagePoint(std::vector<Point> &v1, std::vector<Point> &v2, size_t firstPoint, size_t lastPoint){
//     Point avg1{0,0};
//     Point avg2{0,0};
    
//     if (v1.size() > 0) {
//         avg1 = getAverage(v1, firstPoint, lastPoint);
//     } else {
//         avg1.hor = -0.5;
//         avg1.ver = 1;
//     }
//     if (v2.size() > 0) {
//         avg2 = getAverage(v2, firstPoint, lastPoint);
//     } else {
//         avg2.hor = 0.5;
//         avg2.ver = 1;
//     }
    
//     Point avg{0,0};
//     avg.ver = (avg1.ver + avg2.ver)/2;
//     avg.hor = (avg1.hor + avg2.hor)/2;
        
//     return avg;
// }


//Max steer +-38 deg = +-19/90*PI rad
//A positive steering angle is defined as steering to the left
// float getSteeringAngle(std::vector<Point> &bluePosition,
//                        std::vector<Point> &yellowPosition,
//                        size_t &nPoints,
//                        size_t &nSavedAngles,
//                        float &latestSteeringAngle,
//                        float &averageCloseAngle,
//                        float &averageFarAngle,
//                        std::vector<float> &nLatestCloseAngles,
//                        std::vector<float> &nLatestFarAngles,
//                        float &c1, float &c2, float &c3
//                        ) {
    
//     #define UNUSED(x) (void)(x)
//     float pi = (float) M_PI;
//     Point close = averagePoint(bluePosition, yellowPosition, 1, 2);
//     float closeAngle = static_cast<float>(pi/2 - atan2(close.ver, close.hor));
//     Point far = averagePoint(bluePosition, yellowPosition, 2, nPoints);
//     float farAngle = static_cast<float>(pi/2 - atan2(far.ver, far.hor));
    
//     //std::cout << "Close point: " << close.hor << " | " << close.ver << std::endl;
//     //std::cout << "Close angle: " << closeAngle << std::endl;
//     //std::cout << "Far point: " << far.hor << " | " << far.ver << std::endl;
//     //std::cout << "Far angle: " << farAngle << std::endl;
    
//     nLatestCloseAngles.push_back(closeAngle);
//     nLatestCloseAngles.push_back(farAngle);
//     if (nLatestCloseAngles.size() > nSavedAngles) {
//         nLatestCloseAngles.erase(nLatestCloseAngles.begin());
//     }
//     if (nLatestFarAngles.size() > nSavedAngles) {
//         nLatestFarAngles.erase(nLatestFarAngles.begin());
//     }
    
//     averageCloseAngle = getVectorFloatAverage(nLatestCloseAngles);
//     averageFarAngle = getVectorFloatAverage(nLatestFarAngles);
//     float dCloseAngle = closeAngle - averageCloseAngle;
//     float dFarAngle = farAngle - averageFarAngle;
    
//     //float steeringAngle = (latestSteeringAngle + c1*dCloseAngle + c2*dFarAngle + c3*averageCloseAngle);
//     float steeringAngle = c1*averageCloseAngle + c2*averageFarAngle + c3*0;
    
//     //std::cout << "size: " << yellowPosition.size() << std::endl;
//     UNUSED(dFarAngle);
//     UNUSED(dCloseAngle);
//     UNUSED(latestSteeringAngle);
    
//     //std::cout << "Steering angle: " << steeringAngle << std::endl;
//     return steeringAngle;
// }


int32_t main(int32_t argc, char **argv){
    int32_t retCode{0};
    
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0  == commandlineArguments.count("freq") ||
        0 == commandlineArguments.count("xSpeed") ||
        0 == commandlineArguments.count("nPoints") ||
        0 == commandlineArguments.count("maxSteeringAt") ||
        0 == commandlineArguments.count("nSavedAngles") ||
        0 == commandlineArguments.count("cid") ||
        0 == commandlineArguments.count("c1") ||
        0 == commandlineArguments.count("c2") ||
        0 == commandlineArguments.count("c3")){
            std::cerr << argv[0] << "argument error" << std::endl;
        retCode = 1;
    }
    else {
        float const freq = std::stof(commandlineArguments["freq"]);
        float xSpeed = std::stof(commandlineArguments["xSpeed"]);
        size_t nPoints = std::stoi(commandlineArguments["nPoints"]);
        float maxSteeringAt = std::stof(commandlineArguments["maxSteeringAt"]);
        size_t nSavedAngles = std::stoi(commandlineArguments["nSavedAngles"]);
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};
        float c1 = std::stof(commandlineArguments["c1"]);
        float c2 = std::stof(commandlineArguments["c2"]);
        float c3 = std::stof(commandlineArguments["c3"]);

        const int steeringStrategy{commandlineArguments.count("steeringStrategy") ? std::stoi(commandlineArguments["steeringStrategy"]): DEFAULT_STEERING_STRATEGY};
        const int pedalPositionStrategy{commandlineArguments.count("pedalPositionStrategy") ? std::stoi(commandlineArguments["pedalPositionStrategy"]): DEFAULT_PEDAL_POSITION_STRATEGY};
        const float movingAverageCoeff{commandlineArguments.count("movingAverageCoeff") ? std::stof(commandlineArguments["movingAverageCoeff"]): MOVING_AVERAGE_COEFF};
        const float maxAllowedKiwiHeight{commandlineArguments.count("maxAllowedKiwiHeight") ? std::stof(commandlineArguments["maxAllowedKiwiHeight"]) : MAX_ALLOWED_FRONT_KIWI_HEIGHT};
        const float maxAllowedKiwiHeightBeforeReducingSpeed{commandlineArguments.count("maxAllowedKiwiHeightBeforeReducingSpeed") ? std::stof(commandlineArguments["maxAllowedKiwiHeightBeforeReducingSpeed"]) : MAX_ALLOWED_FRONT_KIWI_HEIGHT_BEFORE_REDUCING_SPEED};

        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const bool TUNE{commandlineArguments.count("tune") != 0};
        const bool stopAtJunction{commandlineArguments.count("stopAtJunction") != 0};
        
        
        std::vector<float> nLatestCloseAngles, nLatestFarAngles;
        float latestSteeringAngle, averageCloseAngle, averageFarAngle;

        Behavior behavior;

        if (stopAtJunction) {behavior.enableStopAtJunction();}
        switch (steeringStrategy) {
            case 1:
                behavior.useStrategy1ForSteering(maxSteeringAt, nPoints);
                break;
            default:
                std::cerr<<"Steering strategy not found"<<std::endl;
                return retCode;
        }
        
        switch (pedalPositionStrategy) {
            case 1:
                behavior.useStrategy1ForPedal(xSpeed, maxAllowedKiwiHeight);
                break;
            case 2:
                behavior.useStrategy2ForPedal(xSpeed, maxAllowedKiwiHeight, maxAllowedKiwiHeightBeforeReducingSpeed);
                break;
            default:
                std::cerr<<"Pedal position strategy not found"<<std::endl;
                return retCode;
        }
        
        behavior.setMovingAverageCoeff(movingAverageCoeff);

        int forNPointsInTrackbar = (int) nPoints-1;
        int forMaxSteeringInTrackbar = (int) (maxSteeringAt*100);
        int forXspeedInTrackbar = (int) (xSpeed*100);
        int forMovingAverageCoeffInTrackbar = (int) (movingAverageCoeff*100);

        if (TUNE) {
            cv::namedWindow(TUNNING_WINDOW_NAME, 1);
            cv::createTrackbar("nPoints-1", TUNNING_WINDOW_NAME, &forNPointsInTrackbar, 10);
            cv::createTrackbar("maxSteeringAt/100", TUNNING_WINDOW_NAME, &forMaxSteeringInTrackbar, 150);
            cv::createTrackbar("xSpeed/100", TUNNING_WINDOW_NAME, &forXspeedInTrackbar, 140);
            cv::createTrackbar("movingAverageCoeff/100", TUNNING_WINDOW_NAME, &forMovingAverageCoeffInTrackbar, 100);
        }

        auto onConeReading{[&behavior](cluon::data::Envelope &&envelope) noexcept {
            auto data = cluon::extractMessage<opendlv::logic::perception::ObjectProperty>(std::move(envelope));
            if (data.objectId()==BLUE_CONES_ID) {behavior.setBlueConePos(extractConePosition(data.property()));}
            else if (data.objectId()==YELLOW_CONES_ID) {behavior.setYellowConePos(extractConePosition(data.property()));}
            else if (data.objectId()==RED_CONES_ID) {behavior.setRedConePos(extractConePosition(data.property()));}
        }};

        auto onKiwiReading{[&behavior](cluon::data::Envelope &&envelope) {
            auto kiwiReading = cluon::extractMessage<opendlv::logic::sensation::BoundingBox>(std::move(envelope));
            behavior.setKiwiPos(kiwiReading.x(), kiwiReading.y(), kiwiReading.width(), kiwiReading.height());
        }};

        auto onDistanceReading{[&behavior](cluon::data::Envelope &&envelope) {
            auto distanceReading = cluon::extractMessage<opendlv::proxy::DistanceReading>(std::move(envelope));
            uint32_t const senderStamp = envelope.senderStamp();
            if (senderStamp==0) {behavior.setFrontDistance(distanceReading.distance());} 
            else if (senderStamp==1) {behavior.setRearDistance(distanceReading.distance());}
        }};

        auto onVoltageReading{[&behavior](cluon::data::Envelope &&envelope) {
            auto voltageReading = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(envelope));
            uint32_t const senderStamp = envelope.senderStamp();
            if (senderStamp==0) {behavior.setLeftVoltage(voltageReading.voltage());}
            else if (senderStamp==1) {behavior.setRightVoltage(voltageReading.voltage());}
        }};

        auto atFrequency{[&od4, &behavior, &steeringStrategy, &pedalPositionStrategy, &forNPointsInTrackbar, &maxAllowedKiwiHeight, &maxAllowedKiwiHeightBeforeReducingSpeed, &forMaxSteeringInTrackbar, &forXspeedInTrackbar, &forMovingAverageCoeffInTrackbar, &VERBOSE, &TUNE]() {
            
            if (TUNE) {
                switch (steeringStrategy) {
                    case 1:
                        behavior.useStrategy1ForSteering(((float)forMaxSteeringInTrackbar)/100.f, forNPointsInTrackbar+1);
                        break;
                }
                
                switch (pedalPositionStrategy) {
                    case 1:
                        behavior.useStrategy1ForPedal(((float)forXspeedInTrackbar)/100.f, maxAllowedKiwiHeight);
                        break;
                    case 2:
                        behavior.useStrategy2ForPedal(((float)forXspeedInTrackbar)/100.f, maxAllowedKiwiHeight, maxAllowedKiwiHeightBeforeReducingSpeed);
                        break;
                }
                
                behavior.setMovingAverageCoeff(((float)forMovingAverageCoeffInTrackbar)/100.f);
            }

            float groundSteering, pedalPosition;
            std::tie(groundSteering, pedalPosition) = behavior.stepTime();
            
            opendlv::proxy::GroundSteeringRequest gsr;
            opendlv::proxy::PedalPositionRequest ppr;
            gsr.groundSteering(groundSteering);
            ppr.position(pedalPosition);
            od4.send(gsr);
            od4.send(ppr);
            
            if (TUNE) {
                cv::waitKey(1);
            }

            if (VERBOSE) {
                std::cout<<"groundSteering:  "<<groundSteering<<"pedalPosition:  "<<pedalPosition<<"\n";
            }

            return true;
        }};
/*
        auto atFrequency{[&od4,
                          &bluePosition,
                          &yellowPosition,
                          &redPosition,
                          &latestSteeringAngle,
                          &averageCloseAngle,
                          &averageFarAngle,
                          &nLatestCloseAngles,
                          &nLatestFarAngles,
                          &nSavedAngles,
                          &xSpeed,
                          &nPoints,
                          &turnSpeed,
                          &c1, &c2, &c3]() -> bool {
            
            //Sets the steering angle
            opendlv::proxy::GroundSteeringRequest gsr;
            float steeringAngle = getSteeringAngle_old(bluePosition, yellowPosition, nPoints, turnSpeed);
            float steeringAngle = getSteeringAngle(bluePosition,
                                                  yellowPosition,
                                                  nPoints,
                                                  nSavedAngles,
                                                  latestSteeringAngle,
                                                  averageCloseAngle,
                                                  averageFarAngle,
                                                  nLatestCloseAngles,
                                                  nLatestFarAngles,
                                                  c1, c2, c3
                                                  );
            gsr.groundSteering(steeringAngle);

            //Sets the speed
            float pedalPosition = xSpeed;
            opendlv::proxy::PedalPositionRequest ppr;
            ppr.position(pedalPosition);
            
            //Sends the steering angle and the speed
            od4.send(gsr);
            od4.send(ppr);
            return true;
        }};
*/

        od4.dataTrigger(opendlv::logic::perception::ObjectProperty::ID(), onConeReading);
        od4.dataTrigger(opendlv::logic::sensation::BoundingBox::ID(), onKiwiReading);
        od4.dataTrigger(opendlv::proxy::DistanceReading::ID(), onDistanceReading);
        od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);
        od4.timeTrigger(freq, atFrequency);
    }

    return retCode;

}
