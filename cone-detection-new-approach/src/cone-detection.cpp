/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include <string>

// const int H_LOW_BLUE = 110;
// const int H_HIGH_BLUE = 130;
// const int S_LOW_BLUE = 80;
// const int S_HIGH_BLUE = 255;
// const int V_LOW_BLUE = 40;
// const int V_HIGH_BLUE = 255;

// const int H_LOW_YELLOW = 15;
// const int H_HIGH_YELLOW = 30;
// const int S_LOW_YELLOW = 0;
// const int S_HIGH_YELLOW = 255;
// const int V_LOW_YELLOW = 0;
// const int V_HIGH_YELLOW = 255;

// const int H_LOW_RED = 15;
// const int H_HIGH_RED = 30;
// const int S_LOW_RED = 0;
// const int S_HIGH_RED = 255;
// const int V_LOW_RED = 0;
// const int V_HIGH_RED = 255;

const int H_LOW_HORIZON = 90;
const int H_HIGH_HORIZON = 80;
const int S_LOW_HORIZON = 100;
const int S_HIGH_HORIZON = 100;
const int V_LOW_HORIZON = 160;
const int V_HIGH_HORIZON = 180;

const float CONTRAST_COEFFICIENT = 1.f;

const int APPROX_POLY_EPSILON = 2;//
const int MAX_POLIGON_SIZE = 20;//

const int MIN_CONE_PIXEL_SIZE = 70;
const int MAX_CONE_PIXEL_SIZE = 4000;

const float CONE_MAX_ASPECT_RATIO = 0.8f;

const int HORIZON = 300;

const cv::Point KIWI_CART_RECT_UP_LEFT_POINT(280, 600);
const cv::Point KIWI_CART_RECT_UP_RIGHT_POINT(1000, 600);

const cv::Scalar BLUE_CONE_CONTOUR_COLOR(0,0,255);
const cv::Scalar BLUE_CONE_CONTOUR_CENTER_COLOR(0,255,0);

const cv::Scalar YELLOW_CONE_CONTOUR_COLOR(255,0,0);
const cv::Scalar YELLOW_CONE_CONTOUR_CENTER_COLOR(0,0,255);

const cv::Scalar RED_CONE_CONTOUR_COLOR(0,255,0);
const cv::Scalar RED_CONE_CONTOUR_CENTER_COLOR(255,0,0);

const int CONTOUR_CENTER_CICLE_RADIUS = 4;

const int ERODE_ITERATIONS = 10;
const int DILATE_ITERATIONS = 10;

const std::string ANNOTATED_WINDOW_NAME = "Cones";
const std::string TUNNING_WINDOW_NAME = "Tunning";

const int BLUE_CONES_ID = 0;
const int YELLOW_CONES_ID = 1;
const int RED_CONES_ID = 2;

const float MOVING_AVERAGE_COEFF_HORIZON = 0.7f;


bool convex_hull_pointing_up(std::vector<cv::Point> ch)
{
    cv::Rect boundRect = cv::boundingRect(ch);
    float width = boundRect.width;
    float height = boundRect.height;
    float y = boundRect.y;
    if (width/height<CONE_MAX_ASPECT_RATIO)
    {
        float vertical_center = y + height/2;
        std::vector<cv::Point> points_above_center, points_below_center;
        for (auto point : ch)
        {
            if (point.y<vertical_center) {points_above_center.push_back(point);}
            else {points_below_center.push_back(point);}
        }
        float left_x = points_below_center[0].x;
        float right_x = points_below_center[0].x;
        for (auto point : points_below_center)
        {
            float x = point.x;
            if (x < left_x) {left_x = x;}
            if (x > right_x) {right_x = x;}
        }
        for (auto point : points_above_center)
        {
            float x = point.x;
            if (x<left_x || x>right_x) {return false;}
        }
        return true;
    }
    return false;
}


void inRange(const cv::Mat &HSVImg, const cv::Scalar &low, const cv::Scalar &high, cv::Mat &HSVImgFiltered) {
    
    std::vector<cv::Scalar> lows = { low };
    std::vector<cv::Scalar> highs = { high };
    
    if (low[0]>high[0]) {
        int size = lows.size();
        for (int idx=0; idx<size; idx++) {
            lows.push_back(lows[idx]);
            highs.push_back(cv::Scalar(255, lows[idx][1], lows[idx][2]));
            lows.push_back(cv::Scalar(0, lows[idx][1], lows[idx][2]));
            highs.push_back(highs[idx]);
        }
        lows.erase(lows.begin(), lows.begin()+size);
        highs.erase(highs.begin(), highs.begin()+size);
    }

    if (low[1]>high[2]) {
        int size = lows.size();
        for (int idx=0; idx<size; idx++) {
            lows.push_back(lows[idx]);
            highs.push_back(cv::Scalar(lows[idx][0], 255, lows[idx][2]));
            lows.push_back(cv::Scalar(lows[idx][0], 0, lows[idx][2]));
            highs.push_back(highs[idx]);
        }
        lows.erase(lows.begin(), lows.begin()+size);
        highs.erase(highs.begin(), highs.begin()+size);
    }

    if (low[1]>high[2]) {
        int size = lows.size();
        for (int idx=0; idx<size; idx++) {
            lows.push_back(lows[idx]);
            highs.push_back(cv::Scalar(lows[idx][0], lows[idx][1], 255));
            lows.push_back(cv::Scalar(lows[idx][0], lows[idx][1], 0));
            highs.push_back(highs[idx]);
        }
        lows.erase(lows.begin(), lows.begin()+size);
        highs.erase(highs.begin(), highs.begin()+size);
    }

    cv::inRange(HSVImg, lows[0], highs[0], HSVImgFiltered);
    for (unsigned int idx=1; idx<lows.size(); idx++) {
        cv::Mat mask;
        cv::inRange(HSVImg, lows[idx], highs[idx], mask);
        cv::bitwise_or(HSVImgFiltered, mask, HSVImgFiltered);
    }
}


// std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>> detectConeContours(const cv::Mat &img, 
//                                                                                            const cv::Mat &mask, 
//                                                                                            const cv::Scalar &low, 
//                                                                                            const cv::Scalar &high, 
//                                                                                            const int &erodeIterations, 
//                                                                                            const int &dilateIterations){
    
//     cv::Mat HSVImg;
//     cv::cvtColor(img, HSVImg, cv::COLOR_BGR2HSV); 

//     cv::Mat HSVImgFiltered;
//     inRange(HSVImg, low, high, HSVImgFiltered);

//     cv::Mat applyDilate;
//     cv::dilate(HSVImgFiltered, applyDilate, cv::Mat(), cv::Point(-1, -1), dilateIterations, 1, 1);

//     cv::Mat applyErode;
//     cv::erode(applyDilate, applyErode, cv::Mat(), cv::Point(-1, -1), erodeIterations, 1, 1);

//     cv::Mat maskedImg(img.size().height, img.size().width, CV_8UC4, cv::Scalar(0,0,0));
//     applyErode.copyTo(maskedImg, mask);

//     std::vector<std::vector<cv::Point>> contours;
//     std::vector<cv::Vec4i> hierarchy;
//     cv::findContours(maskedImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//     std::vector <cv::Point> centerPoints;
//     std::vector<std::vector<cv::Point>> valiedContours;
//     for (auto& c:contours) {
//         cv::Rect boundRect = cv::boundingRect(c);
//         if (boundRect.width<boundRect.height)
//         {
//             centerPoints.push_back((boundRect.tl()+boundRect.br())/2); 
//             valiedContours.push_back(c); 
//         }
//     }

//     return {valiedContours, centerPoints};
// }



std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>> detectConeContours(const cv::Mat &img, 
                                                                                           const cv::Mat &mask, 
                                                                                           const cv::Scalar &low, 
                                                                                           const cv::Scalar &high, 
                                                                                           const int &erodeIterations, 
                                                                                           const int &dilateIterations){
    
    cv::Mat LABImg;
    cv::cvtColor(img, LABImg, cv::COLOR_BGR2Lab); 

    cv::Mat filteredImg;
    inRange(LABImg, low, high, filteredImg);

    cv::Mat applyDilate;
    cv::dilate(filteredImg, applyDilate, cv::Mat(), cv::Point(-1, -1), dilateIterations, 1, 1);

    cv::Mat applyErode;
    cv::erode(applyDilate, applyErode, cv::Mat(), cv::Point(-1, -1), erodeIterations, 1, 1);

    cv::Mat maskedImg(img.size().height, img.size().width, CV_8UC4, cv::Scalar(0,0,0));
    applyErode.copyTo(maskedImg, mask);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(maskedImg, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector <cv::Point> centerPoints;
    std::vector<std::vector<cv::Point>> valiedContours;
    for (auto& c:contours) {
        cv::Rect boundRect = cv::boundingRect(c);
        if (boundRect.width<boundRect.height)
        {
            centerPoints.push_back((boundRect.tl()+boundRect.br())/2); 
            valiedContours.push_back(c); 
        }
    }

    return {valiedContours, centerPoints};
}



std::vector<unsigned int> coneValidation(std::vector<std::vector<cv::Point>> contours, std::vector<cv::Point> contoursCenters, bool sellectPointingUpConesOnly, int minConePixelSize, int maxConePixelSize, int horizon){

    std::vector<unsigned int> valiedContoursIdx;

    for (unsigned int idx=0; idx<contours.size(); idx++)
    {
        if (contoursCenters[idx].y>horizon)
        {
            std::vector<cv::Point> polyApproximation;
            approxPolyDP(contours[idx], polyApproximation, APPROX_POLY_EPSILON, true);
            
            std::vector<cv::Point> polyApproximationConvexHull;
            cv::convexHull(polyApproximation, polyApproximationConvexHull);

            if (polyApproximationConvexHull.size()<=MAX_POLIGON_SIZE)
            {
                float area = (float) cv::contourArea(polyApproximationConvexHull);
                if ((minConePixelSize<=area) && (maxConePixelSize>=area))
                {
                    if (sellectPointingUpConesOnly)
                    {
                        if (convex_hull_pointing_up(polyApproximationConvexHull))
                        {
                            valiedContoursIdx.push_back(idx);
                        }
                    }
                    else {valiedContoursIdx.push_back(idx);}
                }
            }
        }
    }
    return valiedContoursIdx;
}


// std::vector<cv::Point2f> getContoursMassCenter(const std::vector<std::vector<cv::Point>> & contours) {

//     std::vector<cv::Point2f> mc(contours.size());

//     for (auto& c:contours) {
//       auto m = moments(c, false); 
//       mc.push_back(cv::Point2f((float) (m.m10/m.m00), (float) (m.m01/m.m00))); 
//     }
//     return mc;
// }


float getAverageHorisentalPos(std::vector<cv::Point> coneCenters) {
    float sum = 0;
    for (auto coneCenter : coneCenters) {sum+=coneCenter.x;}
    return sum/coneCenters.size();
}


std::string convertToNormalizedCsvString(std::vector<cv::Point> coneCenters, unsigned int imgWidth, unsigned int imgHeight) {
    
    std::string csvString;
    for (auto coneCenter : coneCenters) {
        csvString.append(std::to_string(((float)coneCenter.x)/((float)(imgWidth))) + "," + std::to_string(((float)coneCenter.y)/((float)(imgHeight))) + ";");
    }
    return csvString;
}


int findHorizon(const cv::Mat &img,
                const cv::Scalar &low,
                const cv::Scalar &high, 
                int maxHorizon,
                int prevHorizon)
{
    
    cv::Mat HSVImg;
    cv::cvtColor(img, HSVImg, cv::COLOR_BGR2HSV); 

    cv::Mat HSVImgFiltered;
    inRange(HSVImg, low, high, HSVImgFiltered);

    cv::Mat applyDilate;
    cv::dilate(HSVImgFiltered, applyDilate, cv::Mat(), cv::Point(-1, -1), 15, 1, 1);

    cv::Mat applyErode;
    cv::erode(applyDilate, applyErode, cv::Mat(), cv::Point(-1, -1), 15, 1, 1);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(applyDilate, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    int minY = 0;
    int maxY = 0;
    std::vector <cv::Point> centerPoints;
    std::vector<std::vector<cv::Point>> valiedContours;
    for (auto& c:contours) {
        cv::Rect boundRect = cv::boundingRect(c);
        int up = boundRect.y;
        int down = boundRect.y+boundRect.height;
        minY = (minY>up) ? up : minY;
        maxY = (maxY<down) ? down : maxY;
    }

    if (minY==0 && maxY==0) {return prevHorizon;}
    float horizon = 0.95f*((float)maxY) + 0.05f*((float)minY);
    return std::max(maxHorizon, (int)((1-MOVING_AVERAGE_COEFF_HORIZON)*prevHorizon + MOVING_AVERAGE_COEFF_HORIZON*horizon));
}




cv::Mat getGroundMask(const cv::Mat &img,
                      const cv::Scalar &low,
                      const cv::Scalar &high)
{
    cv::Mat blur;
    cv::GaussianBlur(img, blur, cv::Size(21, 21), 0);

    cv::Mat HSVImg;
    cv::cvtColor(blur, HSVImg, cv::COLOR_BGR2HSV); 

    cv::Mat HSVImgFiltered;
    inRange(HSVImg, low, high, HSVImgFiltered);

    cv::Mat HSVImgFiltered1;
    inRange(blur, cv::Scalar(200,200, 200), cv::Scalar(255,255,255), HSVImgFiltered1);

    HSVImgFiltered = HSVImgFiltered | HSVImgFiltered1;
    cv::Mat applyDilate;
    cv::dilate(HSVImgFiltered, applyDilate, cv::Mat(), cv::Point(-1, -1), 35, 1, 1);

    cv::Mat applyErode;
    cv::erode(applyDilate, applyErode, cv::Mat(), cv::Point(-1, -1), 25, 1, 1);

    cv::Mat applyErode1;
    applyErode.convertTo(applyErode1, CV_32F);
    applyErode1 = applyErode1/255*2-1;


    cv::Mat kernel = (cv::Mat_<float>(2,3) << 1, 1,1 ,-1,-1, -1);
    cv::Mat filtered;
    cv::filter2D(applyErode1, filtered, -1 , kernel, cv::Point(-1, -1), 0, 4);

    cv::Mat filteredFilter;
    cv::inRange(filtered, cv::Scalar(4), cv::Scalar(10), filteredFilter);

    cv::Mat edges;
    cv::Canny(filteredFilter, edges, 100, 200);
// return edges;
    // return edges;
    // cv::Moments m = moments(applyErode,true);
    // cv::rectangle(applyErode, cv::Point(0, 0), cv::Point(img.cols, (int)(m.m01/m.m00)), cv::Scalar(255), -1, cv::LINE_8);

    // cv::Mat edges;
    // cv::Canny(applyErode, edges, 100, 200);

    std::vector<cv::Vec2f> lines;
    cv::HoughLines(edges, lines, 4, 0.75*CV_PI/180, 150, 0, 0 );

    std::vector<std::vector<cv::Point>> contours;
    std::vector <int> y1s, y2s;
    std::vector<float> counts;
    for( size_t i = 0; i < lines.size(); i++ )
    {
        if (i>5) {break;}
        float r = lines[i][0], theta = lines[i][1];
        if (std::abs(theta)<3.1514/4) {continue;}
        int y1 = (int)(r/std::sin(theta));
        int y2 = (int)(r/std::sin(theta) - ((float)img.cols)/std::tan(theta));
        bool added = false;
        for (size_t idx=0; idx<counts.size(); idx++) {
            if (std::abs(y1-y1s[idx])<20 && std::abs(y2-y2s[idx])<20) {
                y1s[idx] = y1s[idx] * counts[idx]/(counts[idx]+1) + y1 * 1/(counts[idx]+1);
                y2s[idx] = y2s[idx] * counts[idx]/(counts[idx]+1) + y1 * 1/(counts[idx]+1);
                counts[idx]++;
                added = true;
                break;
            }
        }
        if (!added) {
            y1s.push_back(y1);
            y2s.push_back(y2);
            counts.push_back(1);
        }
    }
    for (size_t idx=0; idx<counts.size(); idx++) {
        contours.push_back({cv::Point(0,std::max(y1s[idx],0)), cv::Point(0,0), cv::Point(img.cols,0), cv::Point(img.cols,std::max(y2s[idx],0))});
        // cv::line(img, cv::Point(0,y1s[idx]), cv::Point(img.cols,y2s[idx]), cv::Scalar(255, 255, 255), 1, cv::LINE_8);
    }

    cv::Mat groundMask(img.rows, img.cols, CV_8U, cv::Scalar(255,255,255));
    for (int i=0; i<contours.size(); i++) cv::drawContours(groundMask, contours, i, cv::Scalar(0, 0, 0), -1, cv::LINE_8);

    cv::Mat applyErode2;
    cv::dilate(groundMask, applyErode2, cv::Mat(), cv::Point(-1, -1), 15, 1, 1);


    cv::Mat maskedImg1(img.size().height, img.size().width, CV_8UC4, cv::Scalar(0,0,0));
    
    cv::Mat final;
    img.copyTo(final, applyErode2);

    return final;
}





















int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=112 --name=img.argb --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};
        const bool TUNE{commandlineArguments.count("tune") != 0};

        // int hLowBlue = commandlineArguments.count("hLowBlue") ? std::stoi(commandlineArguments["hLowBlue"]) : H_LOW_BLUE;
        // int hHighBlue = commandlineArguments.count("hHighBlue") ? std::stoi(commandlineArguments["hHighBlue"]) : H_HIGH_BLUE;
        // int sLowBlue = commandlineArguments.count("sLowBlue") ? std::stoi(commandlineArguments["sLowBlue"]) : S_LOW_BLUE;
        // int sHighBlue = commandlineArguments.count("sHighBlue") ? std::stoi(commandlineArguments["sHighBlue"]) : S_HIGH_BLUE;
        // int vLowBlue = commandlineArguments.count("vLowBlue") ? std::stoi(commandlineArguments["vLowBlue"]) : V_LOW_BLUE;
        // int vHighBlue = commandlineArguments.count("vHighBlue") ? std::stoi(commandlineArguments["vHighBlue"]) : V_HIGH_BLUE;

        // int hLowYellow = commandlineArguments.count("hLowYellow") ? std::stoi(commandlineArguments["hLowYellow"]) : H_LOW_YELLOW;
        // int hHighYellow = commandlineArguments.count("hHighYellow") ? std::stoi(commandlineArguments["hHighYellow"]) : H_HIGH_YELLOW;
        // int sLowYellow = commandlineArguments.count("sLowYellow") ? std::stoi(commandlineArguments["sLowYellow"]) : S_LOW_YELLOW;
        // int sHighYellow = commandlineArguments.count("sHighYellow") ? std::stoi(commandlineArguments["sHighYellow"]) : S_HIGH_YELLOW;
        // int vLowYellow = commandlineArguments.count("vLowYellow") ? std::stoi(commandlineArguments["vLowYellow"]) : V_LOW_YELLOW;
        // int vHighYellow = commandlineArguments.count("vHighYellow") ? std::stoi(commandlineArguments["vHighYellow"]) : V_HIGH_YELLOW;

        // int hLowRed = commandlineArguments.count("hLowRed") ? std::stoi(commandlineArguments["hLowRed"]) : H_LOW_RED;
        // int hHighRed = commandlineArguments.count("hHighRed") ? std::stoi(commandlineArguments["hHighRed"]) : H_HIGH_RED;
        // int sLowRed = commandlineArguments.count("sLowRed") ? std::stoi(commandlineArguments["sLowRed"]) : S_LOW_RED;
        // int sHighRed = commandlineArguments.count("sHighRed") ? std::stoi(commandlineArguments["sHighRed"]) : S_HIGH_RED;
        // int vLowRed = commandlineArguments.count("vLowRed") ? std::stoi(commandlineArguments["vLowRed"]) : V_LOW_RED;
        // int vHighRed = commandlineArguments.count("vHighRed") ? std::stoi(commandlineArguments["vHighRed"]) : V_HIGH_RED;

        int highBlue = commandlineArguments.count("highBlue") ? std::stoi(commandlineArguments["highBlue"]) : 111;
        int lowYellow = commandlineArguments.count("lowYellow") ? std::stoi(commandlineArguments["lowYellow"]) : 131;

        int hLowHorizon = commandlineArguments.count("hLowHorizon") ? std::stoi(commandlineArguments["hLowHorizon"]) : H_LOW_HORIZON;
        int hHighHorizon = commandlineArguments.count("hHighHorizon") ? std::stoi(commandlineArguments["hHighHorizon"]) : H_HIGH_HORIZON;
        int sLowHorizon = commandlineArguments.count("sLowHorizon") ? std::stoi(commandlineArguments["sLowHorizon"]) : S_LOW_HORIZON;
        int sHighHorizon = commandlineArguments.count("sHighHorizon") ? std::stoi(commandlineArguments["sHighHorizon"]) : S_HIGH_HORIZON;
        int vLowHorizon = commandlineArguments.count("vLowHorizon") ? std::stoi(commandlineArguments["vLowHorizon"]) : V_LOW_HORIZON;
        int vHighHorizon = commandlineArguments.count("vHighHorizon") ? std::stoi(commandlineArguments["vHighHorizon"]) : V_HIGH_HORIZON;

        float contrastCoeff = commandlineArguments.count("contrastCoeff") ? std::stof(commandlineArguments["contrastCoeff"]) : CONTRAST_COEFFICIENT;

        bool sellectPointingUpCones = (commandlineArguments.count("sellectPointingUpCones")>0);

        int dilateIterations = commandlineArguments.count("dilateIterations") ? std::stoi(commandlineArguments["dilateIterations"]) : DILATE_ITERATIONS;
        int erodeIterations = commandlineArguments.count("erodeIterations") ? std::stoi(commandlineArguments["erodeIterations"]) : ERODE_ITERATIONS;

        int maxConePixelSize = commandlineArguments.count("maxConePixelSize") ? std::stoi(commandlineArguments["maxConePixelSize"]) : MAX_CONE_PIXEL_SIZE;
        int minConePixelSize = commandlineArguments.count("minConePixelSize") ? std::stoi(commandlineArguments["minConePixelSize"]) : MIN_CONE_PIXEL_SIZE;

        int maxHorizon = commandlineArguments.count("maxHorizon") ? std::stoi(commandlineArguments["maxHorizon"]) : HORIZON;
        bool adaptiveHorizon = (commandlineArguments.count("adaptiveHorizon")>0);

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session; here, you can send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};


            if (VERBOSE) 
            {
                cv::namedWindow(ANNOTATED_WINDOW_NAME, 1);

                if (TUNE)
                {
                    cv::namedWindow(TUNNING_WINDOW_NAME, 1);

                    cv::createTrackbar("erodeIter", TUNNING_WINDOW_NAME, &erodeIterations, 10);

                    cv::createTrackbar("dilateIter", TUNNING_WINDOW_NAME, &dilateIterations, 10);

                    cv::createTrackbar("highBlue", TUNNING_WINDOW_NAME, &highBlue, 255);

                    cv::createTrackbar("lowYellow", TUNNING_WINDOW_NAME, &lowYellow, 255);

                    cv::createTrackbar("hLowHorizon", TUNNING_WINDOW_NAME, &hLowHorizon, 255);

                    cv::createTrackbar("hHighHorizon", TUNNING_WINDOW_NAME, &hHighHorizon, 255);

                    cv::createTrackbar("sLowHorizon", TUNNING_WINDOW_NAME, &sLowHorizon, 255);

                    cv::createTrackbar("sHighHorizon", TUNNING_WINDOW_NAME, &sHighHorizon, 255);

                    cv::createTrackbar("vLowHorizon", TUNNING_WINDOW_NAME, &vLowHorizon, 255);

                    cv::createTrackbar("vHighHorizon", TUNNING_WINDOW_NAME, &vHighHorizon, 255);

                    // cv::createTrackbar("hLowBlue", TUNNING_WINDOW_NAME, &hLowBlue, 255);

                    // cv::createTrackbar("hHighBlue", TUNNING_WINDOW_NAME, &hHighBlue, 255);

                    // cv::createTrackbar("sLowBlue", TUNNING_WINDOW_NAME, &sLowBlue, 255);

                    // cv::createTrackbar("sHighBlue", TUNNING_WINDOW_NAME, &sHighBlue, 255);

                    // cv::createTrackbar("vLowBlue", TUNNING_WINDOW_NAME, &vLowBlue, 255);

                    // cv::createTrackbar("vHighBlue", TUNNING_WINDOW_NAME, &vHighBlue, 255);

                    // cv::createTrackbar("hLowYellow", TUNNING_WINDOW_NAME, &hLowYellow, 255);

                    // cv::createTrackbar("hHighYellow", TUNNING_WINDOW_NAME, &hHighYellow, 255);

                    // cv::createTrackbar("sLowYellow", TUNNING_WINDOW_NAME, &sLowYellow, 255);

                    // cv::createTrackbar("sHighYellow", TUNNING_WINDOW_NAME, &sHighYellow, 255);

                    // cv::createTrackbar("vLowYellow", TUNNING_WINDOW_NAME, &vLowYellow, 255);

                    // cv::createTrackbar("vHighYellow", TUNNING_WINDOW_NAME, &vHighYellow, 255);

                    // cv::createTrackbar("hLowRed", TUNNING_WINDOW_NAME, &hLowRed, 255);

                    // cv::createTrackbar("hHighRed", TUNNING_WINDOW_NAME, &hHighRed, 255);

                    // cv::createTrackbar("sLowRed", TUNNING_WINDOW_NAME, &sLowRed, 255);

                    // cv::createTrackbar("sHighRed", TUNNING_WINDOW_NAME, &sHighRed, 255);

                    // cv::createTrackbar("vLowRed", TUNNING_WINDOW_NAME, &vLowRed, 255);

                    // cv::createTrackbar("vHighRed", TUNNING_WINDOW_NAME, &vHighRed, 255);
                }
            }

            cv::Mat redConesDetectionMask(HEIGHT, WIDTH, CV_8U, cv::Scalar(1,1,1));
            cv::rectangle(redConesDetectionMask, cv::Point(0,0), cv::Point(WIDTH, maxHorizon), cv::Scalar(0, 0, 0), -1, cv::LINE_8);

            cv::Mat blueConesDetectionMask, yellowConesDetectionMask;
            redConesDetectionMask.copyTo(blueConesDetectionMask);
            redConesDetectionMask.copyTo(yellowConesDetectionMask);

            cv::rectangle(blueConesDetectionMask, cv::Point(0, HEIGHT), KIWI_CART_RECT_UP_RIGHT_POINT, cv::Scalar(0, 0, 0), -1, cv::LINE_8);
            cv::rectangle(yellowConesDetectionMask, cv::Point(WIDTH,HEIGHT), KIWI_CART_RECT_UP_LEFT_POINT, cv::Scalar(0, 0, 0), -1, cv::LINE_8);

            int horizon = maxHorizon;

            while (od4.isRunning()) {
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy image into cvMat structure.
                    // Be aware of that any code between lock/unlock is blocking
                    // the camera to provide the next frame. Thus, any
                    // computationally heavy algorithms should be placed outside
                    // lock/unlock
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                sharedMemory->unlock();

                cv::GaussianBlur(img, img, cv::Size(5, 5), 0);

                cv::convertScaleAbs(img, img, contrastCoeff, 0);

                if (adaptiveHorizon) {
                    img = getGroundMask(img,
                                        cv::Scalar(hLowHorizon, sLowHorizon, vLowHorizon), 
                                        cv::Scalar(hHighHorizon, sHighHorizon, vHighHorizon));
                    // cv::imshow(ANNOTATED_WINDOW_NAME, getGroundMask(img,
                    //                     cv::Scalar(hLowHorizon, sLowHorizon, vLowHorizon), 
                    //                     cv::Scalar(hHighHorizon, sHighHorizon, vHighHorizon)));
                    // cv::waitKey(1);

                    // horizon = findHorizon(img,
                    //                                         cv::Scalar(hLowHorizon, sLowHorizon, vLowHorizon), 
                    //                                         cv::Scalar(hHighHorizon, sHighHorizon, vHighHorizon),
                    //                                         maxHorizon,
                    //                                         horizon);
                }

                std::vector<std::vector<cv::Point>> blueConeContours;
                std::vector<std::vector<cv::Point>> yellowConeContours;
                std::vector<std::vector<cv::Point>> redConeContours;
                
                std::vector<cv::Point> blueConeCenters;
                std::vector<cv::Point> yellowConeCenters;
                std::vector<cv::Point> redConeCenters;

                std::tie(blueConeContours, blueConeCenters) = detectConeContours(img, blueConesDetectionMask,
                                                    cv::Scalar(0, 0, 0), 
                                                    cv::Scalar(255, 255, highBlue),
                                                    erodeIterations,
                                                    dilateIterations);

                std::tie(yellowConeContours, yellowConeCenters) = detectConeContours(img, yellowConesDetectionMask, 
                                                    cv::Scalar(0, 0, lowYellow), 
                                                    cv::Scalar(255, 255, 255),
                                                    erodeIterations,
                                                    dilateIterations);

                // std::tie(redConeContours, redConeCenters) = detectConeContours(img, redConesDetectionMask, 
                //                                     cv::Scalar(hLowRed, sLowRed, vLowRed), 
                //                                     cv::Scalar(hHighRed, sHighRed, vHighRed),
                //                                     erodeIterations,
                //                                     dilateIterations);

                std::vector<unsigned int> valiedBlueConesIdx = coneValidation(blueConeContours, blueConeCenters, sellectPointingUpCones, minConePixelSize, maxConePixelSize, horizon);
                std::vector<unsigned int> valiedYellowConesIdx = coneValidation(yellowConeContours, yellowConeCenters, sellectPointingUpCones, minConePixelSize, maxConePixelSize, horizon);
                // std::vector<unsigned int> valiedRedConesIdx = coneValidation(redConeContours, redConeCenters, sellectPointingUpCones, minConePixelSize, maxConePixelSize, horizon);

                std::vector<std::vector<cv::Point>> valiedBlueConeContours;
                std::vector<std::vector<cv::Point>> valiedYellowConeContours;
                // std::vector<std::vector<cv::Point>> valiedRedConeContours;
                
                std::vector<cv::Point> valiedBlueConeCenters;
                std::vector<cv::Point> valiedYellowConeCenters;
                // std::vector<cv::Point> valiedRedConeCenters;

                for (int idx:valiedBlueConesIdx)
                {
                    valiedBlueConeContours.push_back(blueConeContours[idx]);
                    valiedBlueConeCenters.push_back(blueConeCenters[idx]);
                }
                for (int idx:valiedYellowConesIdx)
                {
                    valiedYellowConeContours.push_back(yellowConeContours[idx]);
                    valiedYellowConeCenters.push_back(yellowConeCenters[idx]);
                }
                // for (int idx:valiedRedConesIdx)
                // {
                //     valiedRedConeContours.push_back(redConeContours[idx]);
                //     valiedRedConeCenters.push_back(redConeCenters[idx]);
                // }

                std::string blueConeCentersString = convertToNormalizedCsvString(valiedBlueConeCenters, WIDTH, HEIGHT);
                std::string yellowConeCentersString = convertToNormalizedCsvString(valiedYellowConeCenters, WIDTH, HEIGHT);
                // std::string redConeCentersString = convertToNormalizedCsvString(valiedRedConeCenters, WIDTH, HEIGHT);

                //Send the strings over UDP multicast through the OpenDLV message opendlv.logic.perception.ObjectProperty.
                //The logic microservice is the intended reciever of the messages.
                opendlv::logic::perception::ObjectProperty blueConePositionsMessage;
                opendlv::logic::perception::ObjectProperty yellowConePositionsMessage;
                // opendlv::logic::perception::ObjectProperty redConePositionsMessage;
                
                blueConePositionsMessage.objectId(BLUE_CONES_ID);
                yellowConePositionsMessage.objectId(YELLOW_CONES_ID);
                // redConePositionsMessage.objectId(RED_CONES_ID);
                
                blueConePositionsMessage.property(blueConeCentersString);
                yellowConePositionsMessage.property(yellowConeCentersString);
                // redConePositionsMessage.property(redConeCentersString);
                
                od4.send(blueConePositionsMessage);
                od4.send(yellowConePositionsMessage);
                // od4.send(redConePositionsMessage);

                if (VERBOSE) {
                    // std::cout<<hLowYellow<<'\t'<<hHighYellow<<'\t'<<sLowYellow<<'\t'<<sHighYellow<<'\t'<<vLowYellow<<'\t'<<vHighYellow<<'\n';
                    cv::Mat conesImg;
                    img.copyTo(conesImg);

                    drawContours(conesImg, blueConeContours, -1, BLUE_CONE_CONTOUR_COLOR, 1, 20);
                    for (auto center:valiedBlueConeCenters) {cv::circle(conesImg, center, CONTOUR_CENTER_CICLE_RADIUS, BLUE_CONE_CONTOUR_CENTER_COLOR, CONTOUR_CENTER_CICLE_RADIUS);}

                    drawContours(conesImg, yellowConeContours, -1, YELLOW_CONE_CONTOUR_COLOR, 1, 20);
                    for (auto center:valiedYellowConeCenters) {cv::circle(conesImg, center, CONTOUR_CENTER_CICLE_RADIUS, YELLOW_CONE_CONTOUR_CENTER_COLOR, CONTOUR_CENTER_CICLE_RADIUS);}

                    // drawContours(conesImg, redConeContours, -1, RED_CONE_CONTOUR_COLOR, 1, 20);
                    // for (auto center:valiedRedConeCenters) {cv::circle(conesImg, center, CONTOUR_CENTER_CICLE_RADIUS, RED_CONE_CONTOUR_CENTER_COLOR, CONTOUR_CENTER_CICLE_RADIUS);}

                    cv::line(conesImg, cv::Point(0,horizon), cv::Point(WIDTH, horizon), cv::Scalar(0, 255, 0), cv::LINE_8);

                    cv::imshow(ANNOTATED_WINDOW_NAME, conesImg);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

