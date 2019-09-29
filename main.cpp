#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <map>
#include <string>
#include <vector>

#include "easylogging++.h"
INITIALIZE_EASYLOGGINGPP

#define NANOSVG_IMPLEMENTATION
#include "nanosvg.h"


std::map<std::string, std::string>
get_input_paths(int argc, char** argv, const cv::String & cmd_keys){
    cv::CommandLineParser parser(argc, argv, cmd_keys);

    if(parser.has("help")){
        parser.printMessage();
        exit(1);
    }

    if(!parser.check()){
        parser.printErrors();
        exit(1);
    }

    std::string image_path = parser.get<std::string>("image");
    std::string camera_path = parser.get<std::string>("camera");
    std::string marker_path = parser.get<std::string>("marker");

    LOG(INFO) << "image is: " <<  image_path;
    LOG(INFO) << "camera is: " << camera_path;
    LOG(INFO) << "marker is: " << marker_path;

    if(image_path.empty() || camera_path.empty() || marker_path.empty()){
        LOG(ERROR) << "All paths mus be presented";
        parser.printMessage();
        exit(1);
    }

    std::map<std::string, std::string> work_paths;
    work_paths["image"] = image_path;
    work_paths["camera"] = camera_path;
    work_paths["marker"] = marker_path;

    return work_paths;
}


cv::Mat read_image(std::string & image_path){
   cv::Mat loaded = cv::imread(image_path);
   if(loaded.data != NULL){
       return loaded;
   }

   LOG(ERROR) << "can\'t load image file - " << image_path;
   exit(-1);
}

cv::Mat read_svg_marker(std::string & marker_path){
    struct NSVGimage* image;
    image = nsvgParseFromFile(marker_path.c_str(), "px", 96);

    if(image == NULL){
        LOG(ERROR) << "can't read svg image marker";
        exit(1);
    }

    // +1 because patterns size is image.w and image.h, we want to store real size of pattern
    // threfore container size for it must be greater a little bit
    cv::Mat mat_from_svg(image->height + 1, image->width + 1, CV_8UC1, cv::Scalar(255, 255, 255));

    std::vector<std::vector<cv::Point>> segments;

    // inspect svg image. Look at all paths in it. Get only start and end points.
    for (auto shape = image->shapes; shape != NULL; shape = shape->next) {
        std::vector<cv::Point> segment;
        for (auto path = shape->paths; path != NULL; path = path->next) {
            bool drop = false;
            for (int i = 0; i < path->npts-1; i += 3, drop =! drop) {
                if(drop) continue; // remove points dublicates
                float* p = &path->pts[i*2];
                cv::Point start_line((int)p[0], (int)p[1]);
                cv::Point end_line((int)p[6], (int)p[7]);
                segment.push_back(start_line);
                segment.push_back(end_line);

                LOG(DEBUG) << segment[0] << " " << segment[1];
                LOG(DEBUG) << p[0] << " " << p[1] << " " << p[6] << " " << p[7];
                LOG(DEBUG) << p[0] << " " << p[1] << " " << p[2] << " "<< p[3] << " " << p[4] << " " << p[5] << " " << p[6] << " " << p[7];
            }

        }
        segments.push_back(segment);
    }

    nsvgDelete(image);

    // draw pattern on mat
    for(auto & s: segments){
        cv::fillConvexPoly(mat_from_svg, s.data(), s.size(), cv::Scalar(0, 0, 0));
    }

    return mat_from_svg;
}


void display_image(const std::string && win_name, const cv::Mat & image, int w, int h){
    cv::namedWindow(win_name, cv::WINDOW_NORMAL);
    cv::resizeWindow(win_name, w, h);
    cv::imshow(win_name, image);
    cv::waitKey(-1);
    cv::destroyWindow(win_name);
}


std::map<std::string, cv::Mat> read_camera_intrinsics(const std::string & camera_file){
    cv::FileStorage storage;
    if(!storage.open(camera_file, cv::FileStorage::READ)){
        LOG(ERROR) << "Can't open json file with camera calibration settings";
        exit(1);
    }

    std::map<std::string, cv::Mat> settings;
    settings["matrix"] = cv::Mat(3, 3, CV_64F, cv::Scalar(-1.0));
    settings["distortion"] = cv::Mat(8, 1, CV_64F, cv::Scalar(-1.0));

    auto intrisics = storage["intrinsics"];

    //read K - camera matrix
    for(int i = 0, K_inex = 0; i < settings["matrix"].cols; i++){
        for(int j = 0; j < settings["matrix"].rows; j++, K_inex++){
            settings["matrix"].at<double>(i, j) = intrisics["K"][K_inex];
        }
    }

    //read distrtion vector
    for(int i = 0; i < settings["distortion"].rows; i++){
        settings["distortion"].at<double>(i, 0) = intrisics["distortion"][i];
    }

    return settings;
}


int main(int argc, char** argv )
{
    const cv::String cmd_keys =
            "{help  h	|| calculate rotaton and translation of marker on image	}"
            "{image i 	|| path to image which contains pattern					}"
            "{camera c	|| path to file which contains camera matrix json		}"
            "{marker m 	|| path to image which is marker						}";

    auto work_paths = get_input_paths(argc, argv, cmd_keys);

    auto work_image = read_image(work_paths["image"]);
    display_image("work img", work_image, 800, 600);

    auto work_marker = read_svg_marker(work_paths["marker"]);
    display_image("marker", work_marker, 100, 150);

    auto camera_settings = read_camera_intrinsics(work_paths["camera"]);
    LOG(INFO) << "camera matrix: " << camera_settings["matrix"] << ", distortion: " << camera_settings["distortion"];


    return 0;
}
