#include <map>
#include <string>
#include <vector>
#include <limits>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

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

    int padding = 10;
    // +padding because patterns size is image.w and image.h, we want to store real size of pattern
    // threfore container size for it must be greater a little bit
    cv::Mat mat_from_svg(image->height + padding, image->width + padding, CV_8UC1, cv::Scalar(255, 255, 255));

    // on this value we shift marker in his Mat storage
    int shift = 4;

    std::vector<std::vector<cv::Point>> segments;

    // inspect svg image. Look at all paths in it. Get only start and end points.
    for (auto shape = image->shapes; shape != NULL; shape = shape->next) {
        std::vector<cv::Point> segment;
        for (auto path = shape->paths; path != NULL; path = path->next) {
            bool drop = false;
            for (int i = 0; i < path->npts-1; i += 3, drop =! drop) {
                if(drop) continue; // remove points dublicates
                float* p = &path->pts[i*2];
                cv::Point start_line((int)p[0] + shift, (int)p[1] + shift);
                cv::Point end_line((int)p[6] + shift, (int)p[7] + shift);
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


void display_image(const std::string && win_name, const cv::Mat & image, int w=800, int h=600){
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


///
/// \brief extract_contours
/// extract contours either work image or marker image
/// from marker we wan to retrieve only one contour - inner contour
/// from image all contours
/// moreover out marker not requires binarisation by default
/// \param image
/// \param make_bin_thresh
/// \param extract_only_inner
/// \return
///
std::vector<std::vector<cv::Point>>
extract_contours(
    const cv::Mat & image,
    bool make_bin_thresh=false,
    bool extract_only_inner=false
){
    cv::Mat tmp_image = image.clone();
    if(make_bin_thresh){
        cv::cvtColor(tmp_image, tmp_image, cv::COLOR_BGR2GRAY);
        cv::threshold(tmp_image, tmp_image, 127, 255, cv::THRESH_BINARY);
    }

    // search enges and contour on marker
    cv::Mat edges;
    cv::Canny(tmp_image, edges, 127, 255, 3);
//    display_image("edges", edges);

    std::vector<std::vector<cv::Point>> contours;

    cv::RetrievalModes extract_mode;
    if(extract_only_inner){
        extract_mode = cv::RETR_CCOMP;
    }
    else
        extract_mode = cv::RETR_LIST;

    cv::findContours(tmp_image, contours, extract_mode, cv::CHAIN_APPROX_SIMPLE);

    // I want leave only strainght forward lines. Drop big part of contour
    int approximation_power = 10;
    for(auto & s: contours){
        cv::approxPolyDP(s, s, approximation_power, true);
    }

    if(extract_only_inner){
        std::vector<std::vector<cv::Point>> internal_contours;
        internal_contours.push_back(contours.back());
        return internal_contours;
    }

    return contours;
}

/**
 * @brief calculate_adjacent_segments_indexes
 * for counter clock wise oriented contour
 * generate indexes of adjacent segments
 * @param contour_size
 * @return
 */
std::vector<std::vector<int>>
calculate_adjacent_segments_indexes(int contour_size){

    int two_pair_size = 4;
    std::vector<std::vector<int>> adjacent;
    for(int i = 0; i < contour_size - 2; i++){
        std::vector<int> adj;
        adj.reserve(two_pair_size);
        adj.push_back(i);
        adj.push_back(i + 1);
        adj.push_back(i + 1);
        adj.push_back(i + 2);

        adjacent.push_back(adj);
    }

    //close indexes chain
    std::vector<int> last_adj;
    last_adj.reserve(two_pair_size);
    last_adj.push_back(adjacent.back()[2]);
    last_adj.push_back(adjacent.back()[3]);
    last_adj.push_back(adjacent.back()[3]);
    last_adj.push_back(0); // because all contours is closed
    adjacent.push_back(last_adj);


    return adjacent;
}


std::map<std::pair<int, int>, std::vector<std::pair<int, int>>>
calculate_all_non_adjance_combination_segments_indexes(int contour_size){
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> res;

    // create all combinations of segments
    std::vector<std::pair<int, int>> all_segments;
    all_segments.reserve(contour_size);
    for(int i = 0; i + 1 < contour_size; i++){
        all_segments.push_back(std::pair<int, int>(i, i+1));
    }
    std::pair<int, int> last_pair = all_segments.back();
    all_segments.push_back(std::pair<int, int>(last_pair.second, 0));

    // match each segments to all not adjacent segments
    for(int i = 0; i < all_segments.size(); i++){
        res[all_segments[i]] = std::vector<std::pair<int, int>>();
        for(int j = 0; j < all_segments.size(); j++){
            bool self_cond = i == j;
            bool right_cond = (i + 1) == j;
            bool left_cond = (i - 1) == j;
            bool left_border_cond = ((i - 1) < 0) && ((j + 1) == all_segments.size());
            bool right_border_cond = ((i + 1) == all_segments.size()) && (j == 0);

            if(self_cond || right_cond || left_cond || left_border_cond || right_border_cond) continue;

            res[all_segments[i]].push_back(all_segments[j]);
        }
    }

//    for(const auto & p: res){
//        auto key = p.first;
//        auto val = p.second;
//        std::cout << " pair " << std::get<0>(key) << " " << std::get<1>(key) << "\n";
//        for(auto i: val){
//                std::cout << " " << std::get<0>(i) << " " << std::get<1>(i) << "\n";
//            }
//    }
    return res;
}


/**
 * @brief cos_between_vectors
 * vectors must be normalized
 * caller must perform substraction vectors toe from heel
 * @param a
 * @param b
 * @return
 */
double cos_between_vectors(const cv::Point & a, const cv::Point & b){
    double ans = 0;
    ans = double(a.dot(b)) / (cv::norm(a) * cv::norm(b));
    return ans;
}


/**
 * @brief has_two_parallel_line
 * search two parallel segment in all not adjacent segments of contour
 * using fact about cos between two parallel vectors is 1.0
 * @param contour
 * @param delta deviation interval from cos == 1.0
 * @return
 */
bool has_two_parallel_line(const std::vector<cv::Point> & contour, double delta=0.1){
    auto indexes = calculate_all_non_adjance_combination_segments_indexes(contour.size());

    for(const auto & anhor_not_adjacent: indexes){
        auto anchor = anhor_not_adjacent.first;
        for(const auto & not_adjacent: anhor_not_adjacent.second){
            cv::Point a = contour[anchor.first] - contour[anchor.second];
            cv::Point b = contour[not_adjacent.first] - contour[not_adjacent.second];
            double cos_ab = cos_between_vectors(a, b);
            cos_ab = cv::abs(cos_ab);
            bool cos_is_in_confidence_interval = cos_ab < (1 + delta) && cos_ab > (1 - delta);
            LOG(DEBUG) << cos_ab;
            if(cos_is_in_confidence_interval){
                return true;
            }
        }
    }

    return false;
}


/**
 * @brief search_best_contours_match
 * our marker is five corner convex house...
 * at further position it has area near 10000
 * It is plased on white A4 paper.
 * Houses walls parallel to longest papers side
 * House have blask filing color on whole area
 * House have 2 parallel lines - left and right walls
 * this function search contour which have best match with condition above
 * @param image_contours
 * @param marker_contour
 * @return index of best corner in image_corners
 * - image_corners[ret_val] is best contour
 */
int search_best_contours_match(
    const std::vector<std::vector<cv::Point>> & image_contours,
    const std::vector<cv::Point> & marker_contour
){
    int best_index = -1;
    double match_score_max = 1000;//std::numeric_limits<double>::max();
    double match_score_cur = 0;
    int min_area_of_contour = 5000; // In test images further object has area 10 000
    int exactly_marker_points_count = 5; // our marker has 5 corners

    for(int i = 0; i < image_contours.size(); i++){
        if(cv::contourArea(image_contours[i]) < min_area_of_contour) continue;
        if(!cv::isContourConvex(image_contours[i])) continue;
        if(image_contours[i].size() != exactly_marker_points_count) continue;
        if(!has_two_parallel_line(image_contours[i])) continue;

        best_index = i;
        break;
    }

    return best_index;
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
//    display_image("work img", work_image, 800, 600);

    auto work_marker = read_svg_marker(work_paths["marker"]);
//    display_image("marker", work_marker, 100, 150);

    auto camera_settings = read_camera_intrinsics(work_paths["camera"]);
//    LOG(INFO) << "camera matrix: " << camera_settings["matrix"] << ", distortion: " << camera_settings["distortion"];

    auto marker_contours = extract_contours(work_marker, false, true);
    auto image_contours = extract_contours(work_image, true, false);

    LOG(DEBUG) << cv::isContourConvex(marker_contours[0]);
    int best_contour_match_index = search_best_contours_match(image_contours, marker_contours[0]);
    if(best_contour_match_index == -1){
        LOG(ERROR) << "Can't find contour matching in image";
        exit(0);
    }

    cv::drawContours(work_image, image_contours, best_contour_match_index, cv::Scalar(0, 127, 255), 5);
    display_image("best contour", work_image);
    LOG(DEBUG) << image_contours[best_contour_match_index];
















    return 0;
}






















