#include <iostream>
#include <fstream>
#include <chrono>
#include "opencv2/opencv.hpp"
#include "src/file_manip.hpp"
#include "src/calib_intrinsics.hpp"
#include <time.h>

#define deg2rad(d)(d * M_PI / 180.0)

int main(int argc, char** argv) {
    /* Read in the images */
    std::vector<std::string> img_names = getAllFileName("./data/eth/", ".png");
    std::cout << img_names.size() << std::endl;
    std::sort(img_names.begin(), img_names.end(), [](std::string name1, std::string name2)->bool {
        int idx1 = atoi(name1.substr(0, name1.find_first_of('_')).c_str());
        int idx2 = atoi(name2.substr(0, name2.find_first_of('_')).c_str());
        return idx1 < idx2;
    });
    std::vector<cv::Mat> input_imgs;
    // load and read in the images
    for (int i = 0; i < img_names.size(); i++) {
        std::string img_path = "./data/eth/" + img_names[i];
        cv::Mat img = cv::imread(img_path);
        input_imgs.push_back(img);
    }

    /* Calculate the feature points */
    std::vector<std::vector<cv::Point2f>> centers_group;
    for (int i = 0; i < input_imgs.size(); i++) {
        std::vector<cv::Point2f> traced_centers;
        std::cout << "Corners for image " << i << std::endl;
        auto start = std::chrono::steady_clock::now();
        findAndDrawChessBoardCorners(input_imgs[i], traced_centers, cv::Size(9, 6), false);
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> dura = end - start;
        std::cout << "Time spent on finding corners for image " << i << " : " << dura.count() << " ms." << std::endl;
        centers_group.push_back(traced_centers);
    }

    /* Load in the camera intrinsics */
    cv::FileStorage fs("./data/eih/intrinsics.yml", cv::FileStorage::READ);
    cv::Mat intr, dist;
    fs["Intrinsic"] >> intr;
    fs["Distortion"] >> dist;
    std::cout << "The read in camera intrinsics are \n" << intr << std::endl;
    std::cout << "The read in distortion coeffs are \n" << dist << std::endl;
    fs.release();

    /* load in robot poses */


    return EXIT_SUCCESS;

}