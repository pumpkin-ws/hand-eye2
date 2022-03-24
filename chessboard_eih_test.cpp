#include <iostream>
#include <fstream>
#include <chrono>
#include "opencv2/opencv.hpp"
#include "src/file_manip.hpp"
#include "src/calib_intrinsics.hpp"
#include <time.h>

#define deg2rad(d)(d * M_PI / 180.0)

int main(int argc, char** argv) {
    /* Read in the images and robot poses */
    std::vector<std::string> img_names = getAllFileName("./data/eih/", ".png");
    std::cout << img_names.size() << std::endl;
    std::sort(img_names.begin(), img_names.end(), [](std::string name1, std::string name2)->bool {
        int idx1 = atoi(name1.substr(0, name1.find_first_of('_')).c_str());
        int idx2 = atoi(name2.substr(0, name2.find_first_of('_')).c_str());
        return idx1 < idx2;
    });
    std::vector<cv::Mat> input_imgs;
    // load and read in the images
    for (int i = 0; i < img_names.size(); i++) {
        std::string img_path = "./data/eih/" + img_names[i];
        cv::Mat img = cv::imread(img_path);
        input_imgs.push_back(img);
    }
    
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

    // find the camera intrinsics
    cv::Mat intrinsics, distortion;
    std::vector<cv::Mat> rvecs, tvecs;
    calibrateIntrinsics(Pattern::CHESSBOARD, input_imgs, cv::Size(9, 6), 18, intrinsics, distortion, rvecs, tvecs);

    std::cout << "The camera intrinsics are: " << std::endl;
    std::cout << intrinsics << std::endl;

    std::cout << "The distortion coeffs are: " << std::endl;
    std::cout << distortion << std::endl;

    // save the camera intrinsics somewhere
    cv::FileStorage fs("./data/eih/intrinsics.yml", cv::FileStorage::WRITE);
    time_t rawtime;
    time(&rawtime);
    fs << "CalibrateDate" << asctime(localtime(&rawtime));
    fs << "Intrinsic" << intrinsics;
    fs << "Distortion" << distortion;
    fs << "ImageSize" << input_imgs[0].size();

    fs.release();

    cv::FileStorage fs_read("./data/eih/intrinsics.yml", cv::FileStorage::READ);
    cv::Mat intr, dist;
    fs_read["Intrinsic"] >> intr;
    fs_read["Distortion"] >> dist;
    std::cout << "The read in camera intrinsics are \n" << intr << std::endl;
    std::cout << "The read in distortion coeffs are \n" << dist << std::endl;
    fs_read.release();

    // start to perform hand eye calibration
    std::vector<std::vector<double>> target2camera;
    
    for (int i = 0; i < centers_group.size(); i++) {
        std::cout << "Pose for calibration image " << i << std::endl;
        cv::Mat board_rvec, board_tvec;
        findBoardPose(cv::Size(9, 6), 18, centers_group[i], intrinsics, distortion, board_rvec, board_tvec);
        std::cout << board_rvec.size() << std::endl;
        std::cout << board_tvec.size() << std::endl;
        cv::Vec3f r = board_rvec;
        cv::Vec3f t = board_tvec;
        target2camera.push_back(
            std::vector<double>{t[0], t[1], t[2], r[0], r[1], r[2]}
        );
    }

    /* Load in the robot poses here */
    std::ifstream f("./data/eth/location.txt");
    std::string cur_line;
    
    auto str2pose = [](std::string line)mutable->RobotPose {
        RobotPose rp;
        double x = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());
        double y = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());
        double z = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double rx = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double ry = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double rz = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        
        rp.x = x;
        rp.y = y;
        rp.z = z;
        rp.rx = rx;
        rp.ry = ry;
        rp.rz = rz;

        return rp;
    };
    std::vector<RobotPose> poses;
    int count = 0;
    while (getline(f, cur_line)) {
        if (cur_line[0] != '\0') {
            RobotPose rp = str2pose(cur_line);
            poses.push_back(rp);
            count++;
            std::cout << "Current pose " << count << ": " << std::endl;
            printf("x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f.\n", rp.x, rp.y, rp.z, rp.rx, rp.ry, rp.rz);
        }
    }
    std::vector<std::vector<double>> gripper2base;
    for (int i = 0; i < poses.size(); i++) {
        gripper2base.push_back(
            std::vector<double>{
                poses[i].x, poses[i].y, poses[i].z, poses[i].rx, poses[i].ry, poses[i].rz
            }
        );
    }

    cv::Mat R_camera2gripper, t_camera2gripper; 
    performEyeHandCalib(gripper2base, target2camera, "xyz", R_camera2gripper, t_camera2gripper);

    std::cout << "The camera to gripper rotation matrix is " << std::endl;
    std::cout << R_camera2gripper << std::endl;
    std::cout << "The camera to gripper translation matrix is " << std::endl;
    std::cout << t_camera2gripper << std::endl;

    // need to write the verification here
    EIHVerify(gripper2base, target2camera, R_camera2gripper, t_camera2gripper, "xyz");

    return EXIT_SUCCESS;
}