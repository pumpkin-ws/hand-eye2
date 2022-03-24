#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "src/file_manip.hpp"
#include "src/calib_intrinsics.hpp"

/* The units of xyz are in meters, the units of rx,ry,rz are in radians. */
// struct RobotPose{
//     double x{0}, y{0}, z{0}, rx{0}, ry{0}, rz{0};
// };

#define deg2rad(d) (d * M_PI / 180.0)

int main(int argc, char** argv) {

    /* Read in the images */
    std::vector<std::string> img_names = getAllFileName("./example_calib/", ".png");
    std::cout << img_names.size() << std::endl;
    printf("The loaded image directory file names are: \n");
    std::sort(img_names.begin(), img_names.end(), [](std::string name1, std::string name2)->bool {
        int idx1 = atoi(name1.substr(0, name1.find_first_of('_')).c_str());
        int idx2 = atoi(name2.substr(0, name2.find_first_of('_')).c_str());
        return idx1 < idx2;

    });

    std::vector<cv::Mat> calib_imgs;
    for (int i = 0; i < img_names.size(); i++) {
        calib_imgs.push_back(cv::imread("./example_calib/" + img_names[i]));
        std::string wname{std::to_string(i) + " calib"};
        // cv::imshow(wname.c_str(), calib_imgs[i]);
        // cv::waitKey(100);
        // cv::destroyWindow(wname);
    }

    /* Read in the robot poses */
    std::ifstream f("./example_calib/RobotPose.txt");
    std::string cur_line;
    /**
     * @brief The output pose is already in radian
     * 
     */
    auto str2pose = [](std::string line)mutable->RobotPose {
        RobotPose rp;
        double x = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());
        double y = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());
        double z = atof(line.substr(0, line.find_first_of(',')).c_str());
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double rx = deg2rad(
            atof(line.substr(0, line.find_first_of(',')).c_str())
        );
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double ry = deg2rad(
            atof(line.substr(0, line.find_first_of(',')).c_str())
        );
        line = line.substr(line.find_first_of(',') + 1, line.size());   
        double rz = deg2rad(
            atof(line.substr(0, line.find_first_of(',')).c_str())
        );
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
            // std::cout << "Current pose " << count << ": " << std::endl;
            // printf("x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f.\n", rp.x, rp.y, rp.z, rp.rx, rp.ry, rp.rz);
        }
    }

    /* Calculate and get the camera intrinscis */
    cv::Mat camera_intrinsics, distortion;
    std::vector<cv::Mat> rvecs, tvecs;
    int success_code = calibrateIntrinsics(Pattern::CIRCLES_GRID, calib_imgs, cv::Size(7, 7), 25, camera_intrinsics, distortion, rvecs, tvecs);

    std::cout << "The camera matrix is: \n";
    std::cout << camera_intrinsics << std::endl;
    printf("The distortion coeffients are:\n");
    std::cout << distortion << std::endl;

    /* Find and save the chessboard corners for each image */
    std::vector<cv::Mat> corner_list;
    std::vector<cv::Mat> rvec_list;
    std::vector<cv::Mat> tvec_list;

    for (int i = 0; i < calib_imgs.size(); i++) {
        cv::Mat corners;
        cv::Mat rvector, tvector;
        cv::findCirclesGrid(calib_imgs[i], cv::Size(7, 7), corners);
        findBoardPose(cv::Size(7, 7), 0.025, corners, camera_intrinsics, distortion, rvector, tvector);
        std::cout << rvector.cols << ", " << rvector.rows << std::endl;
        corner_list.push_back(corners);
        rvec_list.push_back(rvector);
        tvec_list.push_back(tvector);
    }

    cv::Mat R_cam2gripper, t_cam2gripper;
    calibrateHandInEye(poses, rvec_list, tvec_list, R_cam2gripper, t_cam2gripper);
    std::cout << "The rotation of cam2gripper: \n";
    std::cout << R_cam2gripper << std::endl;
    std::cout << "The translation of cam2gripper: \n";
    std::cout << t_cam2gripper << std::endl;


    return EXIT_SUCCESS;

}