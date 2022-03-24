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
        cv::imshow(wname.c_str(), calib_imgs[i]);
        cv::waitKey(100);
        cv::destroyWindow(wname);
    }
    cv::Mat test_img = cv::imread("./example_calib/Test_Color");
    cv::imshow("Test image", test_img);
    cv::waitKey(0);
    cv::destroyWindow("Test image");

    std::ifstream f("./example_calib/RobotPose.txt");
    std::string cur_line;
    
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
            std::cout << "Current pose " << count << ": " << std::endl;
            printf("x=%f, y=%f, z=%f, rx=%f, ry=%f, rz=%f.\n", rp.x, rp.y, rp.z, rp.rx, rp.ry, rp.rz);
        }
    }
    cv::Mat output_centers;
    cv::findCirclesGrid(test_img, cv::Size(7, 7), output_centers);
    std::cout << "rows: " << output_centers.rows << ", cols: " << output_centers.cols << std::endl;
    std::vector<cv::Point2f> circle_centers;
    for (int i = 0; i < output_centers.rows; i++) {
        circle_centers.push_back(output_centers.at<cv::Point2f>(0, i));
        cv::circle(test_img, circle_centers[i], 10, cv::Scalar(0, 0, 255), 5, cv::FILLED);
        cv::putText(test_img, std::to_string(i), circle_centers[i], cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 255));
    }

    cv::imshow("Circle centers", test_img);
    cv::waitKey(0);

    cv::Mat camera_intrinsics, distortion;
    std::vector<cv::Mat> rvecs, tvecs;
    int success_code = calibrateIntrinsics(Pattern::CIRCLES_GRID, calib_imgs, cv::Size(7, 7), 25, camera_intrinsics, distortion, rvecs, tvecs);
    
    std::cout << "The camera matrix is " << camera_intrinsics << std::endl;
    std::cout << "The distortion coeff is " << distortion << std::endl;

    cv::Mat undistorted_img;
    cv::undistort(test_img, undistorted_img, camera_intrinsics, distortion);
    cv::imshow("undistorted image", undistorted_img);
    cv::waitKey(0);


    /* Examine rvecs and tvecs here */
    std::cout << "rvecs dimensions: " << rvecs[0].size() << std::endl;
    std::cout << "tvecs dimensions: " << tvecs[0].size() << std::endl;
    std::cout << "The rotation matrices are: " << std::endl;
    for (int i{0}; i < rvecs.size(); i++) {
        std::string msg{"rotation of " + std::to_string(i) + " pose: "};
        std::cout << msg << std::endl;
        cv::Mat rot;
        cv::Rodrigues(rvecs[i], rot);
        std::cout << rot << std::endl;
        cv::Mat rot_t;
        cv::transpose(rot, rot_t);
        std::cout << "Is rotation Matrix: " << std::boolalpha << (cv::norm(cv::Mat::eye(3, 3, rot.type()), rot * rot_t) < 1e-6) << std::endl;
    }

    cv::Mat rvector;
    cv::Mat tvector;
    findBoardPose(cv::Size(7, 7), 25, output_centers, camera_intrinsics, distortion, rvector, tvector);


    // draw the circle grids
    return EXIT_SUCCESS;
}
