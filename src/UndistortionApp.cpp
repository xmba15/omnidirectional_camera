/**
 * @file    UndistortionApp.cpp
 *
 * @author  btran
 *
 */

#include "CalibrationHandler.hpp"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: [app] [path/to/sample/image]" << std::endl;
        return EXIT_FAILURE;
    }

    std::string IMAGE_PATH = argv[1];
    cv::Mat sampleImage = cv::imread(IMAGE_PATH);
    if (sampleImage.empty()) {
        std::cerr << "failed to load: " << IMAGE_PATH << std::endl;
        return EXIT_FAILURE;
    }

    // camera matrix and distortion params obtained from calibration app
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 366.0786746575819, -1.730277473341805, 643.7193980161953, 0,
                            366.1677891988819, 521.3166467512945, 0, 0, 1);
    cv::Mat distortionParams = (cv::Mat_<double>(1, 4) << 0.1097909284122646, 0.216601395090228, 0.0008254647531474698,
                                -0.0002583633785147333);
    double xi = 1.44309762366835;

    cv::Mat undistorted;
    int rectificationType = cv::omnidir::RECTIFY_CYLINDRICAL;

    cv::omnidir::undistortImage(sampleImage, undistorted, cameraMatrix, distortionParams, xi, rectificationType);

    cv::imshow("undistored image", undistorted);
    cv::waitKey(0);
    cv::destroyAllWindows();

    return 0;
}
