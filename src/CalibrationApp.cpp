/**
 * @file    CalibrationApp.cpp
 *
 * @author  btran
 *
 */

#include "AppUtility.hpp"
#include "CalibrationHandler.hpp"

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: [app] [path/to/json/config]" << std::endl;
        return EXIT_FAILURE;
    }

    std::string CONFIG_PATH = argv[1];
    _cv::CalibrationHandler::Param param = _cv::getCalibrationHandlerParam(CONFIG_PATH);

    std::cout << "\nloading images and detecting checkboard's corners... \n";
    _cv::CalibrationHandler calibHandler(param);
    if (calibHandler.allImagePoints().empty()) {
        std::cerr << "failed to get enough image points" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "\noptimizing camera parameters...\n";

    cv::Mat cameraMatrix, distortionParams;
    double xi;
    std::vector<cv::Vec3d> extrinsicRVecs, extrinsicTVecs;
    double rms = calibHandler.run(cameraMatrix, distortionParams, xi, extrinsicRVecs, extrinsicTVecs);

    std::cout << "\ncamera matrix: " << cameraMatrix << "\n";
    std::cout << "\ndistortion params (k1,k2,p1,p2): " << distortionParams << "\n";
    std::cout << "\nxi (for CMei's model): " << xi << "\n";
    std::cout << "\nreporjection error: " << rms << "\n";

    calibHandler.drawChessboardCornersAllImages();
    calibHandler.drawFrameAxesAllImages(cameraMatrix, distortionParams, xi, extrinsicRVecs, extrinsicTVecs);

    cv::Affine3d camPose;
    double scale = 0.1;
    auto cameraWindow = ::initializeWindowWithCamera(camPose, cv::Matx33d(cameraMatrix), scale);
    auto colors = _cv::utils::generateColorCharts(extrinsicRVecs.size());

    for (std::size_t i = 0; i < extrinsicRVecs.size(); ++i) {
        cv::Mat rotationMatrix;
        cv::Rodrigues(extrinsicRVecs[i], rotationMatrix);
        cv::Affine3d boardPose(rotationMatrix, extrinsicTVecs[i]);
        cv::Affine3d boardPoseInWorld = camPose * boardPose;
        cv::Mat rotation(boardPoseInWorld.rotation());
        cv::Vec3d translation(boardPoseInWorld.translation());
        cv::Vec3d xAxis(rotation.col(0));
        cv::Vec3d yAxis(rotation.col(1));
        cv::Vec3d zAxis(rotation.col(2));

        // clang-format off
        cv::Point3d center = translation +
                             xAxis * calibHandler.param().numCol * calibHandler.param().squareSize / 2 +
                             yAxis * calibHandler.param().numRow * calibHandler.param().squareSize / 2;
        // clang-format on

        cameraWindow.showWidget("board" + std::to_string(i),
                                cv::viz::WPlane(cv::Point3d(center), zAxis, yAxis, cv::Size2d(0.2, 0.2), colors[i]));
        cameraWindow.showWidget("text" + std::to_string(i),
                                cv::viz::WText3D(std::to_string(i), cv::Point3d(translation), 0.01, false, colors[i]));
    }

    cameraWindow.spin();

    return EXIT_SUCCESS;
}
