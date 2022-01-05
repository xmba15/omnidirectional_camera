/**
 * @file    CalibrationHandler.hpp
 *
 * @author  btran
 *
 */

#pragma once

#include <opencv2/ccalib/omnidir.hpp>
#include <opencv2/opencv.hpp>

#include "ParamConfig.hpp"

namespace _cv
{
class CalibrationHandler
{
 public:
    struct Param {
        std::string imagePath;      // path to the directory that stores images
        std::string imageListFile;  // file that stores image list
        int numRow;                 // num (inner) row of the checkerboard
        int numCol;                 // num (inner) column of the checkerboard
        float squareSize;
    };

 public:
    explicit CalibrationHandler(const Param& param);

    double run(cv::Mat& cameraMatrix, cv::Mat& distortionParams, double& xi, std::vector<cv::Vec3d>& extrinsicRVecs,
               std::vector<cv::Vec3d>& extrinsicTVecs) const;

    void drawFrameAxesAllImages(const cv::Mat& cameraMatrix, const cv::Mat& distortionParams, double xi,
                                const std::vector<cv::Vec3d>& extrinsicRVecs,
                                const std::vector<cv::Vec3d>& extrinsicTVecs, float length = 0.1,
                                int thickness = 2) const;

    void drawFrameAxes(cv::Mat& img, const cv::Mat& cameraMatrix, const cv::Mat& distortionParams, double xi,
                       const cv::Vec3d& extrinsicRVec, const cv::Vec3d& extrinsicTVec, float length = 0.05,
                       int thickness = 1) const;

    void drawChessboardCornersAllImages() const;

    const auto& allImagePoints() const
    {
        return m_allImagePoints;
    }

    const auto& param() const
    {
        return m_param;
    }

 private:
    std::vector<std::vector<cv::Vec2f>> getImagePoints();

    std::vector<cv::Vec3f> generateObjectPoints(int numRow, int numCol, float squareSize) const;

 private:
    Param m_param;
    cv::Size m_patternSize;
    cv::Size m_imgSize;

    mutable std::vector<std::string> m_imageList;
    mutable std::vector<std::vector<cv::Vec2f>> m_allImagePoints;
    std::vector<cv::Vec3f> m_objectPoints;
};

CalibrationHandler::Param getCalibrationHandlerParam(const std::string& jsonPath);
template <> void validate<CalibrationHandler::Param>(const CalibrationHandler::Param& param);
}  // namespace _cv
