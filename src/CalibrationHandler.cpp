/**
 * @file    CalibrationHandler.cpp
 *
 * @author  btran
 *
 */

#include "CalibrationHandler.hpp"
#include "BasicUtils.hpp"

namespace _cv
{
CalibrationHandler::CalibrationHandler(const Param& param)
    : m_param(param)
    , m_patternSize(m_param.numCol, m_param.numRow)
{
    validate<Param>(param);
    m_imageList = parseMetaDataFile(param.imageListFile);
    m_allImagePoints = this->getImagePoints();
    m_objectPoints = this->generateObjectPoints(m_param.numRow, m_param.numCol, m_param.squareSize);
}

double CalibrationHandler::run(cv::Mat& cameraMatrix, cv::Mat& distortionParams, double& xi,
                               std::vector<cv::Vec3d>& extrinsicRVecs, std::vector<cv::Vec3d>& extrinsicTVecs) const
{
    const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 0.0001);
    cv::Mat _xi;
    cv::Mat usedImageIndices;

    double rms = cv::omnidir::calibrate(std::vector<std::vector<cv::Vec3f>>(m_allImagePoints.size(), m_objectPoints),
                                        m_allImagePoints, m_imgSize, cameraMatrix, _xi, distortionParams,
                                        extrinsicRVecs, extrinsicTVecs, 0, criteria, usedImageIndices);
    xi = _xi.at<double>(0);

    int usedImageNum = extrinsicRVecs.size();
    std::vector<std::string> newImageList;
    std::vector<std::vector<cv::Vec2f>> newAllImagePoints;
    for (int i = 0; i < usedImageNum; ++i) {
        newImageList.emplace_back(std::move(m_imageList[usedImageIndices.at<int>(i)]));
        newAllImagePoints.emplace_back(std::move(m_allImagePoints[usedImageIndices.at<int>(i)]));
    }
    m_imageList = std::move(newImageList);
    m_allImagePoints = std::move(newAllImagePoints);

    return rms;
}

void CalibrationHandler::drawFrameAxesAllImages(const cv::Mat& cameraMatrix, const cv::Mat& distortionParams, double xi,
                                                const std::vector<cv::Vec3d>& extrinsicRVecs,
                                                const std::vector<cv::Vec3d>& extrinsicTVecs, float length,
                                                int thickness) const
{
    std::string outputPath = "/tmp";
    for (std::size_t i = 0; i < m_imageList.size(); ++i) {
        cv::Mat image = cv::imread(m_param.imagePath + "/" + m_imageList[i]);
        this->drawFrameAxes(image, cameraMatrix, distortionParams, xi, extrinsicRVecs[i], extrinsicTVecs[i], length,
                            thickness);
        cv::imwrite(outputPath + "/" + splitByDelim(m_imageList[i], '.').front() + "_pose.jpg", image);
    }
}

void CalibrationHandler::drawFrameAxes(cv::Mat& img, const cv::Mat& cameraMatrix, const cv::Mat& distortionParams,
                                       double xi, const cv::Vec3d& extrinsicRVec, const cv::Vec3d& extrinsicTVec,
                                       float length, int thickness) const
{
    std::vector<cv::Point3f> axesPoints;
    axesPoints.push_back(cv::Point3f(0, 0, 0));
    axesPoints.push_back(cv::Point3f(length, 0, 0));
    axesPoints.push_back(cv::Point3f(0, length, 0));
    axesPoints.push_back(cv::Point3f(0, 0, length));

    std::vector<cv::Point2f> imagePoints;
    cv::omnidir::projectPoints(axesPoints, imagePoints, extrinsicRVec, extrinsicTVec, cameraMatrix, xi,
                               distortionParams);
    cv::line(img, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), thickness);
    cv::line(img, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), thickness);
    cv::line(img, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), thickness);
}

void CalibrationHandler::drawChessboardCornersAllImages() const
{
    std::string outputPath = "/tmp";
    for (std::size_t i = 0; i < m_imageList.size(); ++i) {
        cv::Mat image = cv::imread(m_param.imagePath + "/" + m_imageList[i]);
        cv::drawChessboardCorners(image, m_patternSize, cv::Mat(m_allImagePoints[i]), true);
        cv::imwrite(outputPath + "/" + splitByDelim(m_imageList[i], '.').front() + ".jpg", image);
    }
}

std::vector<std::vector<cv::Vec2f>> CalibrationHandler::getImagePoints()
{
    std::vector<std::vector<cv::Vec2f>> allImagePoints;
    for (auto it = m_imageList.begin(); it != m_imageList.end();) {
        cv::Mat gray = cv::imread(m_param.imagePath + "/" + *it, 0);

        if (gray.empty()) {
            throw std::runtime_error("failed to read " + *it);
        }

        if (m_imgSize.width * m_imgSize.height == 0) {
            m_imgSize = gray.size();
        }

        std::vector<cv::Vec2f> points;
        const cv::Size subPixWinSize(5, 5);
        const cv::TermCriteria termimateCrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 0.00001);

        bool found = cv::findChessboardCorners(gray, m_patternSize, points);

        if (!found) {
            std::cerr << "failed to find corners on image " + *it << std::endl;
            it = m_imageList.erase(it);
            continue;
        }

        cv::cornerSubPix(gray, points, subPixWinSize, cv::Size(-1, -1), termimateCrit);
        allImagePoints.emplace_back(std::move(points));

        ++it;
    }

    return allImagePoints;
}

std::vector<cv::Vec3f> CalibrationHandler::generateObjectPoints(int numRow, int numCol, float squareSize) const
{
    std::vector<cv::Vec3f> objectPoints;
    objectPoints.reserve(numRow * numCol);

    for (int i = 0; i < numRow; ++i) {
        for (int j = 0; j < numCol; ++j) {
            objectPoints.emplace_back(cv::Vec3f(j * squareSize, i * squareSize, 0.));
        }
    }

    return objectPoints;
}

CalibrationHandler::Param getCalibrationHandlerParam(const std::string& jsonPath)
{
    rapidjson::Document jsonDoc = readFromJsonFile(jsonPath);
    CalibrationHandler::Param param;
    param.imagePath = getValueAs<std::string>(jsonDoc, "image_path");
    param.imageListFile = getValueAs<std::string>(jsonDoc, "image_list_file");
    param.numRow = getValueAs<int>(jsonDoc, "num_row");
    param.numCol = getValueAs<int>(jsonDoc, "num_col");
    param.squareSize = getValueAs<float>(jsonDoc, "square_size");

    return param;
}

template <> void validate<CalibrationHandler::Param>(const CalibrationHandler::Param& param)
{
    if (param.imagePath.empty()) {
        throw std::runtime_error("empty path to images");
    }

    if (param.imageListFile.empty()) {
        throw std::runtime_error("empty file that stores image list");
    }

    if (param.numRow <= 0) {
        throw std::runtime_error("invalid number of rows");
    }

    if (param.numCol <= 0) {
        throw std::runtime_error("invalid number of columns");
    }

    if (param.squareSize <= 0) {
        throw std::runtime_error("invalid square size");
    }
}
}  // namespace _cv
