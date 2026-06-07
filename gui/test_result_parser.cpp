#include "ResultParser.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void check(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "test_result_parser failed: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

void checkNear(double actual, double expected, double tolerance,
               const char* message) {
    check(std::abs(actual - expected) <= tolerance, message);
}

} // namespace

int main() {
    const QString log =
        QStringLiteral(
            "[IMAGE scan C:/tmp/frame 001.png]\n"
            "[LSC_STATUS camera ok=0 metric=1.25e-05]\n"
            "[LSC_STATUS plane ok=1 metric=-2.5E+1]\n"
            "[LSC_STATUS axis ok=1 metric=.125]\n"
            "[LSC_RESULT step1=-7.2 step2=+14.5 volume=2.5e+06 "
            "gt_volume=2500000 elapsed=3.2E-1]\n"
            "[LSC_DONE ok=0]\n");

    const PipelineResult result = ResultParser::parse(log);
    check(result.valid(), "completed result should be valid");
    check(!result.cameraOk, "camera failure status");
    check(result.planeOk, "plane success status");
    check(result.axisOk, "axis success status");
    check(!result.inspectionOk, "inspection failure status");
    checkNear(result.cameraRms, 1.25e-5, 1e-12, "scientific notation");
    checkNear(result.planeAngle, -25.0, 1e-12, "signed exponent");
    checkNear(result.axisAngle, 0.125, 1e-12, "leading decimal point");
    checkNear(result.measVolume, 2.5e6, 1e-6, "large exponent");
    checkNear(result.totalTime, 0.32, 1e-12, "elapsed exponent");
    check(result.imageList.size() == 1, "image marker count");
    check(result.imageList.front() == QStringLiteral("C:/tmp/frame 001.png"),
          "image path with spaces");
    check(ResultParser::parseProgress(
              QStringLiteral("[LSC_PROGRESS value=75]")) == 75,
          "explicit progress");

    const ImageDetailRecord detail = ResultParser::parseImageDetail(
        QStringLiteral(
            "[LSC_DETAIL]\tscan\t20\t扫描帧 20\tC:/raw frame.png\t"
            "C:/overlay frame.png\t灰度重心法\t激光点 123；位移 10.0 mm"));
    check(detail.valid(), "detail record should be valid");
    check(detail.category == QStringLiteral("scan"), "detail category");
    check(detail.index == 20, "detail index");
    check(detail.sourcePath == QStringLiteral("C:/raw frame.png"),
          "detail source path with spaces");
    check(detail.processedPath == QStringLiteral("C:/overlay frame.png"),
          "detail processed path with spaces");
    check(detail.summary.contains(QStringLiteral("123")), "detail summary");

    const PipelineResult successful = ResultParser::parse(
        QStringLiteral(
            "[LSC_DETAIL]\tcamera\t0\t相机图像\t"
            "C:/含中文/原图.png\tC:/含中文/处理图.png\t角点检测\t54 个角点\n"
            "[LSC_EVIDENCE ok=1 records=1 expected=2 available=2]\n"
            "[LSC_DONE ok=1]\n"));
    check(successful.inspectionOk, "complete evidence permits inspection pass");
    check(successful.evidenceReported, "evidence status reported");
    check(successful.evidenceOk, "evidence status success");
    check(successful.evidenceRecords == 1, "evidence record count");
    check(successful.imageDetails.size() == 1, "detail collected in result");
    check(successful.imageDetails.front().sourcePath.contains(
              QStringLiteral("含中文")),
          "UTF-8 evidence path");

    const PipelineResult missingEvidence = ResultParser::parse(
        QStringLiteral(
            "[LSC_EVIDENCE ok=0 records=1 expected=2 available=0]\n"
            "[LSC_DONE ok=1]\n"));
    check(!missingEvidence.inspectionOk,
          "missing evidence must override pipeline success");

    std::cout << "test_result_parser passed\n";
    return EXIT_SUCCESS;
}
