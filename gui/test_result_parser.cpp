#include "ResultParser.h"

#include <QFile>

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

QString validPipelineLog() {
    return QStringLiteral(
        "[LSC_PLAN camera=1 light_plane=1 motion_axis=1 "
        "scan_frames=1 scan_evidence=1 scan_stride=20]\n"
        "[IMAGE chessboard C:/run/output/camera.png]\n"
        "[LSC_DETAIL]\tcamera\t0\t相机图像\t"
        "C:/run/output/camera.png\tC:/run/output/camera_result.png\t"
        "角点检测\t54 个角点\n"
        "[LSC_STATUS camera ok=1 metric=0.25]\n"
        "[IMAGE laser_board C:/run/output/light.png]\n"
        "[LSC_DETAIL]\tlight_plane\t0\t光平面图像\t"
        "C:/run/output/light.png\tC:/run/output/light_result.png\t"
        "激光中心提取\t500 个中心点\n"
        "[LSC_STATUS plane ok=1 metric=0.50]\n"
        "[IMAGE motion C:/run/output/motion.png]\n"
        "[LSC_DETAIL]\tmotion_axis\t0\t移动轴图像\t"
        "C:/run/output/motion.png\tC:/run/output/motion_result.png\t"
        "位姿估计\t位置 0 mm\n"
        "[LSC_STATUS axis ok=1 metric=0.10]\n"
        "[IMAGE scan C:/run/output/scan.png]\n"
        "[LSC_DETAIL]\tscan\t0\t扫描图像\t"
        "C:/run/output/scan.png\tC:/run/output/scan_result.png\t"
        "激光三角测量\t1000 个三维点\n"
        "[LSC_DETAIL]\treconstruction\t0\t三维重建\t"
        "C:/run/output/scan_result.png\tC:/run/output/raw.ply\t"
        "轮廓拼接\t1000 个三维点\n"
        "[LSC_DETAIL]\tfilter\t0\t离群点滤波\t"
        "C:/run/output/raw.ply\tC:/run/output/filtered.ply\t"
        "统计滤波\t保留 950 个点\n"
        "[LSC_DETAIL]\tdownsample\t0\t体素下采样\t"
        "C:/run/output/filtered.ply\tC:/run/output/downsampled.ply\t"
        "体素重心\t输出 700 个点\n"
        "[LSC_DETAIL]\tsegmentation\t0\t平面分割\t"
        "C:/run/output/downsampled.ply\tC:/run/output/segmented.ply\t"
        "RANSAC\t内点 500 个\n"
        "[LSC_DETAIL]\tmeasurement\t0\t尺寸测量\t"
        "C:/run/output/raw.ply\tC:/run/output/measured.ply\t"
        "稳健高度统计\t高度 10/20 mm\n"
        "[LSC_MEASUREMENT]\tC:/run/output/raw.ply\t1000\t800\t"
        "10\t20\t24000\n"
        "[LSC_RESULT step1=10 step2=20 volume=24000 "
        "gt_volume=24000 elapsed=1.25]\n"
        "[LSC_EVIDENCE ok=1 records=9 expected=18 available=18]\n"
        "[LSC_DONE ok=1]\n");
}

} // namespace

int main(int argc, char* argv[]) {
    if (argc == 2) {
        QFile logFile(QString::fromLocal8Bit(argv[1]));
        check(logFile.open(QIODevice::ReadOnly),
              "cannot open integration log");
        const PipelineResult integration =
            ResultParser::parse(QString::fromUtf8(logFile.readAll()));
        if (!integration.protocolOk) {
            for (const QString& error : integration.integrityErrors) {
                std::cerr << error.toStdString() << '\n';
            }
        }
        check(integration.protocolOk,
              "real pipeline protocol must be internally consistent");
        check(integration.inspectionOk,
              "real pipeline inspection must pass strict verification");
        std::cout << "test_result_parser integration passed: "
                  << integration.imageDetails.size() << " records\n";
        return EXIT_SUCCESS;
    }

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

    const PipelineResult successful =
        ResultParser::parse(validPipelineLog());
    check(successful.inspectionOk, "complete evidence permits inspection pass");
    check(successful.protocolOk, "complete protocol should be accepted");
    check(successful.integrityErrors.isEmpty(), "valid protocol has no errors");
    check(successful.evidenceReported, "evidence status reported");
    check(successful.evidenceOk, "evidence status success");
    check(successful.evidenceRecords == 9, "evidence record count");
    check(successful.imageDetails.size() == 9, "detail collected in result");
    check(successful.imageDetails.front().sourcePath.contains(
              QStringLiteral("camera.png")),
          "evidence path");

    QString missingEvidenceLog = validPipelineLog();
    missingEvidenceLog.replace(
        QStringLiteral("ok=1 records=9 expected=18 available=18"),
        QStringLiteral("ok=0 records=9 expected=18 available=16"));
    const PipelineResult missingEvidence =
        ResultParser::parse(missingEvidenceLog);
    check(!missingEvidence.inspectionOk,
          "missing evidence must override pipeline success");

    QString forgedCountLog = validPipelineLog();
    forgedCountLog.replace(
        QStringLiteral("records=9 expected=18 available=18"),
        QStringLiteral("records=10 expected=20 available=20"));
    const PipelineResult forgedCount = ResultParser::parse(forgedCountLog);
    check(!forgedCount.protocolOk,
          "declared evidence count cannot replace actual detail records");
    check(!forgedCount.inspectionOk,
          "forged evidence count must fail inspection");

    QString duplicateRecordLog = validPipelineLog();
    const QString duplicate =
        QStringLiteral(
            "[LSC_DETAIL]\tfilter\t0\t离群点滤波\t"
            "C:/run/output/raw.ply\tC:/run/output/filtered.ply\t"
            "统计滤波\t保留 950 个点\n");
    duplicateRecordLog.replace(
        QStringLiteral("[LSC_DETAIL]\tdownsample"),
        duplicate + QStringLiteral("[LSC_DETAIL]\tdownsample"));
    duplicateRecordLog.replace(
        QStringLiteral("records=9 expected=18 available=18"),
        QStringLiteral("records=10 expected=20 available=20"));
    const PipelineResult duplicateRecord =
        ResultParser::parse(duplicateRecordLog);
    check(!duplicateRecord.protocolOk,
          "duplicate stage record must fail protocol validation");

    QString brokenChainLog = validPipelineLog();
    brokenChainLog.replace(
        QStringLiteral(
            "C:/run/output/raw.ply\tC:/run/output/filtered.ply"),
        QStringLiteral(
            "C:/run/output/other.ply\tC:/run/output/filtered.ply"));
    const PipelineResult brokenChain = ResultParser::parse(brokenChainLog);
    check(!brokenChain.protocolOk,
          "processing input must match the preceding output");

    QString falseMeasurementSourceLog = validPipelineLog();
    falseMeasurementSourceLog.replace(
        QStringLiteral(
            "[LSC_DETAIL]\tmeasurement\t0\t尺寸测量\t"
            "C:/run/output/raw.ply"),
        QStringLiteral(
            "[LSC_DETAIL]\tmeasurement\t0\t尺寸测量\t"
            "C:/run/output/segmented.ply"));
    const PipelineResult falseMeasurementSource =
        ResultParser::parse(falseMeasurementSourceLog);
    check(!falseMeasurementSource.protocolOk,
          "measurement evidence must name the point cloud actually measured");

    QString forgedMeasurementLog = validPipelineLog();
    forgedMeasurementLog.replace(
        QStringLiteral("\t10\t20\t24000\n[LSC_RESULT"),
        QStringLiteral("\t10\t20\t100\n[LSC_RESULT"));
    const PipelineResult forgedMeasurement =
        ResultParser::parse(forgedMeasurementLog);
    check(!forgedMeasurement.protocolOk,
          "measurement trace must match the final reported values");

    QString incompletePlanLog = validPipelineLog();
    incompletePlanLog.replace(
        QStringLiteral("camera=1 light_plane=1"),
        QStringLiteral("camera=2 light_plane=1"));
    const PipelineResult incompletePlan =
        ResultParser::parse(incompletePlanLog);
    check(!incompletePlan.protocolOk,
          "actual frame records must match the declared inspection plan");

    QString missingStatusLog = validPipelineLog();
    missingStatusLog.remove(
        QStringLiteral("[LSC_STATUS plane ok=1 metric=0.50]\n"));
    const PipelineResult missingStatus =
        ResultParser::parse(missingStatusLog);
    check(!missingStatus.inspectionOk,
          "missing calibration status must fail inspection");

    QString rejectedCalibrationLog = validPipelineLog();
    rejectedCalibrationLog.replace(
        QStringLiteral("[LSC_STATUS camera ok=1 metric=0.25]"),
        QStringLiteral("[LSC_STATUS camera ok=0 metric=8.0]"));
    const PipelineResult rejectedCalibration =
        ResultParser::parse(rejectedCalibrationLog);
    check(rejectedCalibration.protocolOk,
          "a reported failure can still be a structurally valid protocol");
    check(!rejectedCalibration.inspectionOk,
          "done ok cannot override a failed calibration stage");

    std::cout << "test_result_parser passed\n";
    return EXIT_SUCCESS;
}
