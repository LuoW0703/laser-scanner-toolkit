#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

/**
 * 一张仿真图像对应的可审查处理记录。
 *
 * sourcePath 保存算法输入，processedPath 保存叠加了检测结果的诊断图。
 * algorithm 和 summary 由流水线生成，因此 GUI 只负责展示，不重复推导
 * 算法状态，避免界面与核心程序出现两套判断标准。
 */
struct ImageDetailRecord {
    QString category;
    int index = -1;
    QString title;
    QString sourcePath;
    QString processedPath;
    QString algorithm;
    QString summary;

    bool valid() const {
        return index >= 0 && !title.isEmpty() &&
               !sourcePath.isEmpty() && !processedPath.isEmpty();
    }
};

struct PipelineResult {
    bool cameraOk = false;
    double cameraRms = 0.0;
    bool planeOk = false;
    double planeAngle = 0.0;
    bool axisOk = false;
    double axisAngle = 0.0;

    double step1Height = 0.0;
    double step2Height = 0.0;
    double measVolume = 0.0;
    double gtVolume = 0.0;
    double totalTime = 0.0;

    bool completed = false;
    bool inspectionOk = false;
    bool evidenceReported = false;
    bool evidenceOk = false;
    int evidenceRecords = 0;
    int evidenceExpectedFiles = 0;
    int evidenceAvailableFiles = 0;
    QString rawLog;
    QStringList imageList;
    QVector<ImageDetailRecord> imageDetails;

    bool valid() const { return completed || totalTime > 0.0 || measVolume > 0.0; }
};

class ResultParser {
public:
    static PipelineResult parse(const QString& log);
    static int parseProgress(const QString& line);
    static QString parseImagePath(const QString& line);
    static ImageDetailRecord parseImageDetail(const QString& line);

private:
    static double valueForKey(const QString& line, const QString& key);
    static bool boolForKey(const QString& line, const QString& key);
};
