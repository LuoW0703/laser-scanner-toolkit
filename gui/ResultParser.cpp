#include "ResultParser.h"

#include <QHash>
#include <QRegularExpression>
#include <QSet>

#include <algorithm>
#include <cmath>

namespace {

bool readBoolean(const QString& line, const QString& key, bool& value) {
    const QRegularExpression re(
        QStringLiteral("(?:^|\\s)%1=(0|1)(?=\\s|\\])")
            .arg(QRegularExpression::escape(key)));
    const auto match = re.match(line);
    if (!match.hasMatch()) {
        return false;
    }
    value = match.captured(1) == QStringLiteral("1");
    return true;
}

bool readNumber(const QString& line, const QString& key, double& value) {
    const QRegularExpression re(
        QStringLiteral(
            "(?:^|\\s)%1=([+-]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)"
            "(?:[eE][+-]?\\d+)?)(?=\\s|\\])")
            .arg(QRegularExpression::escape(key)));
    const auto match = re.match(line);
    if (!match.hasMatch()) {
        return false;
    }
    bool converted = false;
    value = match.captured(1).toDouble(&converted);
    return converted && std::isfinite(value);
}

bool readNonNegativeInteger(
    const QString& line, const QString& key, int& value) {
    const QRegularExpression re(
        QStringLiteral("(?:^|\\s)%1=(\\d+)(?=\\s|\\])")
            .arg(QRegularExpression::escape(key)));
    const auto match = re.match(line);
    if (!match.hasMatch()) {
        return false;
    }
    bool converted = false;
    value = match.captured(1).toInt(&converted);
    return converted;
}

int categoryOrder(const QString& category) {
    static const QHash<QString, int> order = {
        {QStringLiteral("camera"), 0},
        {QStringLiteral("light_plane"), 1},
        {QStringLiteral("motion_axis"), 2},
        {QStringLiteral("scan"), 3},
        {QStringLiteral("reconstruction"), 4},
        {QStringLiteral("filter"), 5},
        {QStringLiteral("downsample"), 6},
        {QStringLiteral("segmentation"), 7},
        {QStringLiteral("measurement"), 8}
    };
    return order.value(category, -1);
}

bool isFrameCategory(const QString& category) {
    return category == QStringLiteral("camera") ||
           category == QStringLiteral("light_plane") ||
           category == QStringLiteral("motion_axis") ||
           category == QStringLiteral("scan");
}

const ImageDetailRecord* recordForCategory(
    const QVector<ImageDetailRecord>& records, const QString& category) {
    for (const ImageDetailRecord& record : records) {
        if (record.category == category) {
            return &record;
        }
    }
    return nullptr;
}

} // namespace

double ResultParser::valueForKey(const QString& line, const QString& key) {
    double value = 0.0;
    return readNumber(line, key, value) ? value : 0.0;
}

bool ResultParser::boolForKey(const QString& line, const QString& key) {
    bool value = false;
    return readBoolean(line, key, value) && value;
}

PipelineResult ResultParser::parse(const QString& log) {
    PipelineResult result;
    result.rawLog = log;

    int cameraStatusCount = 0;
    int planeStatusCount = 0;
    int axisStatusCount = 0;
    int planCount = 0;
    int measurementTraceCount = 0;
    int resultCount = 0;
    int evidenceCount = 0;
    int doneCount = 0;
    int lastCategoryOrder = -1;
    QHash<QString, int> lastIndexByCategory;
    QHash<QString, int> countByCategory;
    QSet<QString> detailKeys;
    QSet<QString> detailPairs;
    QSet<QString> imagePaths;
    bool resultSeen = false;
    bool evidenceSeen = false;
    bool doneSeen = false;

    const auto addError = [&result](const QString& message) {
        if (!result.integrityErrors.contains(message)) {
            result.integrityErrors.append(message);
        }
    };

    const QStringList lines = log.split('\n', Qt::SkipEmptyParts);
    for (const QString& rawLine : lines) {
        const QString line = rawLine.trimmed();

        if (line.startsWith(QStringLiteral("[LSC_PLAN "))) {
            if (!result.imageDetails.isEmpty() ||
                !result.imageList.isEmpty() ||
                cameraStatusCount > 0 || planeStatusCount > 0 ||
                axisStatusCount > 0 || measurementTraceCount > 0 ||
                resultSeen || evidenceSeen || doneSeen) {
                addError(QStringLiteral("点检计划未在处理记录之前发布"));
            }
            ++planCount;
            result.planReported =
                readNonNegativeInteger(
                    line, QStringLiteral("camera"),
                    result.plannedCameraRecords) &&
                readNonNegativeInteger(
                    line, QStringLiteral("light_plane"),
                    result.plannedLightPlaneRecords) &&
                readNonNegativeInteger(
                    line, QStringLiteral("motion_axis"),
                    result.plannedMotionAxisRecords) &&
                readNonNegativeInteger(
                    line, QStringLiteral("scan_frames"),
                    result.plannedScanFrames) &&
                readNonNegativeInteger(
                    line, QStringLiteral("scan_evidence"),
                    result.plannedScanEvidenceRecords) &&
                readNonNegativeInteger(
                    line, QStringLiteral("scan_stride"),
                    result.plannedScanStride);
            if (!result.planReported ||
                result.plannedCameraRecords <= 0 ||
                result.plannedLightPlaneRecords <= 0 ||
                result.plannedMotionAxisRecords <= 0 ||
                result.plannedScanFrames <= 0 ||
                result.plannedScanEvidenceRecords <= 0 ||
                result.plannedScanStride <= 0) {
                addError(QStringLiteral("点检计划字段缺失或数值无效"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_DETAIL]\t"))) {
            const ImageDetailRecord detail = parseImageDetail(line);
            if (!detail.valid()) {
                addError(QStringLiteral("存在字段缺失或输入输出相同的无效详情记录"));
                continue;
            }
            if (resultSeen || evidenceSeen || doneSeen) {
                addError(QStringLiteral("详情记录出现在结果汇总之后"));
            }

            const int order = categoryOrder(detail.category);
            if (order < 0) {
                addError(QStringLiteral("存在未知点检阶段：%1")
                             .arg(detail.category));
                continue;
            }
            if (order < lastCategoryOrder) {
                addError(QStringLiteral("点检阶段顺序发生回退：%1")
                             .arg(detail.category));
            }
            lastCategoryOrder = std::max(lastCategoryOrder, order);

            const QString key =
                detail.category + QChar(0x1f) + QString::number(detail.index);
            if (detailKeys.contains(key)) {
                addError(QStringLiteral("点检记录编号重复：%1 #%2")
                             .arg(detail.category)
                             .arg(detail.index));
            } else {
                detailKeys.insert(key);
            }

            const QString pair =
                detail.sourcePath + QChar(0x1f) + detail.processedPath;
            if (detailPairs.contains(pair)) {
                addError(QStringLiteral("点检输入输出证据对重复：%1 #%2")
                             .arg(detail.category)
                             .arg(detail.index));
            } else {
                detailPairs.insert(pair);
            }

            const int previousIndex =
                lastIndexByCategory.value(detail.category, -1);
            if (isFrameCategory(detail.category)) {
                const bool indexValid =
                    detail.category == QStringLiteral("scan")
                    ? ((previousIndex < 0 && detail.index == 0) ||
                       (previousIndex >= 0 && detail.index > previousIndex))
                    : detail.index == previousIndex + 1;
                if (!indexValid) {
                    addError(QStringLiteral("点检帧编号不连续或未递增：%1 #%2")
                                 .arg(detail.category)
                                 .arg(detail.index));
                }
            } else if (detail.index != 0) {
                addError(QStringLiteral("处理阶段编号必须为 0：%1")
                             .arg(detail.category));
            }
            lastIndexByCategory.insert(detail.category, detail.index);
            countByCategory[detail.category] += 1;
            result.imageDetails.append(detail);
        } else if (line.startsWith(QStringLiteral("[IMAGE "))) {
            if (resultSeen || evidenceSeen || doneSeen) {
                addError(QStringLiteral("原始图像标记出现在结果汇总之后"));
            }
            const QString path = parseImagePath(line);
            if (!path.isEmpty()) {
                if (imagePaths.contains(path)) {
                    addError(QStringLiteral("原始图像标记重复：%1").arg(path));
                }
                imagePaths.insert(path);
                result.imageList.append(path);
            } else {
                addError(QStringLiteral("存在无法解析的原始图像标记"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_STATUS camera "))) {
            ++cameraStatusCount;
            result.cameraReported =
                readBoolean(line, QStringLiteral("ok"), result.cameraOk) &&
                readNumber(
                    line, QStringLiteral("metric"), result.cameraRms);
            if (!result.cameraReported) {
                addError(QStringLiteral("相机标定状态字段不完整"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_STATUS plane "))) {
            ++planeStatusCount;
            result.planeReported =
                readBoolean(line, QStringLiteral("ok"), result.planeOk) &&
                readNumber(
                    line, QStringLiteral("metric"), result.planeAngle);
            if (!result.planeReported) {
                addError(QStringLiteral("光平面标定状态字段不完整"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_STATUS axis "))) {
            ++axisStatusCount;
            result.axisReported =
                readBoolean(line, QStringLiteral("ok"), result.axisOk) &&
                readNumber(
                    line, QStringLiteral("metric"), result.axisAngle);
            if (!result.axisReported) {
                addError(QStringLiteral("移动轴标定状态字段不完整"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_MEASUREMENT]\t"))) {
            if (countByCategory.value(QStringLiteral("measurement")) != 1 ||
                resultSeen || evidenceSeen || doneSeen) {
                addError(QStringLiteral("测量追踪未紧随完整测量过程"));
            }
            ++measurementTraceCount;
            const QStringList fields =
                line.split('\t', Qt::KeepEmptyParts);
            bool sourcePointsOk = false;
            bool topPointsOk = false;
            bool step1Ok = false;
            bool step2Ok = false;
            bool volumeOk = false;
            if (fields.size() == 7) {
                result.measurementSourcePath = fields.at(1);
                result.measurementSourcePoints =
                    fields.at(2).toLongLong(&sourcePointsOk);
                result.measurementTopPoints =
                    fields.at(3).toLongLong(&topPointsOk);
                result.tracedStep1Height =
                    fields.at(4).toDouble(&step1Ok);
                result.tracedStep2Height =
                    fields.at(5).toDouble(&step2Ok);
                result.tracedVolume =
                    fields.at(6).toDouble(&volumeOk);
            }
            result.measurementTraceReported =
                fields.size() == 7 &&
                !result.measurementSourcePath.isEmpty() &&
                sourcePointsOk && result.measurementSourcePoints > 0 &&
                topPointsOk && result.measurementTopPoints > 0 &&
                result.measurementTopPoints <=
                    result.measurementSourcePoints &&
                step1Ok && std::isfinite(result.tracedStep1Height) &&
                step2Ok && std::isfinite(result.tracedStep2Height) &&
                volumeOk && std::isfinite(result.tracedVolume);
            if (!result.measurementTraceReported) {
                addError(QStringLiteral("测量过程追踪字段缺失或数值无效"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_RESULT "))) {
            if (measurementTraceCount != 1 || evidenceSeen || doneSeen) {
                addError(QStringLiteral("最终结果早于测量过程追踪"));
            }
            ++resultCount;
            resultSeen = true;
            result.measurementsReported =
                readNumber(
                    line, QStringLiteral("step1"), result.step1Height) &&
                readNumber(
                    line, QStringLiteral("step2"), result.step2Height) &&
                readNumber(
                    line, QStringLiteral("volume"), result.measVolume) &&
                readNumber(
                    line, QStringLiteral("gt_volume"), result.gtVolume) &&
                readNumber(
                    line, QStringLiteral("elapsed"), result.totalTime);
            if (!result.measurementsReported ||
                result.gtVolume <= 0.0 || result.totalTime <= 0.0) {
                addError(QStringLiteral("测量结果字段缺失或数值无效"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_EVIDENCE "))) {
            ++evidenceCount;
            result.evidenceReported = true;
            evidenceSeen = true;
            if (!resultSeen) {
                addError(QStringLiteral("证据汇总早于测量结果"));
            }
            const bool fieldsValid =
                readBoolean(
                    line, QStringLiteral("ok"), result.evidenceOk) &&
                readNonNegativeInteger(
                    line, QStringLiteral("records"),
                    result.evidenceRecords) &&
                readNonNegativeInteger(
                    line, QStringLiteral("expected"),
                    result.evidenceExpectedFiles) &&
                readNonNegativeInteger(
                    line, QStringLiteral("available"),
                    result.evidenceAvailableFiles);
            if (!fieldsValid) {
                addError(QStringLiteral("证据汇总字段不完整"));
            }
        } else if (line.startsWith(QStringLiteral("[LSC_DONE "))) {
            ++doneCount;
            result.completed = true;
            doneSeen = true;
            if (!evidenceSeen) {
                addError(QStringLiteral("完成标记早于证据汇总"));
            }
            if (!readBoolean(
                    line, QStringLiteral("ok"),
                    result.declaredInspectionOk)) {
                addError(QStringLiteral("完成标记字段不完整"));
            }
        }
    }

    if (planCount != 1 || !result.planReported) {
        addError(QStringLiteral("点检计划必须且只能出现一次"));
    }
    if (cameraStatusCount != 1 || !result.cameraReported) {
        addError(QStringLiteral("相机标定状态必须且只能出现一次"));
    }
    if (planeStatusCount != 1 || !result.planeReported) {
        addError(QStringLiteral("光平面标定状态必须且只能出现一次"));
    }
    if (axisStatusCount != 1 || !result.axisReported) {
        addError(QStringLiteral("移动轴标定状态必须且只能出现一次"));
    }
    if (resultCount != 1 || !result.measurementsReported) {
        addError(QStringLiteral("测量结果必须且只能出现一次"));
    }
    if (measurementTraceCount != 1 ||
        !result.measurementTraceReported) {
        addError(QStringLiteral("测量过程追踪必须且只能出现一次"));
    }
    if (evidenceCount != 1 || !result.evidenceReported) {
        addError(QStringLiteral("证据汇总必须且只能出现一次"));
    }
    if (doneCount != 1 || !result.completed) {
        addError(QStringLiteral("完成标记必须且只能出现一次"));
    }

    const QStringList requiredCategories = {
        QStringLiteral("camera"),
        QStringLiteral("light_plane"),
        QStringLiteral("motion_axis"),
        QStringLiteral("scan"),
        QStringLiteral("reconstruction"),
        QStringLiteral("filter"),
        QStringLiteral("downsample"),
        QStringLiteral("segmentation"),
        QStringLiteral("measurement")
    };
    for (const QString& category : requiredCategories) {
        const int count = countByCategory.value(category);
        if (count == 0) {
            addError(QStringLiteral("缺少点检阶段记录：%1").arg(category));
        } else if (!isFrameCategory(category) && count != 1) {
            addError(QStringLiteral("处理阶段记录必须且只能有一条：%1")
                         .arg(category));
        }
    }
    if (result.planReported) {
        if (countByCategory.value(QStringLiteral("camera")) !=
            result.plannedCameraRecords) {
            addError(QStringLiteral("相机图像详情数与点检计划不一致"));
        }
        if (countByCategory.value(QStringLiteral("light_plane")) !=
            result.plannedLightPlaneRecords) {
            addError(QStringLiteral("光平面图像详情数与点检计划不一致"));
        }
        if (countByCategory.value(QStringLiteral("motion_axis")) !=
            result.plannedMotionAxisRecords) {
            addError(QStringLiteral("移动轴图像详情数与点检计划不一致"));
        }
        if (countByCategory.value(QStringLiteral("scan")) !=
            result.plannedScanEvidenceRecords) {
            addError(QStringLiteral("扫描抽样详情数与点检计划不一致"));
        }
        const int expectedScanEvidence =
            (result.plannedScanFrames + result.plannedScanStride - 1) /
            result.plannedScanStride;
        if (result.plannedScanEvidenceRecords != expectedScanEvidence) {
            addError(QStringLiteral("扫描抽样计划数量与总帧数、步长不一致"));
        }
        int scanSequence = 0;
        for (const ImageDetailRecord& detail : result.imageDetails) {
            if (detail.category != QStringLiteral("scan")) {
                continue;
            }
            if (detail.index != scanSequence * result.plannedScanStride) {
                addError(QStringLiteral("扫描证据编号与计划抽样步长不一致"));
                break;
            }
            ++scanSequence;
        }
    }

    QSet<QString> detailFrameSources;
    for (const ImageDetailRecord& detail : result.imageDetails) {
        if (isFrameCategory(detail.category)) {
            detailFrameSources.insert(detail.sourcePath);
        }
    }
    if (imagePaths != detailFrameSources ||
        result.imageList.size() != detailFrameSources.size()) {
        addError(QStringLiteral(
            "原始图像标记与相机、光平面、移动轴及扫描详情未一一对应"));
    }

    if (result.evidenceRecords != result.imageDetails.size()) {
        addError(QStringLiteral("证据汇总记录数与实际详情条数不一致"));
    }
    if (result.evidenceExpectedFiles != result.evidenceRecords * 2) {
        addError(QStringLiteral("证据汇总文件数不等于每条记录两个文件"));
    }
    if (result.evidenceAvailableFiles != result.evidenceExpectedFiles) {
        addError(QStringLiteral("证据汇总声明存在缺失文件"));
    }
    if (!result.evidenceOk) {
        addError(QStringLiteral("流水线声明证据审计失败"));
    }

    const ImageDetailRecord* reconstruction =
        recordForCategory(
            result.imageDetails, QStringLiteral("reconstruction"));
    const ImageDetailRecord* filter =
        recordForCategory(result.imageDetails, QStringLiteral("filter"));
    const ImageDetailRecord* downsample =
        recordForCategory(
            result.imageDetails, QStringLiteral("downsample"));
    const ImageDetailRecord* segmentation =
        recordForCategory(
            result.imageDetails, QStringLiteral("segmentation"));
    const ImageDetailRecord* measurement =
        recordForCategory(
            result.imageDetails, QStringLiteral("measurement"));

    bool reconstructionLinked = false;
    if (reconstruction) {
        for (const ImageDetailRecord& detail : result.imageDetails) {
            if (detail.category == QStringLiteral("scan") &&
                detail.processedPath == reconstruction->sourcePath) {
                reconstructionLinked = true;
                break;
            }
        }
    }
    if (!reconstructionLinked) {
        addError(QStringLiteral("三维重建输入未对应任何扫描处理结果"));
    }

    const auto requireLink =
        [&addError](
            const ImageDetailRecord* previous,
            const ImageDetailRecord* next,
            const QString& description) {
            if (!previous || !next ||
                previous->processedPath != next->sourcePath) {
                addError(description);
            }
        };
    requireLink(
        reconstruction, filter,
        QStringLiteral("滤波输入未对应三维重建输出"));
    requireLink(
        filter, downsample,
        QStringLiteral("下采样输入未对应滤波输出"));
    requireLink(
        downsample, segmentation,
        QStringLiteral("平面分割输入未对应下采样输出"));
    requireLink(
        reconstruction, measurement,
        QStringLiteral("测量输入未对应原始三维重建输出"));

    if (measurement &&
        result.measurementTraceReported &&
        measurement->sourcePath != result.measurementSourcePath) {
        addError(QStringLiteral("测量详情输入与结构化测量追踪不一致"));
    }
    const auto nearlyEqual = [](double left, double right) {
        const double scale =
            std::max({1.0, std::abs(left), std::abs(right)});
        return std::abs(left - right) <= scale * 1e-9;
    };
    if (result.measurementTraceReported &&
        result.measurementsReported &&
        (!nearlyEqual(
             result.tracedStep1Height, result.step1Height) ||
         !nearlyEqual(
             result.tracedStep2Height, result.step2Height) ||
         !nearlyEqual(result.tracedVolume, result.measVolume))) {
        addError(QStringLiteral("测量过程数值与最终结果不一致"));
    }

    result.protocolOk = result.integrityErrors.isEmpty();
    result.inspectionOk =
        result.declaredInspectionOk &&
        result.protocolOk &&
        result.cameraOk &&
        result.planeOk &&
        result.axisOk &&
        result.measurementsReported &&
        result.evidenceOk;
    return result;
}

int ResultParser::parseProgress(const QString& line) {
    if (line.startsWith(QStringLiteral("[LSC_PROGRESS "))) {
        return static_cast<int>(valueForKey(line, QStringLiteral("value")));
    }

    const QRegularExpression phaseRe(QStringLiteral("\\[Phase (\\d)/4\\]"));
    const auto match = phaseRe.match(line);
    if (match.hasMatch()) {
        return (match.captured(1).toInt() - 1) * 25 + 5;
    }
    return -1;
}

QString ResultParser::parseImagePath(const QString& line) {
    const int start = line.indexOf(QStringLiteral("[IMAGE "));
    if (start < 0) {
        return {};
    }

    const int typeStart = start + 7;
    const int typeEnd = line.indexOf(' ', typeStart);
    if (typeEnd < 0) {
        return {};
    }

    const int pathStart = typeEnd + 1;
    int pathEnd = line.lastIndexOf(']');
    if (pathEnd < pathStart) {
        pathEnd = line.size();
    }
    return line.mid(pathStart, pathEnd - pathStart).trimmed();
}

ImageDetailRecord ResultParser::parseImageDetail(const QString& line) {
    ImageDetailRecord detail;
    if (!line.startsWith(QStringLiteral("[LSC_DETAIL]\t"))) {
        return detail;
    }

    // 制表符不会出现在 Windows 文件名中，既允许路径包含空格，也避免
    // 为展示文本引入 JSON 依赖。协议固定为 8 列，所有字段都必须有效。
    const QStringList fields = line.split('\t', Qt::KeepEmptyParts);
    if (fields.size() != 8) {
        return detail;
    }

    bool indexOk = false;
    const int index = fields.at(2).toInt(&indexOk);
    if (!indexOk) {
        return detail;
    }

    detail.category = fields.at(1);
    detail.index = index;
    detail.title = fields.at(3);
    detail.sourcePath = fields.at(4);
    detail.processedPath = fields.at(5);
    detail.algorithm = fields.at(6);
    detail.summary = fields.at(7);
    return detail;
}
