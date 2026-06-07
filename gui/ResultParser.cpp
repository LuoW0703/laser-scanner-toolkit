#include "ResultParser.h"

#include <QRegularExpression>

double ResultParser::valueForKey(const QString& line, const QString& key) {
    const QRegularExpression re(
        QStringLiteral("(?:^|\\s)%1=([+-]?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)")
            .arg(QRegularExpression::escape(key)));
    const auto match = re.match(line);
    return match.hasMatch() ? match.captured(1).toDouble() : 0.0;
}

bool ResultParser::boolForKey(const QString& line, const QString& key) {
    const QRegularExpression re(
        QStringLiteral("(?:^|\\s)%1=(0|1)")
            .arg(QRegularExpression::escape(key)));
    const auto match = re.match(line);
    return match.hasMatch() && match.captured(1) == QStringLiteral("1");
}

PipelineResult ResultParser::parse(const QString& log) {
    PipelineResult result;
    result.rawLog = log;

    const QStringList lines = log.split('\n', Qt::SkipEmptyParts);
    for (const QString& rawLine : lines) {
        const QString line = rawLine.trimmed();

        if (line.startsWith(QStringLiteral("[IMAGE "))) {
            const QString path = parseImagePath(line);
            if (!path.isEmpty()) {
                result.imageList.append(path);
            }
        } else if (line.startsWith(QStringLiteral("[LSC_STATUS camera "))) {
            result.cameraOk = boolForKey(line, QStringLiteral("ok"));
            result.cameraRms = valueForKey(line, QStringLiteral("metric"));
        } else if (line.startsWith(QStringLiteral("[LSC_STATUS plane "))) {
            result.planeOk = boolForKey(line, QStringLiteral("ok"));
            result.planeAngle = valueForKey(line, QStringLiteral("metric"));
        } else if (line.startsWith(QStringLiteral("[LSC_STATUS axis "))) {
            result.axisOk = boolForKey(line, QStringLiteral("ok"));
            result.axisAngle = valueForKey(line, QStringLiteral("metric"));
        } else if (line.startsWith(QStringLiteral("[LSC_RESULT "))) {
            result.step1Height = valueForKey(line, QStringLiteral("step1"));
            result.step2Height = valueForKey(line, QStringLiteral("step2"));
            result.measVolume = valueForKey(line, QStringLiteral("volume"));
            result.gtVolume = valueForKey(line, QStringLiteral("gt_volume"));
            result.totalTime = valueForKey(line, QStringLiteral("elapsed"));
        } else if (line.startsWith(QStringLiteral("[LSC_DONE "))) {
            result.completed = true;
            result.inspectionOk = boolForKey(line, QStringLiteral("ok"));
        }
    }

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
    // 为展示文本引入 JSON 依赖。协议固定为 8 列，summary 可为空。
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
