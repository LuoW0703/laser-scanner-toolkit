#include "PipelineRunner.h"

#include <QFileInfo>

PipelineRunner::PipelineRunner(QObject* parent)
    : QObject(parent)
    , m_process(new QProcess(this))
    , m_timeout(new QTimer(this))
    , m_timeoutMs(10 * 60 * 1000)
    , m_running(false) {
    m_process->setProcessChannelMode(QProcess::MergedChannels);

    connect(m_process, &QProcess::readyReadStandardOutput,
            this, &PipelineRunner::onReadyReadStdout);
    connect(m_process,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &PipelineRunner::onProcessFinished);
    connect(m_process, &QProcess::errorOccurred,
            this, &PipelineRunner::onProcessError);

    m_timeout->setSingleShot(true);
    connect(m_timeout, &QTimer::timeout, this, &PipelineRunner::onTimeout);
}

PipelineRunner::~PipelineRunner() {
    stop();
}

void PipelineRunner::start(const QString& exePath, const QStringList& args) {
    if (isRunning()) {
        stop();
    }

    m_pending.clear();
    m_running = true;
    m_process->setWorkingDirectory(QFileInfo(exePath).absolutePath());
    m_process->start(exePath, args);

    if (m_timeoutMs > 0) {
        m_timeout->start(m_timeoutMs);
    }
}

void PipelineRunner::stop() {
    m_timeout->stop();
    if (m_process->state() != QProcess::NotRunning) {
        m_process->kill();
    }
    m_running = false;
}

bool PipelineRunner::isRunning() const {
    return m_running && m_process->state() != QProcess::NotRunning;
}

QString PipelineRunner::takeCompleteLines(const QByteArray& fresh) {
    m_pending.append(fresh);
    const int end = m_pending.lastIndexOf('\n');
    if (end < 0) {
        return {};
    }

    const QByteArray complete = m_pending.left(end + 1);
    m_pending.remove(0, end + 1);
    return QString::fromUtf8(complete);
}

void PipelineRunner::onReadyReadStdout() {
    const QString text = takeCompleteLines(m_process->readAllStandardOutput());
    if (!text.isEmpty()) {
        emit outputReceived(text);
    }
}

void PipelineRunner::onProcessFinished(int exitCode, QProcess::ExitStatus status) {
    QString text = takeCompleteLines(m_process->readAllStandardOutput());
    if (!m_pending.isEmpty()) {
        text += QString::fromUtf8(m_pending);
        m_pending.clear();
    }
    if (!text.isEmpty()) {
        emit outputReceived(text);
    }

    m_timeout->stop();
    m_running = false;
    emit finished(status == QProcess::NormalExit ? exitCode : -1);
}

void PipelineRunner::onProcessError(QProcess::ProcessError error) {
    m_timeout->stop();
    m_running = false;

    QString message;
    switch (error) {
    case QProcess::FailedToStart:
        message = QStringLiteral("无法启动点检程序，请检查程序路径和运行库。");
        break;
    case QProcess::Crashed:
        message = QStringLiteral("点检程序异常退出。");
        break;
    case QProcess::Timedout:
        message = QStringLiteral("点检程序等待超时。");
        break;
    default:
        message = QStringLiteral("点检进程错误，代码：%1").arg(static_cast<int>(error));
        break;
    }
    emit errorOccurred(message);
}

void PipelineRunner::onTimeout() {
    emit timeout();
}
