#pragma once

#include <QObject>
#include <QProcess>
#include <QString>
#include <QTimer>

class PipelineRunner : public QObject {
    Q_OBJECT

public:
    explicit PipelineRunner(QObject* parent = nullptr);
    ~PipelineRunner() override;

    void start(const QString& exePath, const QStringList& args = {});
    void stop();
    bool isRunning() const;
    void setTimeout(int ms) { m_timeoutMs = ms; }

signals:
    void outputReceived(const QString& text);
    void finished(int exitCode);
    void errorOccurred(const QString& errorString);
    void timeout();

private slots:
    void onReadyReadStdout();
    void onProcessFinished(int exitCode, QProcess::ExitStatus status);
    void onProcessError(QProcess::ProcessError error);
    void onTimeout();

private:
    QString takeCompleteLines(const QByteArray& fresh);

    QProcess* m_process;
    QTimer* m_timeout;
    int m_timeoutMs;
    bool m_running;
    QByteArray m_pending;
};
