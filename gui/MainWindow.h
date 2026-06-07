#pragma once

#include "ResultParser.h"

#include <QDateTime>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QListWidget>
#include <QMainWindow>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QSpinBox>
#include <QStatusBar>
#include <QTextEdit>
#include <QVector>

class PipelineRunner;
class EvidenceView;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

private slots:
    void onRunClicked();
    void onStopClicked();
    void onProcessOutput(const QString& text);
    void onProcessFinished(int exitCode);
    void onProcessError(QProcess::ProcessError error);
    void onDetailSelectionChanged(int row);

private:
    void setupUi();
    void setupMenuBar();
    void setupParamPanel();
    void setupResultPanel();
    void setupLogPanel();
    void setupImagePanel();
    void setupStatusBar();

    void parseResults(const QString& log, int exitCode);
    QString writeConfigFile();
    void setRunning(bool running);

    void showImageWindow();
    void appendImageDetail(const ImageDetailRecord& detail);
    void displayImageDetail(int index);
    void displayEvidence(EvidenceView* target, const QString& path,
                         const QString& emptyText);
    QString resolveEvidencePath(const QString& path) const;
    bool validateEvidenceFiles(QStringList& errors) const;
    bool validateMeasurementResult(
        const PipelineResult& result, QStringList& errors) const;

    QTextEdit* m_logView;
    QTextEdit* m_resultView;
    QProgressBar* m_progress;

    QPushButton* m_btnRun;
    QPushButton* m_btnStop;

    QLabel* m_statusIcon;
    QLabel* m_statusText;

    QGroupBox* m_paramGroup;
    QGroupBox* m_resultGroup;
    QGroupBox* m_logGroup;
    QGroupBox* m_imageGroup;

    QSpinBox* m_spinCamImages;
    QSpinBox* m_spinLpImages;
    QSpinBox* m_spinMaSteps;
    QDoubleSpinBox* m_spinMaStepMm;
    QDoubleSpinBox* m_spinBlockWidth;
    QDoubleSpinBox* m_spinBlockDepth;
    QDoubleSpinBox* m_spinStep1H;
    QDoubleSpinBox* m_spinStep2H;
    QDoubleSpinBox* m_spinVoxelSize;
    QDoubleSpinBox* m_spinRansacDist;

    PipelineRunner* m_runner;
    QString m_demoPath;
    QString m_fullLog;
    QDateTime m_runStartedAt;

    QLabel* m_imageTitle;
    QListWidget* m_detailList;
    EvidenceView* m_originalEvidenceView;
    EvidenceView* m_processedEvidenceView;
    QTextEdit* m_detailView;
    QVector<ImageDetailRecord> m_imageDetails;
    int m_currentImageIndex;
};
