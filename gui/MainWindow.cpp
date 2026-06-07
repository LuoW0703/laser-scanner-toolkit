#include "MainWindow.h"

#include "EvidenceView.h"
#include "PipelineRunner.h"
#include "ResultParser.h"

#include <QAction>
#include <QApplication>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QScrollBar>
#include <QSplitter>
#include <QTextCursor>
#include <QTextStream>
#include <QToolButton>
#include <QVBoxLayout>

#include <functional>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_runner(nullptr)
    , m_currentImageIndex(-1) {
    setupUi();
    setupMenuBar();

    m_runner = new PipelineRunner(this);
    m_runner->setTimeout(10 * 60 * 1000);
    connect(m_runner, &PipelineRunner::outputReceived,
            this, &MainWindow::onProcessOutput);
    connect(m_runner, &PipelineRunner::finished,
            this, &MainWindow::onProcessFinished);
    connect(m_runner, &PipelineRunner::errorOccurred, this,
            [this](const QString& error) {
                m_logView->append(QStringLiteral("\n[错误] ") + error);
                m_progress->setFormat(QStringLiteral("点检失败"));
                setRunning(false);
            });
    connect(m_runner, &PipelineRunner::timeout, this, [this]() {
        m_logView->append(QStringLiteral("\n[超时] 点检超过 10 分钟，已终止。"));
        m_runner->stop();
        m_progress->setFormat(QStringLiteral("点检超时"));
        setRunning(false);
    });

    const QDir appDir(QApplication::applicationDirPath());
    const QStringList candidates = {
        appDir.filePath(QStringLiteral("demo_full_pipeline.exe")),
        appDir.filePath(QStringLiteral("../demo_full_pipeline.exe")),
        appDir.filePath(QStringLiteral("../../build/demo_full_pipeline.exe")),
        QDir(QStringLiteral("D:/laser_scanner_toolkit/build"))
            .filePath(QStringLiteral("demo_full_pipeline.exe"))
    };
    for (const QString& candidate : candidates) {
        if (QFileInfo::exists(candidate)) {
            m_demoPath = QDir::toNativeSeparators(QFileInfo(candidate).absoluteFilePath());
            break;
        }
    }

    setRunning(false);
    m_logView->append(
        m_demoPath.isEmpty()
            ? QStringLiteral("[就绪] 未找到点检程序，请通过“文件”菜单选择。")
            : QStringLiteral("[就绪] 点检程序：") + m_demoPath);
}

MainWindow::~MainWindow() {
    if (m_runner && m_runner->isRunning()) {
        m_runner->stop();
    }
}

void MainWindow::setupUi() {
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* mainLayout = new QHBoxLayout(central);
    mainLayout->setContentsMargins(8, 8, 8, 8);
    mainLayout->setSpacing(8);

    auto* leftPanel = new QWidget();
    auto* leftLayout = new QVBoxLayout(leftPanel);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(8);

    auto* buttonLayout = new QHBoxLayout();
    m_btnRun = new QPushButton(QStringLiteral("运行点检"));
    m_btnStop = new QPushButton(QStringLiteral("停止"));
    m_btnRun->setMinimumHeight(36);
    m_btnStop->setMinimumHeight(36);
    m_btnRun->setStyleSheet(
        "QPushButton { background:#2e7d32; color:white; font-weight:bold; "
        "border-radius:4px; padding:6px 16px; }"
        "QPushButton:hover { background:#388e3c; }"
        "QPushButton:disabled { background:#aaa; }");
    m_btnStop->setStyleSheet(
        "QPushButton { background:#c62828; color:white; font-weight:bold; "
        "border-radius:4px; padding:6px 16px; }"
        "QPushButton:hover { background:#d32f2f; }"
        "QPushButton:disabled { background:#aaa; }");
    buttonLayout->addWidget(m_btnRun);
    buttonLayout->addWidget(m_btnStop);
    leftLayout->addLayout(buttonLayout);

    m_progress = new QProgressBar();
    m_progress->setRange(0, 100);
    m_progress->setValue(0);
    m_progress->setFormat(QStringLiteral("就绪"));
    leftLayout->addWidget(m_progress);

    setupParamPanel();
    setupResultPanel();
    leftLayout->addWidget(m_paramGroup);
    leftLayout->addWidget(m_resultGroup, 1);
    leftPanel->setMaximumWidth(410);

    setupImagePanel();
    setupLogPanel();

    auto* rightSplitter = new QSplitter(Qt::Vertical);
    rightSplitter->addWidget(m_imageGroup);
    rightSplitter->addWidget(m_logGroup);
    rightSplitter->setStretchFactor(0, 3);
    rightSplitter->setStretchFactor(1, 2);
    rightSplitter->setSizes({430, 280});

    auto* splitter = new QSplitter(Qt::Horizontal);
    splitter->addWidget(leftPanel);
    splitter->addWidget(rightSplitter);
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({390, 800});
    mainLayout->addWidget(splitter);

    connect(m_btnRun, &QPushButton::clicked, this, &MainWindow::onRunClicked);
    connect(m_btnStop, &QPushButton::clicked, this, &MainWindow::onStopClicked);
    setupStatusBar();
}

void MainWindow::setupMenuBar() {
    QMenu* fileMenu = menuBar()->addMenu(QStringLiteral("文件"));
    QAction* selectDemo = fileMenu->addAction(QStringLiteral("选择点检程序..."));
    connect(selectDemo, &QAction::triggered, this, [this]() {
        const QString path = QFileDialog::getOpenFileName(
            this, QStringLiteral("选择 demo_full_pipeline.exe"),
            QApplication::applicationDirPath(),
            QStringLiteral("可执行文件 (*.exe);;所有文件 (*.*)"));
        if (!path.isEmpty()) {
            m_demoPath = QDir::toNativeSeparators(path);
            m_logView->append(QStringLiteral("[就绪] 已选择：") + m_demoPath);
        }
    });

    fileMenu->addSeparator();
    QAction* exitAction = fileMenu->addAction(QStringLiteral("退出"));
    connect(exitAction, &QAction::triggered, this, &QWidget::close);

    QMenu* helpMenu = menuBar()->addMenu(QStringLiteral("帮助"));
    QAction* aboutAction = helpMenu->addAction(QStringLiteral("关于"));
    connect(aboutAction, &QAction::triggered, this, [this]() {
        QMessageBox::about(
            this, QStringLiteral("关于"),
            QStringLiteral("3D 线激光轮廓仪标定与点检工具\n"
                           "C++17 / Eigen / OpenCV / Qt5"));
    });
}

void MainWindow::setupParamPanel() {
    m_paramGroup = new QGroupBox(QStringLiteral("仿真参数"));
    auto* layout = new QVBoxLayout();
    layout->setSpacing(4);

    auto addRow = [layout](const QString& text, QWidget* control) {
        auto* row = new QHBoxLayout();
        row->addWidget(new QLabel(text));
        row->addStretch();
        row->addWidget(control);
        layout->addLayout(row);
    };

    m_spinCamImages = new QSpinBox();
    m_spinCamImages->setRange(6, 50);
    m_spinCamImages->setValue(12);
    addRow(QStringLiteral("相机标定图像："), m_spinCamImages);

    m_spinLpImages = new QSpinBox();
    m_spinLpImages->setRange(3, 30);
    m_spinLpImages->setValue(8);
    addRow(QStringLiteral("光平面图像："), m_spinLpImages);

    m_spinMaSteps = new QSpinBox();
    m_spinMaSteps->setRange(3, 20);
    m_spinMaSteps->setValue(5);
    addRow(QStringLiteral("移动轴位置："), m_spinMaSteps);

    m_spinMaStepMm = new QDoubleSpinBox();
    m_spinMaStepMm->setRange(0.1, 10.0);
    m_spinMaStepMm->setValue(2.0);
    m_spinMaStepMm->setSuffix(QStringLiteral(" mm"));
    addRow(QStringLiteral("移动轴步长："), m_spinMaStepMm);

    m_spinBlockWidth = new QDoubleSpinBox();
    m_spinBlockWidth->setRange(10, 200);
    m_spinBlockWidth->setValue(60.0);
    m_spinBlockWidth->setSuffix(QStringLiteral(" mm"));
    addRow(QStringLiteral("工件宽度："), m_spinBlockWidth);

    m_spinBlockDepth = new QDoubleSpinBox();
    m_spinBlockDepth->setRange(10, 200);
    m_spinBlockDepth->setValue(40.0);
    m_spinBlockDepth->setSuffix(QStringLiteral(" mm"));
    addRow(QStringLiteral("工件深度："), m_spinBlockDepth);

    m_spinStep1H = new QDoubleSpinBox();
    m_spinStep1H->setRange(1, 50);
    m_spinStep1H->setValue(10.0);
    m_spinStep1H->setSuffix(QStringLiteral(" mm"));
    addRow(QStringLiteral("台阶 1 高度："), m_spinStep1H);

    m_spinStep2H = new QDoubleSpinBox();
    m_spinStep2H->setRange(1, 50);
    m_spinStep2H->setValue(20.0);
    m_spinStep2H->setSuffix(QStringLiteral(" mm"));
    addRow(QStringLiteral("台阶 2 高度："), m_spinStep2H);

    m_spinVoxelSize = new QDoubleSpinBox();
    m_spinVoxelSize->setRange(0.05, 5.0);
    m_spinVoxelSize->setDecimals(2);
    m_spinVoxelSize->setValue(0.2);
    m_spinVoxelSize->setSuffix(QStringLiteral(" mm"));
    addRow(QStringLiteral("体素尺寸："), m_spinVoxelSize);

    m_spinRansacDist = new QDoubleSpinBox();
    m_spinRansacDist->setRange(0.1, 10.0);
    m_spinRansacDist->setValue(0.5);
    m_spinRansacDist->setSuffix(QStringLiteral(" mm"));
    addRow(QStringLiteral("RANSAC 阈值："), m_spinRansacDist);

    m_paramGroup->setLayout(layout);
}

void MainWindow::setupResultPanel() {
    m_resultGroup = new QGroupBox(QStringLiteral("点检结果"));
    auto* layout = new QVBoxLayout();
    m_resultView = new QTextEdit();
    m_resultView->setReadOnly(true);
    m_resultView->setHtml(QStringLiteral("<p style='color:#888'>等待点检...</p>"));
    m_resultView->setStyleSheet(
        "QTextEdit { background:#fafafa; border:1px solid #ccc; "
        "font-family:'Microsoft YaHei UI'; font-size:12px; }");
    layout->addWidget(m_resultView);
    m_resultGroup->setLayout(layout);
}

void MainWindow::setupImagePanel() {
    m_imageGroup = new QGroupBox(QStringLiteral("仿真图像与处理证据"));
    auto* layout = new QVBoxLayout();
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(4);

    m_imageTitle = new QLabel(QStringLiteral("等待图像..."));
    m_imageTitle->setStyleSheet("color:#666; font-weight:bold;");

    m_detailList = new QListWidget();
    m_detailList->setMinimumWidth(190);
    m_detailList->setMaximumWidth(260);
    m_detailList->setAlternatingRowColors(true);
    connect(m_detailList, &QListWidget::currentRowChanged,
            this, &MainWindow::onDetailSelectionChanged);

    auto createPreview = [](const QString& title, EvidenceView*& evidenceView) {
        auto* panel = new QWidget();
        auto* panelLayout = new QVBoxLayout(panel);
        panelLayout->setContentsMargins(0, 0, 0, 0);
        panelLayout->setSpacing(3);

        auto* toolbar = new QHBoxLayout();
        toolbar->setContentsMargins(0, 0, 0, 0);
        toolbar->setSpacing(2);
        auto* caption = new QLabel(title);
        caption->setStyleSheet("color:#555; font-weight:bold;");
        toolbar->addWidget(caption);
        toolbar->addStretch();

        evidenceView = new EvidenceView();
        auto addTool = [toolbar](const QString& text,
                                 const QString& toolTip,
                                 const std::function<void()>& action) {
            auto* button = new QToolButton();
            button->setText(text);
            button->setToolTip(toolTip);
            button->setAutoRaise(true);
            button->setFixedHeight(24);
            QObject::connect(button, &QToolButton::clicked, action);
            toolbar->addWidget(button);
        };
        addTool(QStringLiteral("-"), QStringLiteral("缩小"),
                [evidenceView]() { evidenceView->zoomOut(); });
        addTool(QStringLiteral("复位"), QStringLiteral("恢复初始视角或原始像素"),
                [evidenceView]() { evidenceView->resetView(); });
        addTool(QStringLiteral("适应"), QStringLiteral("适应窗口"),
                [evidenceView]() { evidenceView->fitView(); });
        addTool(QStringLiteral("+"), QStringLiteral("放大"),
                [evidenceView]() { evidenceView->zoomIn(); });

        panelLayout->addLayout(toolbar);
        panelLayout->addWidget(evidenceView, 1);
        return panel;
    };

    auto* comparison = new QSplitter(Qt::Horizontal);
    comparison->addWidget(
        createPreview(QStringLiteral("算法输入"), m_originalEvidenceView));
    comparison->addWidget(
        createPreview(QStringLiteral("处理结果"), m_processedEvidenceView));
    comparison->setChildrenCollapsible(false);
    comparison->setStretchFactor(0, 1);
    comparison->setStretchFactor(1, 1);

    m_detailView = new QTextEdit();
    m_detailView->setReadOnly(true);
    m_detailView->setMaximumHeight(112);
    m_detailView->setStyleSheet(
        "QTextEdit { background:#f7f7f7; border:1px solid #ccc; "
        "font-family:'Microsoft YaHei UI'; font-size:12px; }");
    m_detailView->setHtml(
        QStringLiteral("<span style='color:#888'>等待处理详情...</span>"));

    auto* content = new QWidget();
    auto* contentLayout = new QVBoxLayout(content);
    contentLayout->setContentsMargins(0, 0, 0, 0);
    contentLayout->setSpacing(4);
    contentLayout->addWidget(comparison, 1);
    contentLayout->addWidget(m_detailView);

    auto* detailSplitter = new QSplitter(Qt::Horizontal);
    detailSplitter->addWidget(m_detailList);
    detailSplitter->addWidget(content);
    detailSplitter->setChildrenCollapsible(false);
    detailSplitter->setStretchFactor(0, 0);
    detailSplitter->setStretchFactor(1, 1);
    detailSplitter->setSizes({210, 700});

    layout->addWidget(m_imageTitle);
    layout->addWidget(detailSplitter, 1);
    m_imageGroup->setLayout(layout);
}

void MainWindow::setupLogPanel() {
    m_logGroup = new QGroupBox(QStringLiteral("运行日志"));
    auto* layout = new QVBoxLayout();
    layout->setContentsMargins(4, 4, 4, 4);
    m_logView = new QTextEdit();
    m_logView->setReadOnly(true);
    m_logView->setLineWrapMode(QTextEdit::NoWrap);
    m_logView->setStyleSheet(
        "QTextEdit { background:#1e1e1e; color:#d4d4d4; border:1px solid #333; "
        "font-family:Consolas,'Microsoft YaHei UI'; font-size:12px; }");
    layout->addWidget(m_logView);
    m_logGroup->setLayout(layout);
}

void MainWindow::setupStatusBar() {
    m_statusIcon = new QLabel();
    m_statusText = new QLabel(QStringLiteral("就绪"));
    m_statusText->setStyleSheet("font-weight:bold;");
    statusBar()->addWidget(m_statusIcon);
    statusBar()->addWidget(m_statusText, 1);
}

void MainWindow::setRunning(bool running) {
    m_btnRun->setEnabled(!running);
    m_btnStop->setEnabled(running);
    m_paramGroup->setEnabled(!running);
    m_statusIcon->setText(running ? QStringLiteral("●") : QStringLiteral("○"));
    m_statusIcon->setStyleSheet(
        running ? "color:#2e7d32; font-size:14px;" : "color:#888; font-size:14px;");
    m_statusText->setText(running ? QStringLiteral("点检运行中") : QStringLiteral("就绪"));
}

void MainWindow::onRunClicked() {
    if (m_demoPath.isEmpty() || !QFileInfo::exists(m_demoPath)) {
        m_demoPath = QFileDialog::getOpenFileName(
            this, QStringLiteral("选择 demo_full_pipeline.exe"),
            QApplication::applicationDirPath(),
            QStringLiteral("可执行文件 (*.exe)"));
        if (m_demoPath.isEmpty()) {
            return;
        }
    }

    m_logView->clear();
    m_fullLog.clear();
    m_resultView->setHtml(QStringLiteral("<p style='color:#666'>点检运行中...</p>"));
    showImageWindow();

    const QString configPath = writeConfigFile();
    QStringList args;
    if (!configPath.isEmpty()) {
        args << QStringLiteral("--config") << configPath;
    }

    setRunning(true);
    m_progress->setValue(0);
    m_progress->setFormat(QStringLiteral("启动点检..."));
    m_logView->append(QStringLiteral("[启动] ") + m_demoPath + QStringLiteral("\n"));
    m_runner->start(m_demoPath, args);
}

void MainWindow::onStopClicked() {
    m_runner->stop();
    setRunning(false);
    m_progress->setFormat(QStringLiteral("已停止"));
    m_logView->append(QStringLiteral("\n[停止] 用户终止点检。"));
}

void MainWindow::onProcessOutput(const QString& text) {
    m_fullLog += text;
    m_logView->moveCursor(QTextCursor::End);
    m_logView->insertPlainText(text);
    m_logView->moveCursor(QTextCursor::End);

    const QStringList lines = text.split('\n', Qt::SkipEmptyParts);
    for (const QString& rawLine : lines) {
        const QString line = rawLine.trimmed();
        const ImageDetailRecord detail = ResultParser::parseImageDetail(line);
        if (detail.valid()) {
            appendImageDetail(detail);
            continue;
        }

        if (line.contains(QStringLiteral("[IMAGE "))) {
            const QString imagePath = ResultParser::parseImagePath(line);
            if (!imagePath.isEmpty()) {
                m_imageTitle->setText(
                    QStringLiteral("正在处理：%1")
                        .arg(QFileInfo(imagePath).completeBaseName()));
                displayEvidence(
                    m_originalEvidenceView, imagePath,
                    QStringLiteral("原图加载失败"));
                m_processedEvidenceView->clearContent(
                    QStringLiteral("等待算法处理结果"));
            }
        }

        const int progress = ResultParser::parseProgress(line);
        if (progress >= 0) {
            m_progress->setValue(progress);
            m_progress->setFormat(QStringLiteral("点检进度 %1%").arg(progress));
        }
    }
}

void MainWindow::onProcessFinished(int exitCode) {
    setRunning(false);
    parseResults(m_fullLog);
    const PipelineResult result = ResultParser::parse(m_fullLog);

    if (exitCode == 0 && result.completed && result.inspectionOk) {
        m_progress->setValue(100);
        m_progress->setFormat(QStringLiteral("点检通过"));
        m_statusText->setText(QStringLiteral("点检通过"));
        m_logView->append(QStringLiteral("\n[完成] 点检通过。"));
    } else {
        m_progress->setFormat(QStringLiteral("点检失败"));
        m_statusText->setText(QStringLiteral("点检失败"));
        m_logView->append(
            QStringLiteral("\n[失败] 退出码 %1，请查看上方错误日志。").arg(exitCode));
    }
}

void MainWindow::onProcessError(QProcess::ProcessError error) {
    Q_UNUSED(error);
    setRunning(false);
    m_progress->setFormat(QStringLiteral("点检错误"));
}

QString MainWindow::writeConfigFile() {
    const QString path = QDir::temp().filePath(QStringLiteral("lsc_gui_config.yaml"));
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        m_logView->append(QStringLiteral("[错误] 无法写入配置文件：") + path);
        return {};
    }

    QTextStream out(&file);
    out << "num_camera_images: " << m_spinCamImages->value() << '\n'
        << "num_lp_images: " << m_spinLpImages->value() << '\n'
        << "num_ma_steps: " << m_spinMaSteps->value() << '\n'
        << "ma_step_mm: " << m_spinMaStepMm->value() << '\n'
        << "block_width: " << m_spinBlockWidth->value() << '\n'
        << "block_depth: " << m_spinBlockDepth->value() << '\n'
        << "step1_h: " << m_spinStep1H->value() << '\n'
        << "step2_h: " << m_spinStep2H->value() << '\n'
        << "voxel_size: " << m_spinVoxelSize->value() << '\n'
        << "ransac_dist: " << m_spinRansacDist->value() << '\n';
    return path;
}

void MainWindow::parseResults(const QString& log) {
    const PipelineResult result = ResultParser::parse(log);
    if (!result.valid()) {
        m_resultView->setHtml(
            QStringLiteral("<p style='color:#c00'>未收到完整点检结果，请检查日志。</p>"));
        return;
    }

    const auto state = [](bool ok) {
        return ok
            ? QStringLiteral("<span style='color:#2e7d32;font-weight:bold'>通过</span>")
            : QStringLiteral("<span style='color:#c62828;font-weight:bold'>失败</span>");
    };

    QString html = QStringLiteral("<table style='width:100%;border-collapse:collapse'>");
    html += QStringLiteral("<tr><td><b>总体点检</b></td><td>%1</td></tr>")
                .arg(state(result.inspectionOk));
    html += QStringLiteral("<tr><td>相机标定</td><td>%1，指标 %2</td></tr>")
                .arg(state(result.cameraOk))
                .arg(result.cameraRms, 0, 'f', 3);
    html += QStringLiteral("<tr><td>光平面标定</td><td>%1，夹角 %2°</td></tr>")
                .arg(state(result.planeOk))
                .arg(result.planeAngle, 0, 'f', 3);
    html += QStringLiteral("<tr><td>移动轴标定</td><td>%1，夹角 %2°</td></tr>")
                .arg(state(result.axisOk))
                .arg(result.axisAngle, 0, 'f', 3);
    html += QStringLiteral("<tr><td>台阶 1</td><td>%1 mm</td></tr>")
                .arg(result.step1Height, 0, 'f', 3);
    html += QStringLiteral("<tr><td>台阶 2</td><td>%1 mm</td></tr>")
                .arg(result.step2Height, 0, 'f', 3);
    html += QStringLiteral("<tr><td>测量体积</td><td>%1 mm³</td></tr>")
                .arg(result.measVolume, 0, 'f', 1);
    html += QStringLiteral("<tr><td>真实体积</td><td>%1 mm³</td></tr>")
                .arg(result.gtVolume, 0, 'f', 1);
    html += QStringLiteral("<tr><td>总耗时</td><td>%1 s</td></tr>")
                .arg(result.totalTime, 0, 'f', 2);
    html += QStringLiteral("</table>");
    m_resultView->setHtml(html);
}

void MainWindow::showImageWindow() {
    m_imageDetails.clear();
    m_detailList->clear();
    m_currentImageIndex = -1;
    m_imageTitle->setText(QStringLiteral("等待图像..."));
    m_originalEvidenceView->clearContent(QStringLiteral("正在等待仿真图像..."));
    m_processedEvidenceView->clearContent(QStringLiteral("正在等待处理结果..."));
    m_detailView->setHtml(
        QStringLiteral("<span style='color:#888'>等待处理详情...</span>"));
}

void MainWindow::appendImageDetail(const ImageDetailRecord& detail) {
    if (!detail.valid()) {
        return;
    }

    m_imageDetails.append(detail);
    const QString listText =
        QStringLiteral("%1  #%2").arg(detail.title).arg(detail.index);
    m_detailList->addItem(listText);

    // 运行时自动跟随最新证据帧；结束后用户选择任意列表项即可回看。
    m_detailList->setCurrentRow(m_imageDetails.size() - 1);
}

void MainWindow::onDetailSelectionChanged(int row) {
    displayImageDetail(row);
}

void MainWindow::displayEvidence(
    EvidenceView* target, const QString& path, const QString& emptyText) {
    target->setContent(path, emptyText);
}

void MainWindow::displayImageDetail(int index) {
    if (index < 0 || index >= m_imageDetails.size()) {
        return;
    }

    m_currentImageIndex = index;
    const ImageDetailRecord& detail = m_imageDetails.at(index);
    displayEvidence(
        m_originalEvidenceView, detail.sourcePath,
        QStringLiteral("原始图像加载失败"));
    displayEvidence(
        m_processedEvidenceView, detail.processedPath,
        QStringLiteral("处理图像加载失败"));

    m_imageTitle->setText(
        QStringLiteral("%1  [%2/%3]")
            .arg(detail.title)
            .arg(index + 1)
            .arg(m_imageDetails.size()));

    m_detailView->setHtml(
        QStringLiteral(
            "<table style='width:100%;border-collapse:collapse'>"
            "<tr><td style='width:72px'><b>处理方法</b></td><td>%1</td></tr>"
            "<tr><td><b>关键数据</b></td><td>%2</td></tr>"
            "</table>")
            .arg(detail.algorithm.toHtmlEscaped(),
                 detail.summary.toHtmlEscaped()));
}
