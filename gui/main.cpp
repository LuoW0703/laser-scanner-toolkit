/**
 * @file    main.cpp
 * @brief   3D 线激光轮廓仪标定工具 - Qt GUI 入口
 *
 * 通过子进程方式调用 demo_full_pipeline.exe，
 * 解决 Qt(MinGW) 与 OpenCV(MSVC) 编译器不兼容问题。
 */

#include <QApplication>
#include <QDir>
#include "MainWindow.h"

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    app.setApplicationName("Laser Scanner Toolkit");
    app.setApplicationVersion("1.0");
    app.setOrganizationName("LSC");

    // 全局中文字体支持
    QFont font = app.font();
    font.setPointSize(10);
    app.setFont(font);

    MainWindow w;
    w.resize(1100, 720);
    w.setWindowTitle("3D 线激光轮廓仪标定工具 v1.0");
    w.show();

    return app.exec();
}
