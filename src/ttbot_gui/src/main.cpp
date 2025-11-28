#include <QApplication>
#include "ttbot_gui/main_window.hpp"
#include "ttbot_gui/ros_worker.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    RosWorker worker;
    MainWindow window(&worker);
    window.show();

    return app.exec();
}