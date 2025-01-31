#include <QApplication>
#include <memory>
#include "qt_gui_ros2/mainWindow.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    rclcpp::init(argc, argv);

    auto ros_node_abs = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("rviz_render_node");

    auto mainWindow = std::make_shared<MainWindow>(&app, ros_node_abs);
    mainWindow->show();

    while (rclcpp::ok()) {
        app.processEvents();
    }

    return 0;
}

