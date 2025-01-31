#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QVBoxLayout>
#include <QApplication>
#include <QWidget>
#include <QToolButton>
#include <QPushButton>
#include <QTimer>
#include <QLineEdit>
#include <QLabel>
#include <QCloseEvent>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/window_manager_interface.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace rviz_common {
class Display;
}

class MainWindow : public QMainWindow, public rviz_common::WindowManagerInterface {
    Q_OBJECT

public:
    MainWindow(QApplication *app, rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node, QWidget *parent = nullptr);
    ~MainWindow();

    QWidget *getParentWindow() override;
    rviz_common::PanelDockWidget *addPane(const QString &name, QWidget *pane, Qt::DockWidgetArea area, bool floating) override;
    void setStatus(const QString &message) override;

protected:
    void closeEvent(QCloseEvent *event) override;

private slots:
    void sendJoystickCommand();              // Sends cmd_vel based on button input
    void updateFrame();                      // Slot to update the reference frame
    void updateMapReceivedIndicator(bool received);  // Updates the map received indicator in the GUI

private:
    void initializeRViz();                   // Initializes RViz components
    //void DisplayGrid();                      // Sets up the grid and TF displays
    //void setupRobotModel();                  // Sets up the robot model display
    void setupJoystickControls();            // Initializes joystick buttons for movement control
    //void setupMapSubscriber();               // Sets up the map subscriber to listen for map data
    
    
    
    void setupGridDisplay();
    void setupTFDisplay();
    void setupMapDisplay();
    void setupRobotModelDisplay();
    void setupMapSubscriber();
    void setupLaserScanDisplay();

    QApplication *app_;
    QWidget *centralWidget_;
    QVBoxLayout *mainLayout_;
    QLineEdit *frameLineEdit_;               // Text box for the reference frame
    QLabel *mapReceivedIndicator_;           // Indicator for map reception status
    
    rviz_common::RenderPanel *renderPanel_;
    rviz_common::Display *grid_;             // Grid display object
    rviz_common::Display *tf_display_;       // TF display object
    rviz_common::Display *map_display_;      // Map display object
    rviz_common::Display *robot_model_display_; // RobotModel display object
    rviz_common::VisualizationManager *manager_;

    // ROS node and publisher for /cmd_vel
    rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher_;
    geometry_msgs::msg::Twist currentTwist_;

    // Joystick buttons
    QPushButton *forwardButton_;
    QPushButton *backwardButton_;
    QPushButton *leftButton_;
    QPushButton *rightButton_;
    QPushButton *stopButton_;
    
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscriber_; // Subscriber for map data
    bool mapReceived_;                       // Boolean flag to track map data reception
};

#endif // MAINWINDOW_HPP

