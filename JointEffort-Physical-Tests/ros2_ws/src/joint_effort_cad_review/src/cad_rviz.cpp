// Project-local CAD viewer using the installed RViz widgets. Tear down displays
// while the ROS context is alive; signal handlers only request Qt event-loop exit.
#include <csignal>
#include <memory>
#include <map>
#include <QApplication>
#include <QTimer>
#include <rviz_common/logging.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
static volatile std::sig_atomic_t stop_requested = 0;
static void stop_handler(int) { stop_requested = 1; }
int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop_handler);
  std::signal(SIGTERM, stop_handler);
  rviz_common::set_logging_handlers(
    [](const std::string &m, const std::string &, size_t) {RCLCPP_DEBUG(rclcpp::get_logger("cad_rviz"), "%s", m.c_str());},
    [](const std::string &m, const std::string &, size_t) {RCLCPP_INFO(rclcpp::get_logger("cad_rviz"), "%s", m.c_str());},
    [](const std::string &m, const std::string &, size_t) {RCLCPP_WARN(rclcpp::get_logger("cad_rviz"), "%s", m.c_str());},
    [](const std::string &m, const std::string &, size_t) {RCLCPP_ERROR(rclcpp::get_logger("cad_rviz"), "%s", m.c_str());});
  rviz_common::install_rviz_rendering_log_handlers();
  QApplication app(argc, argv);
  QString config;
  for (int i = 1; i + 1 < argc; ++i) if (QString(argv[i]) == "-d") config = argv[i + 1];
  auto node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("cad_rviz");
  int result = 0;
  {
    rviz_common::VisualizationFrame frame(node);
    frame.setApp(&app);
    frame.setSplashPath("");
    frame.initialize(node, config);
    frame.show();
    std::map<std::string, std::string> last_status;
    QTimer status_timer;
    QObject::connect(&status_timer, &QTimer::timeout, [&]() {
      auto *group = frame.getManager()->getRootDisplayGroup();
      for (int i=0; i<group->numDisplays(); ++i) {
        auto *display = group->getDisplayAt(i);
        for (int k=0; k<display->numChildren(); ++k) {
          auto *status = dynamic_cast<rviz_common::properties::StatusProperty *>(display->childAt(k));
          if (status) {
            const auto name = display->getName().toStdString();
            const auto value = status->getName().toStdString();
            if (last_status[name] != value) {
              RCLCPP_INFO(rclcpp::get_logger("cad_rviz"), "%s: %s", name.c_str(), value.c_str());
              last_status[name] = value;
            }
          }
        }
      }
    });
    status_timer.start(1000);
    QTimer::singleShot(3000, &frame, [&]() {
      RCLCPP_INFO(rclcpp::get_logger("cad_rviz"), "CAD viewer initialized with %d displays", frame.getManager()->getRootDisplayGroup()->numDisplays());
    });
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() {
      if (stop_requested) app.quit();
    });
    timer.start(40);
    result = app.exec();
  }
  node.reset();
  rclcpp::shutdown();
  return result;
}
