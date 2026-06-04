#ifndef LATTICE_STANDALONE_TEST__LANE_CHANGE_PANEL_H_
#define LATTICE_STANDALONE_TEST__LANE_CHANGE_PANEL_H_

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>
#include <QPushButton>

namespace lattice_standalone_test
{

class LaneChangePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit LaneChangePanel(QWidget * parent = nullptr);
  ~LaneChangePanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  QPushButton * left_btn_;
  QPushButton * right_btn_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr lane_change_pub_;
};

}  // namespace lattice_standalone_test

#endif  // LATTICE_STANDALONE_TEST__LANE_CHANGE_PANEL_H_
