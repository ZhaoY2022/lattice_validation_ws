#include "lattice_standalone_test/lane_change_panel.h"

#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>

namespace lattice_standalone_test
{

LaneChangePanel::LaneChangePanel(QWidget * parent)
  : rviz_common::Panel(parent)
  , left_btn_(nullptr)
  , right_btn_(nullptr)
{
}

LaneChangePanel::~LaneChangePanel() = default;

void LaneChangePanel::onInitialize()
{
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  lane_change_pub_ = raw_node->create_publisher<std_msgs::msg::Int8>(
      "/lattice_test/lane_change_cmd", rclcpp::QoS(5));

  auto * vbox = new QVBoxLayout(this);
  vbox->setContentsMargins(8, 8, 8, 8);

  auto * title = new QLabel("Lane Change Control");
  title->setAlignment(Qt::AlignCenter);
  QFont font = title->font();
  font.setBold(true);
  font.setPointSize(12);
  title->setFont(font);
  vbox->addWidget(title);

  auto * hbox = new QHBoxLayout();
  hbox->setSpacing(8);

  left_btn_ = new QPushButton();
  left_btn_->setText(QString::fromUtf8("\xe2\x86\x90  Left Lane\n(+3.75m)"));
  left_btn_->setMinimumHeight(50);
  left_btn_->setStyleSheet(
      "QPushButton { background-color: #4CAF50; color: white; font-weight: bold; "
      "font-size: 12px; border-radius: 4px; padding: 6px; }"
      "QPushButton:hover { background-color: #43A047; }"
      "QPushButton:pressed { background-color: #388E3C; }");
  connect(left_btn_, &QPushButton::clicked, this, [this]() {
    std_msgs::msg::Int8 msg;
    msg.data = 1;
    lane_change_pub_->publish(msg);
  });

  right_btn_ = new QPushButton();
  right_btn_->setText(QString::fromUtf8("Right Lane  \xe2\x86\x92\n(-3.75m)"));
  right_btn_->setMinimumHeight(50);
  right_btn_->setStyleSheet(
      "QPushButton { background-color: #2196F3; color: white; font-weight: bold; "
      "font-size: 12px; border-radius: 4px; padding: 6px; }"
      "QPushButton:hover { background-color: #1E88E5; }"
      "QPushButton:pressed { background-color: #1976D2; }");
  connect(right_btn_, &QPushButton::clicked, this, [this]() {
    std_msgs::msg::Int8 msg;
    msg.data = -1;
    lane_change_pub_->publish(msg);
  });

  hbox->addWidget(left_btn_);
  hbox->addWidget(right_btn_);
  vbox->addLayout(hbox);

  auto * hint = new QLabel("Scenario 5: S-curve highway");
  hint->setAlignment(Qt::AlignCenter);
  hint->setStyleSheet("color: gray; font-size: 9px;");
  vbox->addWidget(hint);
}

void LaneChangePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void LaneChangePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace lattice_standalone_test

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(lattice_standalone_test::LaneChangePanel, rviz_common::Panel)
