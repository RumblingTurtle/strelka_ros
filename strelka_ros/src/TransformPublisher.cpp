#include <strelka_ros/TransformPublisher.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_transform_publisher");
  lcm::LCM lcm;
  ros::NodeHandle nh;

  if (!lcm.good())
    return 1;

  TransformPublisher handler(nh);
  lcm::Subscription *sub =
      lcm.subscribe(strelka::constants::ROBOT_STATE_TOPIC_NAME,
                    &TransformPublisher::handle, &handler);

  while (0 == lcm.handle() && ros::ok()) {
  };

  lcm.unsubscribe(sub);
  return 0;
}