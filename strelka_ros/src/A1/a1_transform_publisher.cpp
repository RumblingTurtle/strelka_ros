#include <strelka_ros/A1/a1_transform_publisher.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_transform_publisher");
  lcm::LCM lcm;
  ros::NodeHandle nh;

  if (!lcm.good())
    return 1;

  Handler handler(nh);
  lcm::Subscription *sub =
      lcm.subscribe(strelka::A1::constants::ROBOT_STATE_TOPIC_NAME,
                    &Handler::handle, &handler);

  while (0 == lcm.handle() && ros::ok())
    ;

  lcm.unsubscribe(sub);
  return 0;
}