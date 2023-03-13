#include <strelka_ros/A1/CommandPublisher.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "a1_command_publisher");

  LcmHandler listen_publish_obj("a1");
  ros::AsyncSpinner spinner(1); // one thread
  spinner.start();
  usleep(300000); // must wait 300ms, to get first state

  ros::NodeHandle n;

  if (!listen_publish_obj.lcm.good())
    return 1;

  ros::Rate loop_rate(1000);

  while (ros::ok()) {
    listen_publish_obj.lcm.publish("raw_state", &(listen_publish_obj.state));
    loop_rate.sleep();
  }

  return 0;
}
