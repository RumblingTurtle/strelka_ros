#include <gazebo_msgs/SetPhysicsProperties.h>
#include <ros/ros.h>

#define DEFAULT_MAX_STEP_SIZE 0.001
#define DEFAULT_MAX_UPDATE_RATE 1000
#define DEFAULT_PRECON_ITERS 0
#define DEFAULT_ITERS 500
#define DEFAULT_W 1.3
#define DEFAULT_RMS_ERROR_TOL 0
#define DEFAULT_CONTACT_SURFACE_LAYER 0.001
#define DEFAULT_CONTACT_MAX_CORRECTING_VEL 100
#define DEFAULT_CFM 0
#define DEFAULT_ERP 0.2
#define DEFAULT_MAX_CONTACTS 20
/**
 * @brief Slows down Gazebo simulation by calling /gazebo/set_physics_properties
 * service
 *
 * @param desiredSlowdown slowdown value from 0 to 1
 * @return false on failure
 */
bool slowdownGazebo(float desiredSlowdown = 1.0) {
  gazebo_msgs::SetPhysicsProperties srv;

  geometry_msgs::Vector3 gravity; // gravity vector (e.g. earth ~[0,0,-9.81])
  gravity.x = 0;
  gravity.y = 0;
  gravity.z = -9.81;
  srv.request.gravity = gravity;

  gazebo_msgs::ODEPhysics ode_config; // configurations for ODE

  ode_config.auto_disable_bodies =
      false; // enable auto disabling of bodies, default false
  ode_config.sor_pgs_precon_iters =
      DEFAULT_PRECON_ITERS; // preconditioning inner iterations when uisng
                            // projected Gauss Seidel
  ode_config.sor_pgs_iters =
      DEFAULT_ITERS; // inner iterations when uisng projected Gauss Seidel
  ode_config.sor_pgs_w = DEFAULT_W; // relaxation parameter when using projected
                                    // Gauss Seidel, 1 = no relaxation
  ode_config.sor_pgs_rms_error_tol =
      DEFAULT_RMS_ERROR_TOL; // rms error tolerance before stopping inner
                             // iterations

  ode_config.contact_surface_layer =
      DEFAULT_CONTACT_SURFACE_LAYER; // contact "dead-band" width
  ode_config.contact_max_correcting_vel =
      DEFAULT_CONTACT_MAX_CORRECTING_VEL; // contact maximum correction velocity

  ode_config.cfm = DEFAULT_CFM; // global constraint force mixing
  ode_config.erp = DEFAULT_ERP; // global error reduction parameter

  ode_config.max_contacts =
      DEFAULT_MAX_CONTACTS; // maximum contact joints between two geoms

  srv.request.ode_config = ode_config;

  srv.request.time_step = DEFAULT_MAX_STEP_SIZE; // dt in seconds
  srv.request.max_update_rate =
      DEFAULT_MAX_UPDATE_RATE *
      desiredSlowdown; //  throttle maximum physics update rate

  ros::service::call("/gazebo/set_physics_properties", srv);

  return true;
}
