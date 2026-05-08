#ifndef REACTIVE_CIRCUMNAV_CONTROLLER_HPP
#define REACTIVE_CIRCUMNAV_CONTROLLER_HPP

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>

#include "reactive_circumnav/companion_disk.hpp"
#include "reactive_circumnav/states.hpp"

/**
 * @brief Main controller class
 * @param linear_velocity Linear velocity
 * @param angular_velocity Angular velocity
 * @param rho_0 rho_0
 * @param R_epsilon R_epsilon
 * @param R_vis LiDAR visibility radius
 * @param resolution LiDAR resolution
 * @param lidar_angle_offset LiDAR angle offset
 * @param window_size Window size
 * @param nu Nu
 */
class Controller final
{
public:
  Controller();
  Controller(double linear_velocity,
             double angular_velocity,
             double rho_0,
             double R_epsilon,
             double R_vis,
             int resolution,
             double lidar_angle_offset,
             uint64_t window_size=0,
             double nu=1.0,
             int curvature_points_size=0,
             double k1=1.0,
             double k2=1.0,
             const std::string& mode_c_control_type="relay");

  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) = default;
  Controller& operator=(Controller&&) = default;

  /**
   * @brief Set a new state
   * @param s New state to set
   */
  void set_state(std::unique_ptr<State> s);

  /**
   * @brief Get current state
   * @return Reference to current state
   */
  const State& state() const;

  /**
   * @brief Update the controller state
   */
  void update(const std::vector<double>& robot_state, const std::vector<double>& lidar_data);

  /**
   * @brief Get output control signal
   */
  double get_control_signal() const;

  /**
   * @brief Get robot state
   */
  const std::vector<double>& get_robot_state() const;

  /**
   * @brief Get min distance from lidar data
   */
  double get_min_dist() const;

  /**
   * @brief Get lidar points
   */
  const std::vector<std::vector<double>>& get_lidar_points() const;

  /**
   * @brief Get lidar closest point
   */
  const std::vector<double>& get_closest_lidar_point() const;

  /**
   * @brief Get angular velocity
   */
  double get_angular_velocity() const;

  /**
   * @brief Get linear velocity
   */
  double get_linear_velocity() const;

  /**
   * @brief Nominal max linear speed from config (unchanged by curv braking).
   */
  double get_linear_velocity_max() const;

  /**
   * @brief Get R_min
   */
  double get_R_min() const;

  /**
   * @brief Get rho_0
   */
  double get_rho_0() const;

  /**
   * @brief Get companion disk pose
   */
  const std::vector<double>& get_disk_pose() const;

  /**
   * @brief Get dt
   */
  double get_dt();

  /**
   * @brief Get dR_prev
   */
  double get_dR_prev() const;

  /**
   * @brief Set dR_prev
   */
  void set_dR_prev(double dR_prev);

  /**
   * @brief Get dR_dot (derivative of dR, set by control logic)
   */
  double get_dR_dot() const;

  /**
   * @brief Set dR_dot (used by states when computing control)
   */
  void set_dR_dot(double dR_dot);

  /**
   * @brief Get nu
   */
  double get_nu() const;

  /**
   * @brief Get Mode C control type ("relay", "sta", "curv", "barrier", …)
   */
  const std::string& get_mode_c_control_type() const;

  /**
   * @brief Get k1 (STA gain)
   */
  double get_k1() const;

  /**
   * @brief Set k1
   */
  void set_k1(double k1);

  /**
   * @brief Get k2 (STA gain)
   */
  double get_k2() const;

  /**
   * @brief Set k2
   */
  void set_k2(double k2);

  /**
   * @brief Get integral (STA integral term)
   */
  double get_integral() const;

  /**
   * @brief Set integral
   */
  void set_integral(double integral);

  /**
   * @brief Compute Mode C control signal using the function selected at construction.
   */
  double compute_mode_c_control();

  /**
   * @brief Get disk min length
   */
  double get_disk_min_ray_length() const;

  /**
   * @brief Get disk min arg
   * @return Disk min arg
   */
  uint64_t get_disk_min_arg() const;

  /**
   * @brief Set verA pose
   * @param verA New verA pose (x, y)
   */
  void set_verA(const std::vector<double>& verA);

  /**
   * @brief Get verA pose
   * @return VerA pose (x, y)
   */
  const std::vector<double>& get_verA() const;

  /**
   * @brief Set gap points
   * @param gap_point_1 New gap point 1 (x, y)
   * @param gap_point_2 New gap point 2 (x, y)
   */
  void set_gap_points(const std::vector<double>& gap_point_1,
                      const std::vector<double>& gap_point_2);

  /**
   * @brief Get gap point 1
   * @return Gap point 1 (x, y)
   */
  const std::vector<double>& get_gap_point_1() const;

  /**
   * @brief Get gap point 2
   * @return Gap point 2 (x, y)
   */
  const std::vector<double>& get_gap_point_2() const;

  /**
   * @brief Estimate local obstacle curvature (3-point method, non-negative).
   *
   * @param kappa_out Output curvature. Set to 0.0 if invalid.
   * @param break_jump_m Break segment if neighbor distance exceeds this value.
   * @param half_window Rays to extend left/right from closest ray; if negative, from curvature_points_size.
   * @return True if curvature is valid, false otherwise.
   */
  bool estimate_curvature(double& kappa_out,
                          double break_jump_m = 0.3,
                          int half_window = -1) const;

  /**
   * @brief Get last computed curvature value.
   */
  double get_curvature() const;

  /**
   * @brief Check if last curvature estimate is valid.
   */
  bool is_curvature_valid() const;

  /**
   * @brief Set linear velocity
   * @param linear_velocity New linear velocity
   */
  void set_linear_velocity(double linear_velocity);

private:
  std::unique_ptr<State> state_{std::make_unique<ModeA>()};

  // Controller parameters
  double linear_velocity_;
  /** Upper cap from parameters; curv mode must not use linear_velocity_ for this. */
  double linear_velocity_max_{0.0};
  double angular_velocity_;
  double R_min_;
  double rho_0_;
  double R_vis_;

  // Robot states
  std::vector<double> robot_state_{};

  // Lidar data
  uint64_t resolution_;
  double lidar_angle_offset_{0.0};
  std::vector<double> lidar_data_{};
  std::vector<std::vector<double>> lidar_points_{};
  std::vector<double> lidar_closest_point_{0.0, 0.0};
  double min_dist_{0.0};

  // Companion disk
  CompanionDisk disk_;

  // Control parameters
  double dt_{0.0};
  std::chrono::steady_clock::time_point t_prev_;
  bool t_initialized_{false};
  double dR_prev_{0.0};
  double dR_dot_{0.0};
  double u_{0.0};
  double nu_;
  std::string mode_c_control_type_{"relay"};
  std::function<double(Controller&)> mode_c_control_fn_{};
  std::vector<double> verA_{0.0, 0.0};
  std::vector<double> gap_point_1_{0.0, 0.0};
  std::vector<double> gap_point_2_{0.0, 0.0};

  /**
   * @brief Set output control signal
   * @param u New output control signal
   */
  void set_control_signal_(double u);

  //TODO: Change name  set_robot_state() to something
  /**
   * @brief Set new robot state (x, y, theta) !!!NOT FSM STATE!!!
   * @param robot_state New robot state
   */
  void set_robot_state_(const std::vector<double>& robot_state);

  /**
   * @brief Set new lidar data
   * @param lidar_data New lidar data
   */
  void set_lidar_data_(const std::vector<double>& lidar_data);

  /**
   * @brief Calculate moving average of the control signal
   * @param u Control signal
   * @return Moving average of the control signal
   */

  // Реализации законов управления для режима C (выбор по имени в конструкторе)
  static double relay_mode_c_control_(Controller& ctrl);
  static double barrier_mode_c_control_(Controller& ctrl);
  static double sta_mode_c_control_(Controller& ctrl);
  static double curv_mode_c_control_(Controller& ctrl);
  static std::function<double(Controller&)> get_mode_c_control_fn_(const std::string& name);

  // Curvature of obstacles
  std::vector<double> curvature_points_{};
  double curvature_{0.0};
  bool curvature_valid_{false};

  // Super-Twisting Controller
  double k1_{0.0};
  double k2_{0.0};
  double integral_{0.0};
};

#endif  // REACTIVE_CIRCUMNAV_CONTROLLER_HPP
