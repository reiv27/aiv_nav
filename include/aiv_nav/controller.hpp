#pragma once

#include <memory>
#include <string_view>

#include "aiv_nav/companion_disk.hpp"

class Controller;

/**
 * @brief Abstract base class for all controller states
 */
class State
{
public:
  virtual ~State() = default;
  virtual void handle(Controller& context) = 0;
  virtual std::string_view name() const = 0;
};

/**
 * @brief Main controller class
 */
class Controller final
{
public:
  Controller(std::unique_ptr<State> init_state,
             double linear_velocity,
             double angular_velocity,
             double rho_0,
             double R_epsilon,
             int resolution);
  
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
   * @brief Get lidar data
   */
  const std::vector<double>& get_lidar_data() const;

  /**
   * @brief Increment internal counter
   */
  void increment_count();

  /**
   * @brief Get current counter value
   * @return Current counter value
   */
  int count() const;

private:
  std::unique_ptr<State> state_;

  // Controller parameters
  const double linear_velocity_;
  const double angular_velocity_;
  double R_min_;
  const double rho_0_;

  // Robot states
  std::vector<double> robot_state_{0.0, 0.0, 0.0}; // [x, y, theta]
  std::vector<double> lidar_data_;

  // Companion disk
  CompanionDisk disk_;

  double u_ = 0.0;  // Output control signal

  int count_ = 0;

  /**
   * @brief Set output control signal
   * @param u New output control signal
   */
   void set_control_signal_(double u);

   /**
   * @brief Set new robot state
   * @param robot_state New robot state
   */
   void set_robot_state_(const std::vector<double>& robot_state);

   /**
   * @brief Set new lidar data
   * @param lidar_data New lidar data
   */
   void set_lidar_data_(const std::vector<double>& lidar_data);
};

/**
 * @brief Controller Approaching Mode A implementation
 */
class ModeA : public State
{
public:
  void handle(Controller& context) override;
  std::string_view name() const override;
};

/**
 * @brief Controller Contact Mode C implementation
 */
class ModeC : public State
{
public:
  void handle(Controller& context) override;
  std::string_view name() const override;
};

/**
 * @brief Controller Gap Mode G implementation
 */
class ModeG : public State
{
public:
  void handle(Controller& context) override;
  std::string_view name() const override;
};