#pragma once

#include <string_view>

class Controller;

/**
 * @brief Enum class for state names
 */
 enum class StateName {
  ModeA,
  ModeC,
  ModeG
};

/**
 * @brief Abstract base class for all controller states
 */
class State
{
public:
  virtual ~State() = default;
  virtual void handle(Controller& context) = 0;
  virtual std::string_view name() const = 0;
  virtual StateName state_name() const = 0;

  /**
   * @brief Calculate the control signal
   * @param context Controller context
   * @return Control signal
   */
  virtual double calculate_control_signal(Controller& context) = 0;
};

/**
 * @brief Controller Approaching Mode A implementation
 */
 class ModeA : public State
 {
 public:
   void handle(Controller& context) override;
   std::string_view name() const override;
   StateName state_name() const override;
 
   double calculate_control_signal(Controller& context) override;
 
 private:
   bool goal_flag_{true};
   double goal_angle_{0.0};
 };
 
 /**
  * @brief Controller Contact Mode C implementation
  */
 class ModeC : public State
 {
 public:
   void handle(Controller& context) override;
   std::string_view name() const override;
   StateName state_name() const override;
 
   double calculate_control_signal(Controller& context) override;
 };
 
 /**
  * @brief Controller Gap Mode G implementation
  */
 class ModeG : public State
 {
 public:
   void handle(Controller& context) override;
   std::string_view name() const override;
   StateName state_name() const override;

   double calculate_control_signal(Controller& context) override;
 };
