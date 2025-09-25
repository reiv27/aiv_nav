#pragma once

#include <memory>
#include <string_view>

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
  explicit Controller(std::unique_ptr<State> initial)
    : state_(std::move(initial))
  {
  }
  
  Controller(const Controller&) = delete;
  Controller& operator=(const Controller&) = delete;
  Controller(Controller&&) = default;
  Controller& operator=(Controller&&) = default;
  
  /**
   * @brief Update the controller state
   */
  void update();

  /**
   * @brief Set a new state
   * @param s New state to set
   */
  void setState(std::unique_ptr<State> s);

  /**
   * @brief Get current state
   * @return Reference to current state
   */
  const State& state() const;

  /**
   * @brief Increment internal counter
   */
  void incrementCount();

  /**
   * @brief Get current counter value
   * @return Current counter value
   */
  int count() const;

private:
  std::unique_ptr<State> state_;
  int count_ = 0;
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