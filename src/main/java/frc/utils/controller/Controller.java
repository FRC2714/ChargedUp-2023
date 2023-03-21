package frc.utils.controller;

import edu.wpi.first.util.sendable.Sendable;

public interface Controller extends Sendable {

  /**
   * Set the controller target reference and run it
   *
   * @param setpoint is the controller setpoint
   * @param measurement is the process variable
   */
  default void setReference(double setpoint, double measurement) {
    setReference(setpoint, measurement, 0.0);
  }

  /**
   * Set the controller target reference and run it
   *
   * @param setpoint controller setpoint
   * @param measurement process variable
   * @param feedforward feedforward value
   */
  void setReference(double setpoint, double measurement, double feedforward);

  /**
   * Enables continuous input.
   *
   * <p>Rather then using the max and min input range as constraints, it considers them to be the
   * same point and automatically calculates the shortest route to the setpoint.
   *
   * @param minimumInput The minimum value expected from the input.
   * @param maximumInput The maximum value expected from the input.
   */
  void enableContinuousInput(double minimumInput, double maximumInput);

  /** Disable continuous input */
  void disableContinuousInput();

  /**
   * Returns true if continuous input is enabled.
   *
   * @return True if continuous input is enabled.
   */
  public boolean isContinuousInputEnabled();
}