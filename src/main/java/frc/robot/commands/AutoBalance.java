// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.controller.AsymmetricProfiledPIDCommand;
import frc.robot.utils.controller.AsymmetricProfiledPIDController;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends AsymmetricProfiledPIDCommand {
  /** Creates a new AutoBalance. */
  private DriveSubsystem drivetrain;
  public AutoBalance(DriveSubsystem drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new AsymmetricProfiledPIDController(
            // The PID gains
            0.006,
            0,
            0.001,
            // The motion profile constraints
            new Constraints(1, 1, 1)), //deg/s, deg/s^2
        // This should return the measurement
        drivetrain::getPitchDegrees, //gyro X angle
        // This should return the goal (can also be a constant)
        new State(0, 0), //TODO
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.drive(output, 0, 0, true, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    getController().setTolerance(2);

  }
  
  public void initialize() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  public void end() {
    drivetrain.drive(0, 0, 0, true, false);
    drivetrain.setX();
  }
}