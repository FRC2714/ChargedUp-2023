// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.utils.controller.AsymmetricProfiledPIDCommand;
import frc.robot.utils.controller.AsymmetricProfiledPIDController;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends AsymmetricProfiledPIDCommand {
  /** Creates a new AutoBalance. */
  private DriveSubsystem m_drivetrain;
  public AutoBalance(DriveSubsystem m_drivetrain, boolean isReversed) {
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
        m_drivetrain::getPitchDegrees, //gyro X angle
        // This should return the goal (can also be a constant)
        new State(0, 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          m_drivetrain.drive( isReversed ? -output : output
            , 0, 0, true, false);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_drivetrain);
    this.m_drivetrain = m_drivetrain;
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
    m_drivetrain.drive(0, 0, 0, true, false);
    m_drivetrain.setX();
  }
}