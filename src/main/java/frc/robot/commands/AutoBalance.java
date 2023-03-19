// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {
  /** Creates a new AutoBalance. */
  private DriveSubsystem drivetrain;
  public AutoBalance(DriveSubsystem drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.006,
            0,
            0.0005,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(1, 1)),
        // This should return the measurement
        drivetrain::getPitchDegrees, //gyro X angle
        // This should return the goal (can also be a constant)
        -2,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.drive(output, 0, 0, false, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    getController().setTolerance(2); //degrees

  }
  
  public void initialize() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  public void end() {
    drivetrain.setX();
  }
}