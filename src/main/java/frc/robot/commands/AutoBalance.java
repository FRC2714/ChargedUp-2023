// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBalance extends ProfiledPIDCommand {
  /** Creates a new AutoBalance. */
  private DriveSubsystem drivetrain;
  private boolean atSetpoint;
  public AutoBalance(DriveSubsystem drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            0.008,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)),
        // This should return the measurement
        drivetrain::getPitchDegrees, //gyro X angle
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          drivetrain.drive(output, 0, 0, false, true);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    getController().setTolerance(1); //degrees

    if (getController().atSetpoint()) {
      atSetpoint = true;
    }
  }
  
  public void initialize() {
    System.out.println("autobalance start");
    atSetpoint = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetpoint;
  }

  public void end() {
    System.out.println("autobalance end");
    drivetrain.setX();
  }
}