// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Autoalign extends ProfiledPIDCommand {
  private DriveSubsystem drivetrain;
  private Limelight limelight;
  private boolean atSetpoint;
  
  /** Creates a new Autoalign. */
  public Autoalign(DriveSubsystem drivetrain, Limelight limelight) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            1, //todo
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)),
        // This should return the measurement
        limelight::getXRadianOffset,
        // This should return the goal (can also be a constant)
        () -> 0,
        // This uses the output
        (output, setpoint) -> drivetrain.drive(output, 0, 0, true) //could be y
          // Use the output (and setpoint, if desired) here
        );
        addRequirements(drivetrain);
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  public void initialize() {
    atSetpoint = false;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0, true);
  }
}
