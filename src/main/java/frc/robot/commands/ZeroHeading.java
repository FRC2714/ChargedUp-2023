// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroHeading extends ProfiledPIDCommand {
  
  /** Creates a new Autoalign. */
  public ZeroHeading(DriveSubsystem drivetrain) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)),
        // This should return the measurement
        drivetrain::getHeadingRadians,
        // This should return the goal (can also be a constant)
        0,
        // This uses the output
        (output, setpoint) -> drivetrain.drive(0, 0, output, true, false)
          // Use the output (and setpoint, if desired) here
        );
        addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Units.degreesToRadians(1), 0);
  }

  public void initialize() {
  }

  public void execute() {
    SmartDashboard.putNumber("zero heading position error", getController().getPositionError());
    SmartDashboard.putNumber("zero heading tolerance", getController().getPositionTolerance());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("zero heading");
  }
}
