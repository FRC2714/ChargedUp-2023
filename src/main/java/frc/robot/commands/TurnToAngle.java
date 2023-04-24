// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.utils.controller.AsymmetricProfiledPIDCommand;
import frc.robot.utils.controller.AsymmetricProfiledPIDController;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.Constraints;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToAngle extends AsymmetricProfiledPIDCommand {
  
  /** Creates a new Autoalign. */
  public TurnToAngle(DriveSubsystem m_drivetrain, double targetAngleDegrees) {
    super(
        // The ProfiledPIDController used by the command
        new AsymmetricProfiledPIDController(
            // The PID gains
            2.0,
            0,
            0,
            // The motion profile constraints
            new Constraints(5.0, 12.0, 7.0)), //rad/s, rad/s^2
        // This should return the measurement
        m_drivetrain::getHeadingRadians,
        // This should return the goal (can also be a constant)
        Units.degreesToRadians(targetAngleDegrees),
        // This uses the output
        (output, setpoint) -> m_drivetrain.drive(0, 0, output, true, false)
          // Use the output (and setpoint, if desired) here
        );
        addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Units.degreesToRadians(1.5));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atGoal();
  }

  @Override
  public void end(boolean interrupted) {}
}
