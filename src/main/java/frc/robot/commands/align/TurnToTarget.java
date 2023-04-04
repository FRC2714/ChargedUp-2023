// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.controller.AsymmetricProfiledPIDCommand;
import frc.robot.utils.controller.AsymmetricProfiledPIDController;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.State;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends AsymmetricProfiledPIDCommand {
  
  /** Creates a new Autoalign. */
  public TurnToTarget(DriveSubsystem drivetrain, Limelight m_limelight) {
    super(
        // The ProfiledPIDController used by the command
        new AsymmetricProfiledPIDController(
            // The PID gains
            1,
            0,
            0,
            // The motion profile constraints
            new Constraints(1, 1, 1)), //rad/s, rad/s^2
        // This should return the measurement
        m_limelight::getXOffsetRadians,
        // This should return the goal (can also be a constant)
        new State(0, 0),
        // This uses the output
        (output, setpoint) -> drivetrain.drive(0, 0, output, true, false)
          // Use the output (and setpoint, if desired) here
        );
        addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().enableContinuousInput(-Math.PI, Math.PI);
    getController().setTolerance(Units.degreesToRadians(1.5));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {}
}
