// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotIntake extends PIDCommand {
  Intake m_intake;
  /** Creates a new PivotIntake. */
  public PivotIntake(Intake m_intake, double targetAngleDegrees) {
    super(
        // The controller that the command will use
        new PIDController(0.23, 0, 0),
        // This should return the measurement
        () -> m_intake.getPivotAngleRadians(),
        // This should return the setpoint (can also be a constant)
        () -> Units.degreesToRadians(targetAngleDegrees),
        // This uses the output
        output -> m_intake.setPivotPower(output));
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.m_intake = m_intake;
    addRequirements(m_intake);
    getController().disableContinuousInput();
    getController().setTolerance(Units.degreesToRadians(4));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}