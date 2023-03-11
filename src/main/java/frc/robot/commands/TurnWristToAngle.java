// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Arm.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnWristToAngle extends PIDCommand {
  private Wrist m_wrist;
  /** Creates a new TurnWristToAngle. */
  public TurnWristToAngle(Wrist m_wrist, double targetAngleDegrees) {
    super(
        // The controller that the command will use
        new PIDController(1, 0, 0.02),
        // This should return the measurement
        () -> m_wrist.getCurrentAngleRadians(),
        // This should return the setpoint (can also be a constant)
        () -> Units.degreesToRadians(targetAngleDegrees),
        // This uses the output
        output -> m_wrist.setPower(output));
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    this.m_wrist = m_wrist;
    getController().enableContinuousInput(0, 2*Math.PI);
    getController().setTolerance(Units.degreesToRadians(2));
  }
  
  @Override
  public void initialize() {
    System.out.println("turn wrist to angle start");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("turn wrist to angle finished");
    m_wrist.setPower(0);
  }
}
