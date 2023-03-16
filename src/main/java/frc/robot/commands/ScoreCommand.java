// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;

public class ScoreCommand extends CommandBase {
  private ArmStateMachine m_armStateMachine;
  private Claw m_claw;

  /** Creates a new ScoreCommand. */
  public ScoreCommand(ArmStateMachine m_armStateMachine, Claw m_claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_armStateMachine = m_armStateMachine;
    this.m_claw = m_claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_armStateMachine.getCargoType() == CargoType.CONE) {
      m_claw.stop();
      m_claw.open();
    } else {
      m_claw.shoot();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
