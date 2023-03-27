// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Superstructure.CargoType;
import frc.robot.subsystems.Superstructure.ScoreMode;
import frc.robot.subsystems.Superstructure;

public class ScoreCommand extends CommandBase {
  private Superstructure m_superstructure;
  private Claw m_claw;

  /** Creates a new ScoreCommand. */
  public ScoreCommand(Superstructure m_superstructure, Claw m_claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_superstructure = m_superstructure;
    this.m_claw = m_claw;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_superstructure.getScoreMode() == ScoreMode.ARM) {
      if (m_superstructure.getCargoType() == CargoType.CONE) {
        m_claw.stop();
        m_claw.open();
      } else {
        m_claw.shoot();
      }
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
