// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.ShooterScoreLevel;

public class ShooterStateMachine extends SubsystemBase {
  Shooter m_shooter;

  public enum ShooterState {
    RETRACT, HOLD, DYNAMIC, DOWN
  }

  public ShooterState shooterState = ShooterState.RETRACT;

  /** Creates a new ShooterStateMachine. */
  public ShooterStateMachine(Shooter m_shooter) {
    this.m_shooter = m_shooter;
  }

  public Command toRetract() {
    return new ParallelCommandGroup(
      m_shooter.setDynamicEnabledCommand(false),
      m_shooter.stopCommand(),
      m_shooter.pivotToRetract()
    );
  }

  public Command toHold() {
    return new ParallelCommandGroup(
      m_shooter.setDynamicEnabledCommand(false),
      m_shooter.pivotToHold()
    );
  }

  public Command toDynamic() {
    return new ParallelCommandGroup(
      m_shooter.setDynamicEnabledCommand(true)
    );
  }

  public Command toDown(Command ScoreLevelPivotComand) {
    return new ParallelCommandGroup(
      m_shooter.setDynamicEnabledCommand(false),
      ScoreLevelPivotComand
    );
  }

  public Command getShooterCommand(ShooterScoreLevel shooterScorelevel) {
    switch(shooterState) {
      case RETRACT: return toRetract();
      case HOLD: return toHold();
      case DYNAMIC: return toDynamic();
      case DOWN: 
        switch(shooterScorelevel) {
          case INTAKE: return toDown(m_shooter.intakeSequence());
          case OUTTAKE: return toDown(m_shooter.outtakeSequence());
          case SHOOT: return toDown(m_shooter.shootSequence());
        };
    }
    return new InstantCommand();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
