// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Superstructure.ShooterScoreLevel;

public class ShooterStateMachine extends SubsystemBase {
  Shooter m_shooter;

  public enum ShooterState {
    RETRACT, HOLD, FRONT, BACK
  }

  public ShooterState shooterState = ShooterState.RETRACT;

  /** Creates a new ShooterStateMachine. */
  public ShooterStateMachine(Shooter m_shooter) {
    this.m_shooter = m_shooter;
  }

  public Command setShooterStateCommand(ShooterState shooterState, ShooterScoreLevel scoreLevel) {
    return new InstantCommand(() -> setShooterState(shooterState, scoreLevel));
  }

  public void setShooterState(ShooterState shooterState, ShooterScoreLevel scoreLevel) {
    if (this.shooterState != shooterState) {
      this.shooterState = shooterState;
      getShooterCommand(scoreLevel).schedule();
    }
  }

  public Command toRetract() {
    return new ParallelCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, false),
      m_shooter.stopCommand(),
      m_shooter.pivotToRetract()
    );
  }

  public Command toHold() {
    return new ParallelCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, false),
      m_shooter.pivotToHold()
    );
  }

  public Command toFront(Command ScoreLevelPivotComand) {
    return new ParallelCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, false),
      ScoreLevelPivotComand
    );
  }

  private Command nothingCommand() {
    return new InstantCommand();
  }

  public Command getShooterCommand(ShooterScoreLevel shooterScorelevel) {
    switch(shooterState) {
      case RETRACT: return toRetract();
      case HOLD: return toHold();
      case BACK: return m_shooter.setDynamicEnabledCommand(true, true);
      case FRONT: 
        switch(shooterScorelevel) {
          case INTAKE: return toFront(m_shooter.intakeSequence());
          case OUTTAKE: return toFront(m_shooter.outtakeSequence());
          case DYNAMIC: return m_shooter.setDynamicEnabledCommand(true, false);
        };
    }
    return nothingCommand();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Shooter State", shooterState.toString());
  }
}
