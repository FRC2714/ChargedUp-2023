// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class ShooterStateMachine {
  Shooter m_shooter;

  public enum ShooterState {
    RETRACT, HOLD, FRONT, BACK
  }

  public enum ShooterScoreLevel {
    INTAKE, OUTTAKE, DYNAMIC
  }

  public ShooterState shooterState = ShooterState.RETRACT;
  public ShooterScoreLevel scoreLevel = ShooterScoreLevel.INTAKE;

  /** Creates a new ShooterStateMachine. */
  public ShooterStateMachine(Shooter m_shooter) {
    this.m_shooter = m_shooter;
  }

  public Command setShooterStateCommand(ShooterState shooterState) {
    return new InstantCommand(() -> {
        System.out.println("setting target armstate");
        if (this.shooterState != shooterState) {
          this.shooterState = shooterState;
        }
        getShooterCommand(scoreLevel).withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
      }
    );
  }

  //Score level
  public Command setShooterScoreLevelCommand(ShooterScoreLevel scoreLevel) {
    return new InstantCommand(() -> this.scoreLevel = scoreLevel);
  }

  public ShooterScoreLevel getShooterScoreLevel() {
    return this.scoreLevel;
  }

  private Command toRetract() {
    return new SequentialCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, false),
      m_shooter.stopCommand(),
      m_shooter.pivotToRetract()
    );
  }

  private Command toHold() {
    return new SequentialCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, false),
      m_shooter.pivotToHold()
    );
  }

  private Command toFront(Command ScoreLevelPivotComand) {
    return new SequentialCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, false),
      ScoreLevelPivotComand
    );
  }

  private Command getShooterCommand(ShooterScoreLevel shooterScorelevel) {
    System.out.println("get shooter command");
    switch(shooterState) { //TODO UPDATE THIS VARIABLE
      case RETRACT: return toRetract();
      case HOLD: return toHold();
      case BACK: return m_shooter.setDynamicEnabledCommand(true, true);
      case FRONT: switch(shooterScorelevel) {
        case INTAKE: return toFront(m_shooter.intakeSequence());
        case OUTTAKE: return toFront(m_shooter.outtakeSequence());
        case DYNAMIC: return m_shooter.setDynamicEnabledCommand(true, false);
      };
    }
    return new InstantCommand();
  }
      
  public void updateTelemetry() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Shooter State", shooterState.toString());
  }
}
