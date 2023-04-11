// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Limelight;

public class ShooterStateMachine {
  Shooter m_shooter;
  Limelight m_frontLimelight;

  public enum ShooterState {
    RETRACT, HOLD, MANUAL, DYNAMIC
  }

  public enum ShooterScoreLevel {
    INTAKE, LOW, MIDDLE, HIGH
  }

  public ShooterState shooterState = ShooterState.RETRACT;
  public ShooterScoreLevel scoreLevel = ShooterScoreLevel.INTAKE;

  /** Creates a new ShooterStateMachine. */
  public ShooterStateMachine(Shooter m_shooter, Limelight m_frontLimelight) {
    this.m_shooter = m_shooter;
    this.m_frontLimelight = m_frontLimelight;
  }

  public Command setShooterStateCommand(ShooterState shooterState) {
    return new InstantCommand(() -> {
      System.out.println("setting target armstate");
      if (this.shooterState != shooterState) {
        this.shooterState = shooterState;
      }
      getShooterCommand(scoreLevel).withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
    });
  }

  //Score level
  public Command setShooterScoreLevelCommand(ShooterScoreLevel scoreLevel) {
    return new InstantCommand(() -> this.scoreLevel = scoreLevel);
  }

  public ShooterScoreLevel getShooterScoreLevel() {
    return this.scoreLevel;
  }

  public ShooterState getShooterState() {
    return this.shooterState;
  }

  private Command toRetract(ShooterScoreLevel shooterScorelevel) {
    return new SequentialCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, shooterScorelevel),
      m_shooter.stopCommand(),
      m_shooter.setPreset(ShooterConstants.kRetractPreset));
  }

  private Command toHold(ShooterScoreLevel shooterScorelevel) {
    return new SequentialCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, shooterScorelevel),
      m_shooter.setPreset(ShooterConstants.kHoldPreset));
  }

  // private Command toDynamic(ShooterScoreLevel shooterScorelevel) {
  //   return new SequentialCommandGroup(
  //     new PrintCommand("to dynamic"),
  //     new InstantCommand(() -> m_frontLimelight.setLED(true)),
  //     new SelectCommand(
  //       Map.ofEntries(
  //         Map.entry(ShooterScoreLevel.HIGH, new InstantCommand(() -> m_frontLimelight.setHighCubePipeline())),
  //         Map.entry(ShooterScoreLevel.MIDDLE, new InstantCommand(() -> m_frontLimelight.setMiddleCubePipeline())),
  //         Map.entry(ShooterScoreLevel.LOW, new InstantCommand(() -> m_frontLimelight.setLowCubePipeline())),
  //         Map.entry(ShooterScoreLevel.INTAKE, new InstantCommand())
  //       ), () -> shooterScorelevel),
  //     m_shooter.setDynamicEnabledCommand(true, shooterScorelevel));
  // }

  private Command toDynamic(ShooterScoreLevel shooterScorelevel) {
    return new SequentialCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, shooterScorelevel),
      //new InstantCommand(() -> m_frontLimelight.setLED(true)),
      new SelectCommand(
        Map.ofEntries(
          Map.entry(ShooterScoreLevel.HIGH, new InstantCommand(() -> m_frontLimelight.setHighCubePipeline())),
          Map.entry(ShooterScoreLevel.MIDDLE, new InstantCommand(() -> m_frontLimelight.setMiddleCubePipeline())),
          Map.entry(ShooterScoreLevel.LOW, new InstantCommand(() -> m_frontLimelight.setLowCubePipeline())),
          Map.entry(ShooterScoreLevel.INTAKE, new InstantCommand())
        ), () -> shooterScorelevel),
      new SelectCommand(
        Map.ofEntries(
          Map.entry(ShooterScoreLevel.HIGH, m_shooter.setPreset(ShooterConstants.kCloseHighCubePreset)),
          Map.entry(ShooterScoreLevel.MIDDLE, m_shooter.setPreset(ShooterConstants.kCloseMiddleCubePreset)),
          Map.entry(ShooterScoreLevel.LOW, m_shooter.setPreset(ShooterConstants.kLaunchCubePreset)),
          Map.entry(ShooterScoreLevel.INTAKE, m_shooter.setPreset(ShooterConstants.kLaunchCubePreset))
        ), () -> shooterScorelevel));
      //m_shooter.setDynamicEnabledCommand(true, shooterScorelevel));
  }

  private Command toManual(ShooterScoreLevel shooterScorelevel) {
    return new SequentialCommandGroup(
      m_shooter.setDynamicEnabledCommand(false, shooterScorelevel),
      new SelectCommand(
        Map.ofEntries(
          Map.entry(ShooterScoreLevel.HIGH, m_shooter.setPreset(ShooterConstants.kLaunchCubePreset)),
          Map.entry(ShooterScoreLevel.MIDDLE, m_shooter.setPreset(ShooterConstants.kLaunchCubePreset)),
          Map.entry(ShooterScoreLevel.LOW, m_shooter.outtakeSequence()),
          Map.entry(ShooterScoreLevel.INTAKE, m_shooter.intakeSequence())
        ), () -> shooterScorelevel));
  }

  private Command getShooterCommand(ShooterScoreLevel shooterScorelevel) {
    System.out.println("get shooter command");
    switch(shooterState) {
      case RETRACT: return toRetract(shooterScorelevel);
      case HOLD: return toHold(shooterScorelevel);
      case DYNAMIC: return toDynamic(shooterScorelevel);
      case MANUAL: return toManual(shooterScorelevel);
    };
    return new InstantCommand();
  }
      
  public void updateTelemetry() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Shooter State", shooterState.toString());
  }
}
