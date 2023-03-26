// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine;

public class Superstructure extends SubsystemBase {
  ArmStateMachine m_armStateMachine;
  ShooterStateMachine m_shooterStateMachine;

  public enum ScoreMode{
    ARM, SHOOTER
  }

  public enum ScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE
  }

  public ScoreMode scoreMode = ScoreMode.ARM;
  public ScoreLevel scoreLevel = ScoreLevel.INTAKE;
  public CargoType cargoType = CargoType.CONE;

  /** Creates a new Superstructure. */
  public Superstructure(ArmStateMachine m_armStateMachine, ShooterStateMachine m_shooterStateMachine) {
    this.m_armStateMachine = m_armStateMachine;
    this.m_shooterStateMachine = m_shooterStateMachine;
  }

  //Score mode
  public InstantCommand setScoreModeCommand(ScoreMode scoreMode) {
    return new InstantCommand(() -> setScoreMode(scoreMode));
  }

  public void setScoreMode(ScoreMode scoreMode) {
    this.scoreMode = scoreMode;
  }

  public ScoreLevel getScoreMode() {
    return this.scoreLevel;
  }

  //Score level
  public InstantCommand setScoreLevelCommand(ScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setScoreLevel(targetScoreLevel));
  }

  public void setScoreLevel(ScoreLevel scoreLevel) {
    this.scoreLevel = scoreLevel;
  }

  public ScoreLevel getScoreLevel() {
    return this.scoreLevel;
  }

  //Cargot type
  public InstantCommand setCargoTypeCommand(CargoType cargoType) {
    return new InstantCommand(() -> setCargoType(cargoType));
  }

  public void setCargoType(CargoType cargoType) {
    this.cargoType = cargoType;
  }

  public CargoType getCargoType() {
    return this.cargoType;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
