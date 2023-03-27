// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

  public enum ArmScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE
  }

  public enum ShooterScoreLevel {
    INTAKE, OUTTAKE, SHOOT
  }

  public ScoreMode scoreMode = ScoreMode.ARM;
  public ArmScoreLevel scoreLevel = ArmScoreLevel.INTAKE;
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
    if(this.scoreMode != scoreMode) {
      switch(scoreMode) {
        case ARM: //transition
        case SHOOTER: // transition
      }
    }

    this.scoreMode = scoreMode;
  }

  public ScoreMode getScoreMode() {
    return this.scoreMode;
  }

  //Score level
  public InstantCommand setScoreLevelCommand(ArmScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setScoreLevel(targetScoreLevel));
  }

  public void setScoreLevel(ArmScoreLevel scoreLevel) {
    this.scoreLevel = scoreLevel;
  }

  public ArmScoreLevel getScoreLevel() {
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

  public Command getCommand() {
    switch (scoreMode) {
      case ARM: return m_armStateMachine.getArmCommand(scoreLevel, cargoType);
      case SHOOTER: return new InstantCommand();
    }
    return new InstantCommand();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Score Level", scoreLevel.toString());
    SmartDashboard.putString("Cargo Type", cargoType.toString());
  }
}
