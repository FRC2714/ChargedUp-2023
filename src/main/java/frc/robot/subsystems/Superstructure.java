// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterState;

public class Superstructure extends SubsystemBase {
  Arm m_arm;
  Shooter m_shooter;

  ArmStateMachine m_armStateMachine;
  ShooterStateMachine m_shooterStateMachine;
  
  public enum ScoreMode {
    ARM, SHOOTER
  }

  public enum DPadInput {
    UP, DOWN, LEFT, RIGHT
  }

  public enum ArmScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE
  }

  public enum ShooterScoreLevel {
    INTAKE, OUTTAKE, DYNAMIC
  }

  public ScoreMode scoreMode = ScoreMode.ARM;
  public ArmScoreLevel armScoreLevel = ArmScoreLevel.INTAKE;
  public CargoType cargoType = CargoType.CONE;
  public ShooterScoreLevel shooterScoreLevel = ShooterScoreLevel.INTAKE;

  /** Creates a new Superstructure. */
  public Superstructure(Arm m_arm, Shooter m_shooter) {
    this.m_arm = m_arm;
    this.m_shooter = m_shooter;

    m_armStateMachine = new ArmStateMachine(m_arm);
	  m_shooterStateMachine = new ShooterStateMachine(m_shooter);
  }

  public Command armToShooter() {
    return new SequentialCommandGroup(
      //m_armStateMachine.set();
    );
  }

  public Command shooterToArm() {
    return new SequentialCommandGroup(null);
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

  //Subsystem State
  public Command setSubsystemState(DPadInput dPadInput) {
    switch (scoreMode) {
      case ARM: switch (dPadInput) {
        case UP: return m_armStateMachine.setTargetArmStateCommand(ArmState.STOW);
        case DOWN: return m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER);
        case LEFT: return m_armStateMachine.setTargetArmStateCommand(ArmState.BACK);
        case RIGHT: return m_armStateMachine.setTargetArmStateCommand(ArmState.FRONT);
      }
      case SHOOTER: switch (dPadInput) {
        case UP: return m_shooterStateMachine.setShooterStateCommand(ShooterState.HOLD);
        case DOWN: return m_shooterStateMachine.setShooterStateCommand(ShooterState.RETRACT);
        case LEFT: return m_shooterStateMachine.setShooterStateCommand(ShooterState.BACK);
        case RIGHT: return m_shooterStateMachine.setShooterStateCommand(ShooterState.FRONT);
      }
    }
    return new InstantCommand();
  }

  // public void setArmScoreLevel(ArmScoreLevel scoreLevel) {
  //   this.armScoreLevel = scoreLevel;
  // }

  // public ArmScoreLevel getArmScoreLevel() {
  //   return this.armScoreLevel;
  // }

  //Score level
  public InstantCommand setScoreLevelCommand(ArmScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setArmScoreLevel(targetScoreLevel));
  }

  public void setArmScoreLevel(ArmScoreLevel scoreLevel) {
    this.armScoreLevel = scoreLevel;
  }

  public ArmScoreLevel getArmScoreLevel() {
    return this.armScoreLevel;
  }

  //Cargo type
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
      case ARM: return m_armStateMachine.getArmCommand(armScoreLevel, cargoType);
      case SHOOTER: return m_shooterStateMachine.getShooterCommand(shooterScoreLevel);
    }
    return new InstantCommand();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Score Level", armScoreLevel.toString());
    SmartDashboard.putString("Cargo Type", cargoType.toString());
  }
}
