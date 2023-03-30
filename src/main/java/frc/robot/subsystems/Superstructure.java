// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmScoreLevel;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterScoreLevel;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterState;
import edu.wpi.first.wpilibj2.command.SelectCommand;

public class Superstructure {
  Arm m_arm;
  Shooter m_shooter;

  ArmStateMachine m_armStateMachine;
  ShooterStateMachine m_shooterStateMachine;
  
  public enum ScoreMode {
    ARM, SHOOTER
  }

  public enum ScoreModeAction {
    TO_ARM, TO_SHOOTER, DO_NOTHING
  }

  public enum ControllerInput {
    UP, DOWN, LEFT, RIGHT
  }

  public enum CargoType {
    CONE, CUBE
  }

  public ScoreMode scoreMode = ScoreMode.ARM;
  
  public CargoType cargoType = CargoType.CONE;
 

  /** Creates a new Superstructure. */
  public Superstructure(Arm m_arm, Shooter m_shooter) {
    this.m_arm = m_arm;
    this.m_shooter = m_shooter;

    m_armStateMachine = new ArmStateMachine(m_arm);
	  m_shooterStateMachine = new ShooterStateMachine(m_shooter);
  }

  public Command armToShooter() {
    return new SequentialCommandGroup(
      setSubsystemState(ControllerInput.UP),
      new WaitUntilCommand(2)
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  public Command shooterToArm() {
    return new SequentialCommandGroup(
      setSubsystemState(ControllerInput.DOWN),
      new WaitUntilCommand(() -> m_shooter.atPivotSetpoint()),
      new InstantCommand(() -> m_shooter.setShooterEnabled(false))
    );
  }

  //Score mode
  public Command setScoreModeCommand(ScoreMode targetScoreMode) {
    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreModeAction.TO_ARM, shooterToArm()),
        Map.entry(ScoreModeAction.TO_SHOOTER, armToShooter()),
        Map.entry(ScoreModeAction.DO_NOTHING, new InstantCommand())),
      () -> {
        if(this.scoreMode == targetScoreMode) {
          return ScoreModeAction.DO_NOTHING;
        } 
        return targetScoreMode == ScoreMode.ARM ? ScoreModeAction.TO_ARM : ScoreModeAction.TO_SHOOTER;
      }
    ).andThen(new InstantCommand(() -> this.scoreMode = targetScoreMode));
  }

  public ScoreMode getScoreMode() {
    return this.scoreMode;
  }

  //Subsystem State
  public Command setSubsystemState(ControllerInput dPadInput) {
    final SelectCommand armSelectCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(ControllerInput.UP, m_armStateMachine.setTargetArmStateCommand(ArmState.STOW, cargoType)),
        Map.entry(ControllerInput.DOWN, m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER, cargoType)),
        Map.entry(ControllerInput.LEFT, m_armStateMachine.setTargetArmStateCommand(ArmState.FRONT, cargoType)),
        Map.entry(ControllerInput.RIGHT, m_armStateMachine.setTargetArmStateCommand(ArmState.BACK, cargoType))), 
      () -> dPadInput);

    final SelectCommand shooterSelectCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(ControllerInput.UP, m_shooterStateMachine.setShooterStateCommand(ShooterState.HOLD)),
        Map.entry(ControllerInput.DOWN, m_shooterStateMachine.setShooterStateCommand(ShooterState.RETRACT)),
        Map.entry(ControllerInput.LEFT, m_shooterStateMachine.setShooterStateCommand(ShooterState.FRONT)),
        Map.entry(ControllerInput.RIGHT, m_shooterStateMachine.setShooterStateCommand(ShooterState.BACK))),
      () -> dPadInput); 
    
    System.out.println("Creating SelectCommand for arm vs shooter");
    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreMode.ARM, armSelectCommand),
        Map.entry(ScoreMode.SHOOTER, shooterSelectCommand)), 
      () -> getScoreMode());
  }

  //Score Level
  public Command setScoreLevelCommand(ControllerInput buttonInput) {
    final SelectCommand armSelectCommand = new SelectCommand(
        Map.ofEntries(
          Map.entry(ControllerInput.UP, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.THREE)),
          Map.entry(ControllerInput.DOWN, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.TWO)),
          Map.entry(ControllerInput.LEFT, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.INTAKE)),
          Map.entry(ControllerInput.RIGHT, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.ONE))
          ), 
        () -> buttonInput);

    final SelectCommand shooterSelectCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(ControllerInput.UP, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.DYNAMIC)),
        Map.entry(ControllerInput.DOWN, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.INTAKE)),
        Map.entry(ControllerInput.LEFT, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.OUTTAKE)),
        Map.entry(ControllerInput.RIGHT, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.DYNAMIC))
        ),
      () -> buttonInput);

    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreMode.ARM, armSelectCommand),
        Map.entry(ScoreMode.SHOOTER, shooterSelectCommand)), 
      () -> getScoreMode());
  }

  //Cargo type
  public InstantCommand setCargoTypeCommand(CargoType cargoType) {
    return new InstantCommand(() -> this.cargoType = cargoType);
  }

  public CargoType getCargoType() {
    return this.cargoType;
  }

  public void updateTelemetry() {
    SmartDashboard.putString("Score Mode", scoreMode.toString());
    SmartDashboard.putString("Cargo Type", cargoType.toString());

    m_arm.updateTelemetry();
    m_armStateMachine.updateTelemetry();
    m_shooterStateMachine.updateTelemetry();
  }
}
