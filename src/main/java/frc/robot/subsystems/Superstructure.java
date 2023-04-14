// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.align.AlignToCone;
import frc.robot.commands.align.AlignToCube;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmScoreLevel;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterScoreLevel;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterState;

public class Superstructure {
  DriveSubsystem m_robotDrive;

  Arm m_arm;
  Claw m_claw;

  Shooter m_shooter;

  Limelight m_backLimelight;

  LED m_armLED;

  ArmStateMachine m_armStateMachine;
  ShooterStateMachine m_shooterStateMachine;
  
  public enum ScoreMode {
    ARM, SHOOTER
  }

  private enum ScoreModeTransition {
    TO_ARM, TO_SHOOTER, DO_NOTHING
  }

  public enum DPAD {
    UP, DOWN, LEFT, RIGHT
  }

  public enum BUTTON {
    Y, A, X, B
  }

  public enum CargoType {
    CONE, CUBE
  }

  public ScoreMode scoreMode = ScoreMode.ARM; //default to arm
  public CargoType cargoType = CargoType.CONE; //default to cone

  /** Creates a new Superstructure. */
  public Superstructure(DriveSubsystem m_robotDrive, Arm m_arm, Claw m_claw, Shooter m_shooter, Limelight m_backLimelight, LED m_armLED) {
    this.m_robotDrive = m_robotDrive;

    this.m_arm = m_arm;
    this.m_claw = m_claw;

    this.m_shooter = m_shooter;

    this.m_backLimelight = m_backLimelight;

    this.m_armLED = m_armLED;

    m_armStateMachine = new ArmStateMachine(m_arm);
	  m_shooterStateMachine = new ShooterStateMachine(m_shooter);
  }

  private Command armToShooter() {
    return new SequentialCommandGroup(
      setSubsystemState(DPAD.UP),
      new WaitUntilCommand(() -> m_arm.isShoulderAtGoal()),
      new InstantCommand(() -> this.scoreMode = ScoreMode.SHOOTER),
      new InstantCommand(() -> m_armLED.setRed()),
      new InstantCommand(() -> m_shooter.setShooterEnabled(true))
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  private Command shooterToArm() {
    return new SequentialCommandGroup(
      setSubsystemState(DPAD.DOWN),
      new InstantCommand(() -> m_shooter.setShooterEnabled(false)),
      new InstantCommand(() -> this.scoreMode = ScoreMode.ARM),
      new InstantCommand(() -> m_armLED.set(getCargoType() == CargoType.CONE ? LEDConstants.kYellow : LEDConstants.kPurple))
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  //Score mode
  public Command setScoreModeCommand(ScoreMode targetScoreMode) {
    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreModeTransition.TO_ARM, shooterToArm()),
        Map.entry(ScoreModeTransition.TO_SHOOTER, armToShooter()),
        Map.entry(ScoreModeTransition.DO_NOTHING, new InstantCommand())),
      () -> {
        if(getScoreMode() == targetScoreMode) { return ScoreModeTransition.DO_NOTHING;} 
        return targetScoreMode == ScoreMode.ARM ? ScoreModeTransition.TO_ARM : ScoreModeTransition.TO_SHOOTER;
      }
    );
  }

  public ScoreMode getScoreMode() {
    return this.scoreMode;
  }

  //Subsystem State
  public Command setSubsystemState(DPAD dPadInput) {
    final SelectCommand coneArmSelectCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(DPAD.UP, m_armStateMachine.setTargetArmStateCommand(ArmState.STOW, CargoType.CONE)),
        Map.entry(DPAD.DOWN, m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER, CargoType.CONE)),
        Map.entry(DPAD.LEFT, m_armStateMachine.setTargetArmStateCommand(ArmState.FRONT, CargoType.CONE)),
        Map.entry(DPAD.RIGHT, m_armStateMachine.setTargetArmStateCommand(ArmState.BACK, CargoType.CONE))), 
      () -> dPadInput);

    final SelectCommand cubeArmSelectCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(DPAD.UP, m_armStateMachine.setTargetArmStateCommand(ArmState.STOW, CargoType.CUBE)),
        Map.entry(DPAD.DOWN, m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER, CargoType.CUBE)),
        Map.entry(DPAD.LEFT, m_armStateMachine.setTargetArmStateCommand(ArmState.FRONT, CargoType.CUBE)),
        Map.entry(DPAD.RIGHT, m_armStateMachine.setTargetArmStateCommand(ArmState.BACK, CargoType.CUBE))), 
      () -> dPadInput);

    final SelectCommand shooterSelectCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(DPAD.UP, m_shooterStateMachine.setShooterStateCommand(ShooterState.HOLD)),
        Map.entry(DPAD.DOWN, m_shooterStateMachine.setShooterStateCommand(ShooterState.RETRACT)),
        Map.entry(DPAD.LEFT, m_shooterStateMachine.setShooterStateCommand(ShooterState.MANUAL)),
        Map.entry(DPAD.RIGHT, m_shooterStateMachine.setShooterStateCommand(ShooterState.DYNAMIC))),
      () -> dPadInput); 

    final SelectCommand armSelectCommand = new SelectCommand(
        Map.ofEntries(
          Map.entry(CargoType.CONE, coneArmSelectCommand),
          Map.entry(CargoType.CUBE, cubeArmSelectCommand)
        ), 
        () -> getCargoType()
    );
    
    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreMode.ARM, armSelectCommand),
        Map.entry(ScoreMode.SHOOTER, shooterSelectCommand)), 
      () -> getScoreMode());
  }

  //Score Level
  public Command setScoreLevelCommand(BUTTON buttonInput) {
    final SelectCommand armSelectCommand = new SelectCommand(
        Map.ofEntries(
          Map.entry(BUTTON.Y, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.HIGH)),
          Map.entry(BUTTON.B, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.MIDDLE)),
          Map.entry(BUTTON.A, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.INTAKE)),
          Map.entry(BUTTON.X, m_armStateMachine.setArmScoreLevelCommand(ArmScoreLevel.LOW))
          ), 
        () -> buttonInput);

    final SelectCommand shooterSelectCommand = new SelectCommand(
      Map.ofEntries(
        Map.entry(BUTTON.Y, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.HIGH)),
        Map.entry(BUTTON.B, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.MIDDLE)),
        Map.entry(BUTTON.A, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.LOW)),
        Map.entry(BUTTON.X, m_shooterStateMachine.setShooterScoreLevelCommand(ShooterScoreLevel.INTAKE))
        ),
      () -> buttonInput);

    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreMode.ARM, armSelectCommand),
        Map.entry(ScoreMode.SHOOTER, shooterSelectCommand)), 
      () -> getScoreMode());
  }

  //Cargo type
  public Command setCargoTypeCommand(CargoType targetCargoType) {
    return new InstantCommand(() -> {
      m_armLED.set(targetCargoType == CargoType.CONE ? LEDConstants.kYellow : LEDConstants.kPurple);
      this.cargoType = targetCargoType;
    });
  }

  public CargoType getCargoType() {
    return this.cargoType;
  }

  //Score Command
  public Command ScoreCommand() {
    final SelectCommand armScore = new SelectCommand(
      Map.ofEntries(
        Map.entry(CargoType.CONE, m_claw.scoreCone()),
        Map.entry(CargoType.CUBE, m_claw.scoreCube())
      ), 
      () -> getCargoType()
    );

    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreMode.ARM, armScore),
        Map.entry(ScoreMode.SHOOTER, 
          new StartEndCommand(() -> {
            m_shooter.setKicker(-0.4);
            m_armLED.setRed();
          },
          () -> m_shooter.setKicker(0), 
          m_shooter)) //TODO SHOOTER SCORE
      ), 
      () -> getScoreMode()
    );
  }


  //INTAKE BINDINGS
  public Command shooterIntakeSequence() {
    return new ConditionalCommand(
      new InstantCommand(), 
      new SequentialCommandGroup(
        setScoreLevelCommand(BUTTON.X),
        setSubsystemState(DPAD.LEFT)), 
      () -> getScoreMode() != ScoreMode.SHOOTER
    );
  }

  public Command shooterOuttakeSequence() {
    return new ConditionalCommand(
      new InstantCommand(), 
      new SequentialCommandGroup(
        setScoreLevelCommand(BUTTON.A),
        setSubsystemState(DPAD.LEFT)), 
      () -> getScoreMode() != ScoreMode.SHOOTER
    );
  }

  public Command getAlign() {
    final SelectCommand armAlign = new SelectCommand(
      Map.ofEntries(
        Map.entry(CargoType.CONE, new AlignToCone(m_robotDrive, m_backLimelight)), // align to cone
        Map.entry(CargoType.CUBE, new AlignToCube(m_robotDrive, m_backLimelight)) //align to cube
      ), 
      () -> getCargoType()
    );

    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreMode.ARM, armAlign),
        Map.entry(ScoreMode.SHOOTER, new TurnToAngle(m_robotDrive, 180))
      ), 
      () -> getScoreMode()
    );
  }

  public Command scorePreloadedCone(double waitTime) { // for auto use only 3.5 good
    return new SequentialCommandGroup(
      m_claw.intakeCone(),
			setScoreModeCommand(ScoreMode.ARM),
			setCargoTypeCommand(CargoType.CONE),
			setScoreLevelCommand(BUTTON.Y),
			setSubsystemState(DPAD.RIGHT),
			new WaitCommand(waitTime).raceWith(new AlignToCone(m_robotDrive, m_backLimelight)),
			m_claw.scoreCone()
    );
  }

  public void updateTelemetry() {
    //auto intake
    if (m_shooter.isCubeDetected() &&
      scoreMode == ScoreMode.SHOOTER &&
      m_shooterStateMachine.getShooterScoreLevel() == ShooterScoreLevel.INTAKE &&
      m_shooterStateMachine.getShooterState() == ShooterState.MANUAL) {
      setSubsystemState(DPAD.UP).schedule();
      m_armLED.setGreen();
    }

    SmartDashboard.putString("Score Mode", scoreMode.toString());
    SmartDashboard.putString("Cargo Type", cargoType.toString());

    m_arm.updateTelemetry();
    m_armStateMachine.updateTelemetry();
    m_shooterStateMachine.updateTelemetry();
  }
}
