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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.align.AlignToCone;
import frc.robot.commands.align.AlignToCube;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmScoreLevel;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Arm.Claw.ClawState;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.Shooter.KickerState;
import frc.robot.subsystems.Shooter.ShooterStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterScoreLevel;
import frc.robot.subsystems.Shooter.ShooterStateMachine.ShooterState;

public class Superstructure {
  DriveSubsystem m_drivetrain;

  Arm m_arm;
  Claw m_claw;

  Shooter m_shooter;

  Limelight m_limelight;

  LED m_led;

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
  public Superstructure(DriveSubsystem m_drivetrain, Arm m_arm, Claw m_claw, Shooter m_shooter, Limelight m_limelight, LED m_led) {
    this.m_drivetrain = m_drivetrain;
    this.m_arm = m_arm;
    this.m_claw = m_claw;
    this.m_shooter = m_shooter;
    this.m_limelight = m_limelight;
    this.m_led = m_led;

    m_armStateMachine = new ArmStateMachine(m_arm);
	  m_shooterStateMachine = new ShooterStateMachine(m_shooter);
  }

  private Command armToShooter() {
    return new SequentialCommandGroup(
      setSubsystemState(DPAD.UP),
      new WaitUntilCommand(() -> m_arm.isShoulderAtGoal()),
      new InstantCommand(() -> {
        this.scoreMode = ScoreMode.SHOOTER;
        m_shooter.setShooterEnabled(true);
        m_led.setRed();
      })
    ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  private Command shooterToArm() {
    return new SequentialCommandGroup(
      setSubsystemState(DPAD.DOWN),
      new WaitUntilCommand(() -> m_shooter.atPivotSetpoint()),
      new InstantCommand(() -> {
        m_claw.setClawStop();
        m_shooter.setShooterEnabled(false);
        this.scoreMode = ScoreMode.ARM;
        m_led.set(getCargoType() == CargoType.CONE ? LEDConstants.kYellow : LEDConstants.kPurple);
      })
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
      if (scoreMode == ScoreMode.ARM) m_led.set(targetCargoType == CargoType.CONE ? LEDConstants.kYellow : LEDConstants.kPurple);
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
        Map.entry(ScoreMode.SHOOTER, m_shooter.setKickerOuttakeCommand(ShooterConstants.kKickSpeed)) //TODO SHOOTER SCORE
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
        Map.entry(CargoType.CONE, new AlignToCone(m_drivetrain, m_limelight)), // align to cone
        Map.entry(CargoType.CUBE, new AlignToCube(m_drivetrain, m_limelight)) //align to cube
      ), 
      () -> getCargoType()
    );

    return new SelectCommand(
      Map.ofEntries(
        Map.entry(ScoreMode.ARM, armAlign),
        Map.entry(ScoreMode.SHOOTER, new TurnToAngle(m_drivetrain, 180))
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
			new WaitCommand(waitTime).raceWith(new AlignToCone(m_drivetrain, m_limelight)),
			m_claw.scoreCone()
    );
  }

  public Command shootCube() {
    return new SequentialCommandGroup(
      m_shooter.setKickerOuttakeCommand(ShooterConstants.kKickSpeed),
      new WaitCommand(0.2),
      m_shooter.stopCommand()
    );
  }

  public void AutomationLogic() {
    if (scoreMode == ScoreMode.ARM) {
      if (m_claw.getClawState() == ClawState.OUTTAKING) {
        m_led.set(getCargoType() == CargoType.CONE ? LEDConstants.kYellow : LEDConstants.kPurple);
      }
      if (m_claw.isCurrentSpikeDetected()) {
        m_led.setGreen();
      }
    } else if (scoreMode == ScoreMode.SHOOTER) {
      if (m_shooter.isCubeDetected() &&
      m_shooterStateMachine.getShooterScoreLevel() == ShooterScoreLevel.INTAKE &&
      m_shooterStateMachine.getShooterState() == ShooterState.MANUAL) { //auto intake
        m_led.setGreen();
        setSubsystemState(DPAD.UP).schedule();
        m_shooter.setKickerIntake(ShooterConstants.kKickerHoldMotorSpeed);
      }
      if (m_shooter.getKickerState() == KickerState.OUTTAKING) {
        m_led.setRed();
      }
    }
  }

  public void periodic() {
    AutomationLogic();

    SmartDashboard.putString("Score Mode", scoreMode.toString());
    SmartDashboard.putString("Cargo Type", cargoType.toString());

    m_arm.updateTelemetry();
    m_armStateMachine.updateTelemetry();
    m_shooterStateMachine.updateTelemetry();
  }
}
