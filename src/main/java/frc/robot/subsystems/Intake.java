// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
    
  private DoubleSolenoid leftRetractionSolenoid;
  private DoubleSolenoid rightRetractionSolenoid;
  private DoubleSolenoid intakeSolenoid;

  private PneumaticHub pneumaticHub;

  private IntakeState intakeState = IntakeState.STOPPED;

  private Timer intakeRunningTimer = new Timer();

  /** Creates a new Intake. */
  public Intake() {
    topMotor = new CANSparkMax(IntakeConstants.kTopMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(IntakeConstants.kBottomMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    topMotor.setInverted(true);
    bottomMotor.follow(topMotor, false);

    topMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.setIdleMode(IdleMode.kBrake);
    topMotor.setSmartCurrentLimit(IntakeConstants.kTopMotorCurrentLimit);
    bottomMotor.setSmartCurrentLimit(IntakeConstants.kBottomMotorCurrentLimit);

    topMotor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);
    bottomMotor.enableVoltageCompensation(IntakeConstants.kNominalVoltage);

    pneumaticHub = new PneumaticHub(IntakeConstants.kPneumaticHubCanId);
    pneumaticHub.enableCompressorAnalog(IntakeConstants.kCompressorMinPressure, IntakeConstants.kCompressorMaxPressure);

    leftRetractionSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.kLeftRetractionSolenoidForwardChannel, IntakeConstants.kLeftRetractionSolenoidReverseChannel);
    rightRetractionSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.kRightRetractionSolenoidForwardChannel, IntakeConstants.kRightRetractionSolenoidReverseChannel);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.kIntakeSolenoidForwardChannel, IntakeConstants.kIntakeSolenoidReverseChannel);
  }

  public enum IntakeState {
    INTAKING, OUTTAKING, STOPPED
  }

  public void intake() {
    topMotor.setVoltage(IntakeConstants.kIntakeMotorSpeed*IntakeConstants.kNominalVoltage);
    if (intakeState != IntakeState.INTAKING) {
      intakeRunningTimer.reset();
      intakeRunningTimer.start();
      intakeState = IntakeState.INTAKING;
    }
  }

  public void outtake() {
    topMotor.setVoltage(IntakeConstants.kOuttakeMotorSpeed*IntakeConstants.kNominalVoltage);
    if (intakeState != IntakeState.OUTTAKING) {
      intakeRunningTimer.reset();
      intakeRunningTimer.start();
      intakeState = IntakeState.OUTTAKING;
    }
  }

  public void stop() {
    intakeState = IntakeState.STOPPED;
    topMotor.set(0);
  }

  public void deploy() {
    leftRetractionSolenoid.set(Value.kForward);
    rightRetractionSolenoid.set(Value.kForward);
  }

  public void retract() {
    leftRetractionSolenoid.set(Value.kReverse);
    rightRetractionSolenoid.set(Value.kReverse);
  }

  public void open() {
    intakeSolenoid.set(Value.kReverse);
  }

  public void close() {
    intakeSolenoid.set(Value.kForward);
  }

  public boolean getDeployed() {
    return leftRetractionSolenoid.get() == Value.kForward;
  }

  public boolean getClosed() {
    return intakeSolenoid.get() == Value.kForward;
  }

  public boolean isConeDetected() {
    return (intakeRunningTimer.get() > 0.1) && //excludes current spike when motor first starts
      (topMotor.getOutputCurrent() > 18) && //cone intake current threshold
      (intakeState == IntakeState.INTAKING) && 
      getClosed();
  }

  public Command intakeCone() {
    return (
      new InstantCommand(() -> deploy()))
        .andThen(new InstantCommand(() -> close())
        .andThen(new InstantCommand(() -> intake())));
  }

  public Command intakeCube() {
    return (
      new InstantCommand(() -> deploy()))
        .andThen(new InstantCommand(() -> open())
        .andThen(new InstantCommand(() -> intake())));
  }

  public Command retractAndStop() {
    return (
      new InstantCommand(() -> retract()))
        .andThen(new InstantCommand(() -> stop()));
  }

  public Command AutoConeIntake() {
    return new SequentialCommandGroup(
      new WaitCommand(0.25),
      new InstantCommand(() -> retract()),
      new WaitCommand(1),
      new InstantCommand(() -> stop()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isConeDetected()) {AutoConeIntake().schedule();}

    // SmartDashboard.putNumber("top motor output current", topMotor.getOutputCurrent());
    // SmartDashboard.putNumber("bootom motor output current", bottomMotor.getOutputCurrent());
    // SmartDashboard.putBoolean("cone detected?", isConeDetected());
    // SmartDashboard.putNumber("intake running timer", intakeRunningTimer.get());
    // SmartDashboard.putBoolean("is closed?", getClosed());
    // SmartDashboard.putString("intake state", intakeState.toString());
  }
}
