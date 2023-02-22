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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
    
  private DoubleSolenoid leftRetractionSolenoid;
  private DoubleSolenoid rightRetractionSolenoid;
  private DoubleSolenoid intakeSolenoid;

  private PneumaticHub pneumaticHub;

  private boolean isDeployed;
  private boolean isDown;

  /** Creates a new Intake. */
  public Intake() {
    topMotor = new CANSparkMax(IntakeConstants.kTopMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    bottomMotor = new CANSparkMax(IntakeConstants.kBottomMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
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

  public void intake() {
    topMotor.setVoltage(-0.7*IntakeConstants.kNominalVoltage);
  }

  public void outtake() {
    topMotor.setVoltage(0.7*IntakeConstants.kNominalVoltage);
  }

  public void stop() {
    topMotor.set(0);
  }

  public void deploy() {
    leftRetractionSolenoid.set(Value.kForward);
    rightRetractionSolenoid.set(Value.kForward);
    isDeployed = true;
  }

  public void retract() {
    leftRetractionSolenoid.set(Value.kReverse);
    rightRetractionSolenoid.set(Value.kReverse);
    isDeployed = false;
  }

  public void open() {
    intakeSolenoid.set(Value.kReverse);
    isDown = false;
  }

  public void close() {
    intakeSolenoid.set(Value.kForward);
    isDown = true;
  }

  public boolean getDeployedState() {
    return isDeployed;
  }

  public boolean getDownState() {
    return isDown;
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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
