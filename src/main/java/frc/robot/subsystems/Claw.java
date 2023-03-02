// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;

  private DoubleSolenoid clawSolenoid;
  
  private boolean isOpen;

  /** Creates a new Claw. */
  public Claw() {
    clawMotor = new CANSparkMax(ClawConstants.kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);

    clawMotor.enableVoltageCompensation(ClawConstants.kNominalVoltage);

    clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.kClawSolenoidForwardChannel, ClawConstants.kClawSolenoidReverseChannel);
  }

  public void intake() {
    clawMotor.setVoltage(ClawConstants.kIntakeMotorSpeed*ClawConstants.kNominalVoltage);
  }

  public void outtake() {
    clawMotor.setVoltage(ClawConstants.kOuttakeMotorSpeed*ClawConstants.kNominalVoltage);
  }

  public void shoot() {
    clawMotor.setVoltage(6*ClawConstants.kOuttakeMotorSpeed*ClawConstants.kNominalVoltage);
  }

  public void stop() {
    clawMotor.set(0);
  }

  public void close() {
    clawSolenoid.set(Value.kReverse);
    isOpen = false;
  }
  
  public void open() {
    clawSolenoid.set(Value.kForward);
    isOpen = true;
  }

  public boolean getState() {
    return isOpen;
  }

  public void intakeClose() {
    close();
    intake();
  }
  public void intakeOpen() {
    open();
    intake();
  }

  public Command intakeConeCommand() {
    return (
      new InstantCommand(() -> close())).andThen(
      new InstantCommand(() -> intake()));
  }

  public Command intakeCubeCommand() {
    return (
      new InstantCommand(() -> open())).andThen(
      new InstantCommand(() -> intake()));
  }

  public Command stopOpen() {
    return (
      new InstantCommand(() -> stop())).andThen(
      new InstantCommand(() -> open()));
  }

  public Command scoreCone() {
    return (
      new InstantCommand(() -> stop())).andThen(
      new InstantCommand(() -> open()));
  }

  public Command shootCube() {
    return 
      new InstantCommand(() -> shoot()).andThen(
      new InstantCommand(() -> open())); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
