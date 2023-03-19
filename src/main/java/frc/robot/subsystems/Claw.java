// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;
import frc.robot.Constants.PneumaticsConstants;

public class Claw extends SubsystemBase {
  private CANSparkMax clawMotor;

  private DoubleSolenoid clawSolenoid;

  private PneumaticHub pneumaticHub;

  /** Creates a new Claw. */
  public Claw() {
    pneumaticHub = new PneumaticHub(PneumaticsConstants.kPneumaticHubCanId);
    pneumaticHub.enableCompressorAnalog(PneumaticsConstants.kCompressorMinPressure, PneumaticsConstants.kCompressorMaxPressure);

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
    clawMotor.setVoltage(ClawConstants.kShootMotorSpeed*ClawConstants.kNominalVoltage);
  }

  public void stop() {
    clawMotor.set(0);
  }

  public void open() {
    clawSolenoid.set(Value.kForward);
  }

  public void close() {
    clawSolenoid.set(Value.kReverse);
  }

  public void toggle() {
    clawSolenoid.toggle();
  }
  
  public boolean isOpen() {
    return clawSolenoid.get() == Value.kForward;
  }

  public void intakeAndToggle() {
    toggle();
    intake();
  }

  public void intakeClose() {
    close();
    intake();
  }

  public void intakeOpen() {
    open();
    intake();
  }

  public Command intakeOpenCommand() {
    return new InstantCommand(() -> intakeOpen());
  }

  public Command intakeCloseCommand() {
    return new InstantCommand(() -> intakeClose());
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

  public Command scoreCube() {
    return 
      new InstantCommand(() -> shoot()).andThen(
      new InstantCommand(() -> open())); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
