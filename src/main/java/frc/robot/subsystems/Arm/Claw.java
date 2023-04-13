// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

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
  /** Creates a new Claw. */
  public Claw() {
    clawMotor = new CANSparkMax(ClawConstants.kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    clawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);
    clawMotor.setInverted(true);
    clawMotor.enableVoltageCompensation(ClawConstants.kNominalVoltage);
    clawMotor.burnFlash();

    clawSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH, 
      ClawConstants.kClawSolenoidForwardChannel, 
      ClawConstants.kClawSolenoidReverseChannel);
  }

  public void intake() {
    clawMotor.set(ClawConstants.kIntakeMotorSpeed * ClawConstants.kNominalVoltage);
  }

  public void outtake() {
    clawMotor.set(ClawConstants.kShootMotorSpeed * ClawConstants.kNominalVoltage);
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

  public Command intakeAndToggleCommand() {
    return new InstantCommand(() -> {
      clawSolenoid.toggle();
      intake();
    });
  }

  public Command intakeCube() {
    return new InstantCommand(() -> {
      open();
      intake();
    });
  }

  public Command intakeCone() {
    return new InstantCommand(() -> {
      close();
      intake();
    });
  }

  public Command scoreCone() {
    return new InstantCommand(() -> {
      stop();
      open();
    });
  }

  public Command scoreCube() {
    return new InstantCommand(() -> {
      outtake();
      open();
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}