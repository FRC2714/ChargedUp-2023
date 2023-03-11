// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private CANSparkMax ClawMotor;

  private SparkMaxPIDController ClawPID;

  private double openPosition = 0;
  private double closePosition = 0;
  private double maxPosition = 0;

  /** Creates a new Claw. */
  public Claw() {
    ClawMotor = new CANSparkMax(ClawConstants.kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    ClawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);

    ClawMotor.enableVoltageCompensation(ClawConstants.kNominalVoltage);

    ClawPID = ClawMotor.getPIDController();

    ClawPID.setFF(0, 0);
    ClawPID.setP(0, 0);
    ClawPID.setI(0, 0);
    ClawPID.setD(0, 0);

    ClawPID.setPositionPIDWrappingEnabled(false);
    ClawPID.setPositionPIDWrappingMinInput(0);
    ClawPID.setPositionPIDWrappingMaxInput(maxPosition);
  }

  public void stop() {
    ClawMotor.set(0);
  }

  public void open() {
    ClawPID.setReference(openPosition, ControlType.kPosition);
  }

  public void close() {
    ClawPID.setReference(closePosition, ControlType.kPosition);
  }

  public void intakeClose() {
    close();
  }
  public void intakeOpen() {
    open();
  }

  public Command intakeConeCommand() {
    return (
      new InstantCommand(() -> close()));
  }

  public Command intakeCubeCommand() {
    return (
      new InstantCommand(() -> open()));
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
      new InstantCommand(() -> open()); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
