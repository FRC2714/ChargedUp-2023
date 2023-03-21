// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase {
  private CANSparkMax ClawMotor;
  private RelativeEncoder ClawEncoder;
  private SparkMaxPIDController ClawPID;

  private double openPosition = 0;
  private double closePosition = 32;
  private double maxPosition = 0;

  /** Creates a new Claw. */
  public Claw() {
    ClawMotor = new CANSparkMax(ClawConstants.kClawMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    ClawMotor.setSmartCurrentLimit(ClawConstants.kClawMotorCurrentLimit);

    ClawMotor.enableVoltageCompensation(ClawConstants.kNominalVoltage);

    ClawEncoder = ClawMotor.getEncoder();
    ClawEncoder.setPosition(0);
    ClawEncoder.setPositionConversionFactor(60); //claw gear ratio

    ClawPID = ClawMotor.getPIDController();
    ClawPID.setFeedbackDevice(ClawEncoder);
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

  public double getPosition() {
    return ClawEncoder.getPosition();
  }

  private void setTargetPosition(double position) {
    ClawPID.setReference(position, ControlType.kPosition);
  }

  public Command open() {
    return new InstantCommand(() -> setTargetPosition(openPosition));
  }


  public Command close() {
    return new InstantCommand(() -> setTargetPosition(closePosition));
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
    SmartDashboard.putNumber("Claw Position", getPosition());
  }
}
