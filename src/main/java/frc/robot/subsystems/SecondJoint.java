// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class SecondJoint extends SubsystemBase {
  private CANSparkMax SecondJointMotor;

  private AbsoluteEncoder SecondJointEncoder;

  private SparkMaxPIDController SecondJointPID;
  //ticks per rev: 8192
  //BaseEncoder.setPositionConversionFactor(2*Math.PI); ???
  private double conversionFactor = 1/8192 * 360;

  private double targetPosition;
  
  /** Creates a new SecondJoint. */
  public SecondJoint() {
    SecondJointMotor = new CANSparkMax(ArmConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    SecondJointEncoder = SecondJointMotor.getAbsoluteEncoder(Type.kDutyCycle);

    SecondJointPID = SecondJointMotor.getPIDController();
    SecondJointPID.setFF(ArmConstants.kSecondJointFF);
    SecondJointPID.setP(ArmConstants.kSecondJointP, 0);
    SecondJointPID.setI(ArmConstants.kSecondJointI, 0);
    SecondJointPID.setD(ArmConstants.kSecondJointD, 0);
    SecondJointPID.setSmartMotionMaxVelocity(ArmConstants.kSecondJointMaxVelocity, 0);
    SecondJointPID.setSmartMotionMaxAccel(ArmConstants.kSecondJointMaxAcceleration, 0);
    SecondJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kSecondJointTolerance, 0);

  }

  public double getAngle() {
    return SecondJointEncoder.getPosition() * conversionFactor;
  }

  public void setTarget(double targetPosition) {
    this.targetPosition = targetPosition;
    SecondJointPID.setReference(-targetPosition, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(targetPosition + getAngle()) < ArmConstants.kSecondJointTolerance;
  }

  public void disable() {
    SecondJointMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Second Joint Encoder", SecondJointEncoder.getPosition());
    SmartDashboard.putNumber("Second Joint Angle", getAngle());
  }
}
