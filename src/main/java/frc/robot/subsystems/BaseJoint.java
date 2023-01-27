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

public class BaseJoint extends SubsystemBase {
  private CANSparkMax RightBaseMotor;
  private CANSparkMax LeftBaseMotor;

  private AbsoluteEncoder BaseEncoder;

  private SparkMaxPIDController BaseJointPID;
  //ticks per rev: 8192
  //BaseEncoder.setPositionConversionFactor(2*Math.PI); ???
  private double conversionFactor = 1/8192 * 360;

  private double targetPosition;
  
  /** Creates a new BaseJoint. */
  public BaseJoint() {
    LeftBaseMotor = new CANSparkMax(ArmConstants.kLeftBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightBaseMotor = new CANSparkMax(ArmConstants.kRightBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    BaseEncoder = RightBaseMotor.getAbsoluteEncoder(Type.kDutyCycle);

    LeftBaseMotor.follow(RightBaseMotor, false);
    
    RightBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);
    LeftBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    BaseJointPID = RightBaseMotor.getPIDController();
    BaseJointPID.setFF(ArmConstants.kBaseJointFF);
    BaseJointPID.setP(ArmConstants.kBaseJointP, 0);
    BaseJointPID.setI(ArmConstants.kBaseJointI, 0);
    BaseJointPID.setD(ArmConstants.kBaseJointD, 0);
    BaseJointPID.setSmartMotionMaxVelocity(ArmConstants.kBaseJointMaxVelocity, 0);
    BaseJointPID.setSmartMotionMaxAccel(ArmConstants.kBaseJointMaxAcceleration, 0);
    BaseJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kBaseJointTolerance, 0);

  }

  public double getAngle() {
    return BaseEncoder.getPosition() * conversionFactor;
  }

  public void setTarget(double targetPosition) {
    this.targetPosition = targetPosition;
    BaseJointPID.setReference(-targetPosition, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(targetPosition + getAngle()) < ArmConstants.kBaseJointTolerance;
  }

  public void disable() {
    RightBaseMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Base Joint Encoder", BaseEncoder.getPosition());
    SmartDashboard.putNumber("Base Joint Angle", getAngle());
  }
}
