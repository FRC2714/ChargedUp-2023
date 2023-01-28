// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
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
  private double conversionFactor = 1/8192 * 2*Math.PI;

  private double targetAngle;
  private double targetPosition;
  
  /** Creates a new BaseJoint. */
  public BaseJoint() {
    LeftBaseMotor = new CANSparkMax(ArmConstants.kLeftBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightBaseMotor = new CANSparkMax(ArmConstants.kRightBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    BaseEncoder = RightBaseMotor.getAbsoluteEncoder(Type.kDutyCycle);

    LeftBaseMotor.follow(RightBaseMotor, true);

    RightBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);
    LeftBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    //RightBaseMotor.setIdleMode(IdleMode.kCoast);
    //LeftBaseMotor.setIdleMode(IdleMode.kCoast);

    BaseJointPID = RightBaseMotor.getPIDController();
    BaseJointPID.setPositionPIDWrappingEnabled(false);
    BaseJointPID.setPositionPIDWrappingMinInput(0);
    BaseJointPID.setPositionPIDWrappingMinInput(Math.PI);
    BaseJointPID.setFeedbackDevice(BaseEncoder);

    BaseJointPID.setFF(ArmConstants.kBaseJointFF, 0);
    BaseJointPID.setP(ArmConstants.kBaseJointP, 0);
    BaseJointPID.setI(ArmConstants.kBaseJointI, 0);
    BaseJointPID.setD(ArmConstants.kBaseJointD, 0);
    BaseJointPID.setSmartMotionMaxVelocity(ArmConstants.kBaseJointMaxVelocity, 0);
    BaseJointPID.setSmartMotionMaxAccel(ArmConstants.kBaseJointMaxAcceleration, 0);
    BaseJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kBaseJointTolerance, 0);

  }

  public double getAngle() {
    if ((BaseEncoder.getPosition() * 2*Math.PI) > Math.PI) {
      return (BaseEncoder.getPosition() * 2*Math.PI)-(2*Math.PI);
    } else {
      return BaseEncoder.getPosition() * 2*Math.PI;
    }
    
  }

  public void setTarget(double targetAngle) {
    this.targetAngle = targetAngle/(2*Math.PI);
    BaseJointPID.setReference(-targetAngle * 2*Math.PI, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(targetPosition + getAngle()) < ArmConstants.kBaseJointTolerance;
  }

  public void disable() {
    RightBaseMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("BaseJoint Encoder", BaseEncoder.getPosition());
    SmartDashboard.putNumber("BaseJoint Current Angle", Units.radiansToDegrees(getAngle()));

    SmartDashboard.putNumber("BaseJoint Target Position", targetPosition);
  }
}
