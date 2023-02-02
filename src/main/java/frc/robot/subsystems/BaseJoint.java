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

  private double targetAngle;
  
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
    BaseEncoder.setPositionConversionFactor(2*Math.PI);

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

  public double convertAngle(double angle) {
    if ((angle)> Math.PI) {
      return (angle) - (2*Math.PI);
    } else {
      return angle;
    }
  }

  public double getAngle() {
    return convertAngle(BaseEncoder.getPosition());
  }

  public void setTarget(double target) {
    this.targetAngle = convertAngle(target);
    BaseJointPID.setReference(targetAngle, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(targetAngle + getAngle()) < ArmConstants.kBaseJointTolerance;
  }

  public void disable() {
    RightBaseMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("BaseJoint Encoder", BaseEncoder.getPosition());
    SmartDashboard.putNumber("BaseJoint Current Angle", Units.radiansToDegrees(getAngle()));
    SmartDashboard.putNumber("BaseJoint Target Position", targetAngle);
  }
}
