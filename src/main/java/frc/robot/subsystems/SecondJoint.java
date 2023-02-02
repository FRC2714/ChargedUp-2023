// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class SecondJoint extends SubsystemBase {
  private CANSparkMax SecondJointMotor;
  private AbsoluteEncoder SecondJointEncoder;
  private SparkMaxPIDController SecondJointPID;

  private double targetAngle;
  
  /** Creates a new SecondJoint. */
  public SecondJoint() {
    SecondJointMotor = new CANSparkMax(ArmConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    SecondJointEncoder = SecondJointMotor.getAbsoluteEncoder(Type.kDutyCycle);

    //SecondJointMotor.setIdleMode(IdleMode.kCoast);
    SecondJointEncoder.setPositionConversionFactor(2*Math.PI);

    SecondJointPID = SecondJointMotor.getPIDController();
    SecondJointPID.setPositionPIDWrappingEnabled(false);
    SecondJointPID.setPositionPIDWrappingMinInput(0);
    SecondJointPID.setPositionPIDWrappingMinInput(Math.PI);

    SecondJointPID.setFeedbackDevice(SecondJointEncoder);
    SecondJointPID.setFF(ArmConstants.kSecondJointFF, 0);
    SecondJointPID.setP(ArmConstants.kSecondJointP, 0);
    SecondJointPID.setI(ArmConstants.kSecondJointI, 0);
    SecondJointPID.setD(ArmConstants.kSecondJointD, 0);
    SecondJointPID.setSmartMotionMaxVelocity(ArmConstants.kSecondJointMaxVelocity, 0);
    SecondJointPID.setSmartMotionMaxAccel(ArmConstants.kSecondJointMaxAcceleration, 0);
    SecondJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kSecondJointTolerance, 0);
  }

  public double convertAngle(double angle) {
    if ((angle)> Math.PI) {
      return (angle) - (2*Math.PI);
    } else {
      return angle;
    }
  }

  public double getAngle() {
    return convertAngle(SecondJointEncoder.getPosition());
  }

  public void setTarget(double targetAngle) {
    this.targetAngle = targetAngle;
    SecondJointPID.setReference(targetAngle, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(targetAngle + getAngle()) < ArmConstants.kSecondJointTolerance;
  }

  public void disable() {
    SecondJointMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SecondJoint Encoder", SecondJointEncoder.getPosition());
    SmartDashboard.putNumber("SecondJoint Current Angle", Units.radiansToDegrees(getAngle()));
    SmartDashboard.putNumber("SecondJoint Target Position", targetAngle);
  }
}
