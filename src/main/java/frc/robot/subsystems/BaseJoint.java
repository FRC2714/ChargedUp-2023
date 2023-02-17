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
  private double currentSparkAngle;
  private double currentKinematicAngle;
  
  /** Creates a new BaseJoint. */
  public BaseJoint() {
    LeftBaseMotor = new CANSparkMax(ArmConstants.kLeftBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightBaseMotor = new CANSparkMax(ArmConstants.kRightBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftBaseMotor.follow(RightBaseMotor, true);

    RightBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);
    LeftBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    RightBaseMotor.setInverted(ArmConstants.kBaseJointInverted); //must be inverted
    RightBaseMotor.setIdleMode(IdleMode.kBrake);
    LeftBaseMotor.setIdleMode(IdleMode.kBrake);

    BaseEncoder = RightBaseMotor.getAbsoluteEncoder(Type.kDutyCycle);
    BaseEncoder.setPositionConversionFactor(ArmConstants.kBaseJointPositionConversionFactor);
    BaseEncoder.setInverted(ArmConstants.kBaseJointInverted); //must be inverted
    //todo set velocity conversion factor

    BaseJointPID = RightBaseMotor.getPIDController();
    BaseJointPID.setPositionPIDWrappingEnabled(false);
    BaseJointPID.setFeedbackDevice(BaseEncoder);
    BaseJointPID.setFF(ArmConstants.kBaseJointFF, 0);
    BaseJointPID.setP(ArmConstants.kBaseJointP, 0);
    BaseJointPID.setI(ArmConstants.kBaseJointI, 0);
    BaseJointPID.setD(ArmConstants.kBaseJointD, 0);
    BaseJointPID.setSmartMotionMaxVelocity(ArmConstants.kBaseJointMaxVelocity, 0);
    BaseJointPID.setSmartMotionMaxAccel(ArmConstants.kBaseJointMaxAcceleration, 0);
    BaseJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kBaseJointTolerance, 0);
  }

  public double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    sparkAngle -= ArmConstants.kBaseJointKinematicOffset; //subtract kinematic offset
    sparkAngle /= ArmConstants.kBaseJointGearRatio; //divide by gear ratio
    //convert 0,360 to -180,180
    if ((sparkAngle) > Math.PI) { //when angle > 180, convert to -
      sparkAngle = ((sparkAngle) - (2*Math.PI));
      SmartDashboard.putNumber("BaseJoint Calculated Kinematic Angle > 180", Units.radiansToDegrees(currentKinematicAngle));
    } else { //when angle < 180, convert to +
      SmartDashboard.putNumber("BaseJoint Calculated Kinematic Angle < 180", Units.radiansToDegrees(currentKinematicAngle));
    }
    currentKinematicAngle = sparkAngle;
    return currentKinematicAngle;
  }

  public double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    //convert -180,180 to 0,360
    kinematicAngle += Math.PI; //add 180
    if (kinematicAngle < 0) { //when angle is -, convert to 180,360
      kinematicAngle = (kinematicAngle + (Math.PI));
      SmartDashboard.putNumber("BaseJoint Calculated SparkMax Position < 0", Units.radiansToDegrees(currentSparkAngle));
    } else { //when angle is +, convert to 0,180
      kinematicAngle = (kinematicAngle - Math.PI);
      SmartDashboard.putNumber("BaseJoint Calculated SparkMax Position > 0", Units.radiansToDegrees(currentSparkAngle));
    }
    kinematicAngle *= ArmConstants.kBaseJointGearRatio; //multiply by gear ratio
    kinematicAngle += ArmConstants.kBaseJointKinematicOffset; //add kinematic offset
    currentSparkAngle = kinematicAngle;
    return currentSparkAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(BaseEncoder.getPosition());
  }

  public void setTargetKinematicAngle(double targetAngle) {
    this.targetAngle = targetAngle;
    SmartDashboard.putNumber("BaseJoint Target Kinematic Angle", targetAngle);
    SmartDashboard.putNumber("BaseJoint Target SparkMax Position", convertAngleFromKinematicToSparkMax(targetAngle));
    BaseJointPID.setReference(convertAngleFromKinematicToSparkMax(targetAngle), CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(getKinematicAngle() - targetAngle) < convertAngleFromSparkMaxToKinematic(ArmConstants.kBaseJointTolerance);
  }

  public void disable() {
    RightBaseMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("BaseJoint Encoder", BaseEncoder.getPosition());
    SmartDashboard.putBoolean("BaseJoint atSetpoint", atSetpoint());
  }
}
