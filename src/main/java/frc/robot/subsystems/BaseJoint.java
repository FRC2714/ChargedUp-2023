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
    BaseEncoder = RightBaseMotor.getAbsoluteEncoder(Type.kDutyCycle);

    LeftBaseMotor.follow(RightBaseMotor, true);

    RightBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);
    LeftBaseMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    RightBaseMotor.setInverted(true);
    //RightBaseMotor.setIdleMode(IdleMode.kCoast);
    //LeftBaseMotor.setIdleMode(IdleMode.kCoast);
    BaseEncoder.setPositionConversionFactor(2*Math.PI * 240);
    BaseEncoder.setInverted(true);
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
    sparkAngle -= 200;//350 = difference from kinematic 0 to sparkmax 0 approx 160deg
    sparkAngle /= 240; //divide by gear ratio
    //convert 0,360 to -180,180
    if ((sparkAngle) > Math.PI) { //when angle > 180, convert to -, then negate
      sparkAngle = ((sparkAngle) - (2*Math.PI));
      currentKinematicAngle = sparkAngle;
      SmartDashboard.putNumber("BaseJoint Kinematic Angle > 180", Units.radiansToDegrees(currentKinematicAngle));
      return currentKinematicAngle;
    } else { //when angle < 180, convert to +, then negate
      currentKinematicAngle = sparkAngle;
      SmartDashboard.putNumber("BaseJoint Kinematic Angle < 180", Units.radiansToDegrees(currentKinematicAngle));
      return currentKinematicAngle;
    }
  }

  public double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    //convert -180,180 to 0,360
    kinematicAngle += Math.PI;
    if (kinematicAngle < 0) { //when angle is -, convert to 180,360 then negate
      kinematicAngle = (kinematicAngle + (Math.PI));
      SmartDashboard.putNumber("BaseJoint Calculated SparkMax Angle < 0", Units.radiansToDegrees(currentSparkAngle));
    } else { //when angle is +, convert to 0,180 then negate
      kinematicAngle = (kinematicAngle - Math.PI);
      SmartDashboard.putNumber("BaseJoint Calculated SparkMax Angle > 0", Units.radiansToDegrees(currentSparkAngle));
    }
    kinematicAngle *=240; //multiply by gear ratio
    kinematicAngle += 200; //add kinematic offset
    currentSparkAngle = kinematicAngle;
    return currentSparkAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(BaseEncoder.getPosition());
  }

  public void setTarget(double target) {
    this.targetAngle = targetAngle;
    BaseJointPID.setReference(convertAngleFromKinematicToSparkMax(target), CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(targetAngle + getKinematicAngle()) < ArmConstants.kBaseJointTolerance;
  }

  public void disable() {
    RightBaseMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("BaseJoint Encoder", BaseEncoder.getPosition());
    SmartDashboard.putNumber("BaseJoint Target Position", targetAngle);
  }
}
