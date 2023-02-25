// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

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
  private double currentSparkAngle;
  private double currentKinematicAngle;
  
  /** Creates a new SecondJoint. */
  public SecondJoint() {
    SecondJointMotor = new CANSparkMax(ArmConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    SecondJointMotor.setSmartCurrentLimit(ArmConstants.kSecondJointMotorCurrentLimit);

    SecondJointMotor.setInverted(ArmConstants.kSecondJointInverted); //must not be inverted
    SecondJointMotor.setIdleMode(IdleMode.kBrake);

    SecondJointEncoder = SecondJointMotor.getAbsoluteEncoder(Type.kDutyCycle);
    SecondJointEncoder.setPositionConversionFactor(ArmConstants.kSecondJointPositionConversionFactor);
    SecondJointEncoder.setInverted(ArmConstants.kSecondJointInverted); //must not be inverted
    //todo set velocity conversion factor

    SecondJointPID = SecondJointMotor.getPIDController();
    SecondJointPID.setPositionPIDWrappingEnabled(false);
    SecondJointPID.setFeedbackDevice(SecondJointEncoder);
    SecondJointPID.setFF(ArmConstants.kSecondJointFF, 0);
    SecondJointPID.setP(ArmConstants.kSecondJointP, 0);
    SecondJointPID.setI(ArmConstants.kSecondJointI, 0);
    SecondJointPID.setD(ArmConstants.kSecondJointD, 0);
    SecondJointPID.setSmartMotionMaxVelocity(ArmConstants.kSecondJointMaxVelocity, 0);
    SecondJointPID.setSmartMotionMaxAccel(ArmConstants.kSecondJointMaxAcceleration, 0);
    SecondJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kSecondJointTolerance, 0);
  }

  public double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    sparkAngle -= ArmConstants.kSecondJointKinematicOffset; //subtract kinematic offset
    sparkAngle /= ArmConstants.kSecondJointGearRatio; //divide by gear ratio
    //convert 0,360 to -180,180
    if ((sparkAngle) > Math.PI) { //when angle > 180, convert to -, then negate
      sparkAngle = -((sparkAngle) - (2*Math.PI));
      currentKinematicAngle = sparkAngle;
      //SmartDashboard.putNumber("SecondJoint Calculated Kinematic Angle > 180", Units.radiansToDegrees(currentKinematicAngle));
    } else { //when angle < 180, convert to +, then negate
      currentKinematicAngle = -sparkAngle;
      //SmartDashboard.putNumber("SecondJoint Calculated Kinematic Angle < 180", Units.radiansToDegrees(currentKinematicAngle));
    }
    return currentKinematicAngle;
  }

  public double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    //convert -180,180 to 0,360
    kinematicAngle += Math.PI;
    if (kinematicAngle < 0) { //when angle is -, convert to 180,360 then negate
      kinematicAngle = -(kinematicAngle + (Math.PI));
      //SmartDashboard.putNumber("SecondJoint Calculated SparkMax Position < 0", Units.radiansToDegrees(currentSparkAngle));
    } else { //when angle is +, convert to 0,180 then negate
      kinematicAngle = -(kinematicAngle - Math.PI);
      //SmartDashboard.putNumber("SecondJoint Calculated SparkMax Position > 0", Units.radiansToDegrees(currentSparkAngle));
    }
    kinematicAngle *= ArmConstants.kSecondJointGearRatio; //multiply by gear ratio
    kinematicAngle += ArmConstants.kSecondJointKinematicOffset; //add kinematic offset
    currentSparkAngle = kinematicAngle;
    return currentSparkAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(SecondJointEncoder.getPosition());
  }

  public void setTargetKinematicAngle(double targetAngle) {
    this.targetAngle = targetAngle;
    SmartDashboard.putNumber("SecondJoint Target Kinematic Angle", Units.radiansToDegrees(targetAngle));
    //SmartDashboard.putNumber("SecondJoint Target SparkMax Position", convertAngleFromKinematicToSparkMax(targetAngle));
    SecondJointPID.setReference(convertAngleFromKinematicToSparkMax(targetAngle), CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(getKinematicAngle() - targetAngle) < Units.degreesToRadians(7);
  }

  public void disable() {
    SecondJointMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SecondJoint Encoder", SecondJointEncoder.getPosition());
    SmartDashboard.putBoolean("SecondJoint atSetpoint", atSetpoint());
    SmartDashboard.putNumber("SecondJoint Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
