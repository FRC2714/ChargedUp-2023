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
  private double currentSparkAngle;
  private double currentKinematicAngle;
  
  /** Creates a new SecondJoint. */
  public SecondJoint() {
    SecondJointMotor = new CANSparkMax(ArmConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    SecondJointEncoder = SecondJointMotor.getAbsoluteEncoder(Type.kDutyCycle);

    //todo motor should not be inverted
    SecondJointMotor.setInverted(false);
    //need to 
    //SecondJointMotor.setIdleMode(IdleMode.kCoast);
    SecondJointEncoder.setPositionConversionFactor(2*Math.PI *125); //multiply by gear ratio?
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
    //todo set second joint encoder to inverted
    // 3.5 radians = 200 degrees-ish
    sparkAngle -= 350;//350 = difference from kinematic 0 to sparkmax 0 in radians
    
    sparkAngle /= 125; //divide by gear ratio
    //convert 0,360 to -180,180
    if ((sparkAngle) > Math.PI) { //when angle > 180, convert to -, then negate
      sparkAngle = -((sparkAngle) - (2*Math.PI));
      currentKinematicAngle = sparkAngle;
      SmartDashboard.putNumber("SecondJoint Kinematic Angle > 180", Units.radiansToDegrees(currentKinematicAngle));
      return currentKinematicAngle;
    } else { //when angle < 180, convert to +, then negate
      currentKinematicAngle = -sparkAngle;
      SmartDashboard.putNumber("SecondJoint Kinematic Angle < 180", Units.radiansToDegrees(currentKinematicAngle));
      return currentKinematicAngle;
    }
  }

  public double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    //convert -180,180 to 0,360
    kinematicAngle += Math.PI;
    if (kinematicAngle < 0) { //when angle is -, convert to 180,360
      kinematicAngle = -(kinematicAngle + (Math.PI));
      SmartDashboard.putNumber("SecondJoint Calculated SparkMax Angle < 0", Units.radiansToDegrees(currentSparkAngle));
    } else { //when angle is +, convert to 0,180
      kinematicAngle = -(kinematicAngle- Math.PI);
      SmartDashboard.putNumber("SecondJoint Calculated SparkMax Angle > 0", Units.radiansToDegrees(currentSparkAngle));
    }
    kinematicAngle *=125; //multiply by gear ratio
    kinematicAngle += 350;
    currentSparkAngle = kinematicAngle;
    SmartDashboard.putNumber("SecondJoint SparkMax Angle", Units.radiansToDegrees(currentSparkAngle));
    return currentSparkAngle;
  }

  public double getAngle() {
    return convertAngleFromSparkMaxToKinematic(SecondJointEncoder.getPosition());
  }

  public void setTarget(double targetAngle) {
    this.targetAngle = targetAngle;
    SecondJointPID.setReference(convertAngleFromKinematicToSparkMax(targetAngle), CANSparkMax.ControlType.kSmartMotion, 0);
    SmartDashboard.putNumber("Second joint refernece", convertAngleFromKinematicToSparkMax(targetAngle));
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
    SmartDashboard.putNumber("SecondJoint Target Position", targetAngle);
  }
}