// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class Joint extends SubsystemBase {
  private CANSparkMax RightJointMotor;
  private CANSparkMax LeftJointMotor;
  private AbsoluteEncoder JointEncoder;

  private ProfiledPIDController JointController;

  private double targetKinematicAngle;

  private double KinematicOffset;
  private double GearRatio;

  private double nearSetpointTolerance;
  
  /** Creates a new Joint. */
  public Joint(
    int RightMotorCanId, 
    int LeftMotorCanId, 
    boolean isMotorsInvertedRelative, 
    boolean isRightJointMotorInverted, 
    double EncoderPositionConversionFactor , 
    boolean isEncoderInverted,
    double EncoderOffset,
    ProfiledPIDController JointController,
    double nearSetpointTolerance,
    double KinematicOffset,
    double GearRatio) {

    RightJointMotor = new CANSparkMax(RightMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftJointMotor = new CANSparkMax(LeftMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftJointMotor.follow(RightJointMotor, isMotorsInvertedRelative);

    RightJointMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);
    LeftJointMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    RightJointMotor.setInverted(isRightJointMotorInverted);
    RightJointMotor.setIdleMode(IdleMode.kBrake);
    LeftJointMotor.setIdleMode(IdleMode.kBrake);

    JointEncoder = RightJointMotor.getAbsoluteEncoder(Type.kDutyCycle);
    JointEncoder.setPositionConversionFactor(EncoderPositionConversionFactor);
    JointEncoder.setInverted(isEncoderInverted);
    JointEncoder.setZeroOffset(EncoderOffset);
    //todo set velocity conversion factor

    RightJointMotor.burnFlash();
    LeftJointMotor.burnFlash();

    this.JointController = JointController;

    this.KinematicOffset = KinematicOffset;
    this.GearRatio = GearRatio;
    this.nearSetpointTolerance = nearSetpointTolerance;
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= KinematicOffset; //subtract kinematic offset
    kinematicAngle /= GearRatio; //divide by gear ratio

    //convert 0,360 to -180,180
    kinematicAngle -= (2*Math.PI);

    return kinematicAngle;
  }

  private double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    double sparkAngle = kinematicAngle;

    //convert -180,180 to 0,360
    sparkAngle += (2*Math.PI);

    sparkAngle *= KinematicOffset; //multiply by gear ratio
    sparkAngle += GearRatio; //add kinematic offset

    return sparkAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(JointEncoder.getPosition());
  }

  public void setTargetKinematicAngle(double targetAngleRadians) {
    this.targetKinematicAngle = targetAngleRadians;
    JointController.setGoal(targetAngleRadians);
  }

  public double getTargetKinematicAngle() {
    return targetKinematicAngle;
  }

  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - targetKinematicAngle) < Units.degreesToRadians(nearSetpointTolerance);
  }

  public boolean atSetpoint() {
    return JointController.atSetpoint();
  }

  public void setConstraints(Constraints constraints) {
    JointController.setConstraints(constraints);
  }

  public void disable() {
    RightJointMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}
