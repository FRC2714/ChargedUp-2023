// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.controller.AsymmetricTrapezoidProfile;
import frc.utils.controller.ProfiledPositionController;
import frc.utils.controller.AsymmetricTrapezoidProfile.Constraints;


public class BaseJoint extends SubsystemBase {
  private CANSparkMax RightBaseJointMotor;
  private CANSparkMax LeftBaseJointMotor;
  private AbsoluteEncoder BaseEncoder;

  private ProfiledPositionController BaseJointController;
  private SparkMaxPIDController sparkMaxPIDController;
  private Constraints BaseJointConstraints;

  private double targetAngle;
  
  /** Creates a new BaseJoint. */
  public BaseJoint() {
    LeftBaseJointMotor = new CANSparkMax(ArmConstants.kLeftBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightBaseJointMotor = new CANSparkMax(ArmConstants.kRightBaseJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftBaseJointMotor.follow(RightBaseJointMotor, true);

    RightBaseJointMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);
    LeftBaseJointMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    RightBaseJointMotor.setInverted(ArmConstants.kBaseJointEncoderInverted); //must be inverted
    RightBaseJointMotor.setIdleMode(IdleMode.kBrake);
    LeftBaseJointMotor.setIdleMode(IdleMode.kBrake);

    BaseEncoder = RightBaseJointMotor.getAbsoluteEncoder(Type.kDutyCycle);
    BaseEncoder.setPositionConversionFactor(ArmConstants.kBaseJointPositionConversionFactor);
    BaseEncoder.setInverted(ArmConstants.kBaseJointEncoderInverted); //must be inverted
    BaseEncoder.setZeroOffset(1049.0689405);
    //todo set velocity conversion factor

    RightBaseJointMotor.burnFlash();
    LeftBaseJointMotor.burnFlash();

    BaseJointConstraints = new Constraints(0, 0, 0);
    sparkMaxPIDController = RightBaseJointMotor.getPIDController();
    BaseJointController = new ProfiledPositionController(sparkMaxPIDController, BaseJointConstraints);
    BaseJointController.disableContinuousInput();
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ArmConstants.kBaseJointKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ArmConstants.kBaseJointGearRatio; //divide by gear ratio

    //convert 0,360 to -180,180
    kinematicAngle -= (2*Math.PI);

    return kinematicAngle;
  }

  private double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    double sparkAngle = kinematicAngle;

    //convert -180,180 to 0,360
    sparkAngle += (2*Math.PI);

    sparkAngle *= ArmConstants.kBaseJointGearRatio; //multiply by gear ratio
    sparkAngle += ArmConstants.kBaseJointKinematicOffset; //add kinematic offset

    return sparkAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(BaseEncoder.getPosition());
  }

  public void setTargetKinematicAngle(double targetAngleRadians) {
    this.targetAngle = targetAngleRadians;
    SmartDashboard.putNumber("BaseJoint Target Kinematic Angle", Units.radiansToDegrees(targetAngleRadians));
    //SmartDashboard.putNumber("BaseJoint Target SparkMax Position", convertAngleFromKinematicToSparkMax(targetAngle));
    //BaseJointController.set(targetAngleRadians);
    //TODO
  }

  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - targetAngle) < Units.degreesToRadians(4);
  }

  public boolean atGoal() {
    return BaseJointController.atGoal();
  }

  public void disable() {
    RightBaseJointMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("BaseJoint Encoder", BaseEncoder.getPosition());
    SmartDashboard.putBoolean("BaseJoint nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("BaseJoint Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
