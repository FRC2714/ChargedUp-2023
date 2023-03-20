// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class SecondJoint extends SubsystemBase {
  private CANSparkMax RightSecondJointMotor;
  private CANSparkMax LeftSecondJointMotor;
  private AbsoluteEncoder SecondJointEncoder;

  private ProfiledPIDController SecondJointController;
  private Constraints SecondJointConstraints;

  private double targetAngle;
  
  /** Creates a new SecondJoint. */
  public SecondJoint() {
    RightSecondJointMotor = new CANSparkMax(ArmConstants.kRightSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftSecondJointMotor = new CANSparkMax(ArmConstants.kLeftSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftSecondJointMotor.follow(RightSecondJointMotor, false);

    RightSecondJointMotor.setSmartCurrentLimit(ArmConstants.kSecondJointMotorCurrentLimit);
    LeftSecondJointMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    RightSecondJointMotor.setInverted(ArmConstants.kSecondJointMotorInverted);
    RightSecondJointMotor.setIdleMode(IdleMode.kBrake);
    LeftSecondJointMotor.setIdleMode(IdleMode.kBrake);
    
    SecondJointEncoder = RightSecondJointMotor.getAbsoluteEncoder(Type.kDutyCycle);
    SecondJointEncoder.setPositionConversionFactor(ArmConstants.kSecondJointPositionConversionFactor);
    SecondJointEncoder.setInverted(ArmConstants.kSecondJointEncoderInverted);
    SecondJointEncoder.setZeroOffset(230.2364949);
    //todo set velocity conversion factor

    RightSecondJointMotor.burnFlash();
    LeftSecondJointMotor.burnFlash();

    SecondJointConstraints = new Constraints(0, 0);
    SecondJointController = new ProfiledPIDController(0, 0, 0, SecondJointConstraints);
    SecondJointController.setTolerance(0);
    SecondJointController.disableContinuousInput();
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
    return convertAngleFromSparkMaxToKinematic(SecondJointEncoder.getPosition());
  }

  public void setTargetKinematicAngle(double targetAngleRadians) {
    this.targetAngle = targetAngleRadians;
    SmartDashboard.putNumber("BaseJoint Target Kinematic Angle", Units.radiansToDegrees(targetAngleRadians));
    //SmartDashboard.putNumber("BaseJoint Target SparkMax Position", convertAngleFromKinematicToSparkMax(targetAngle));
    SecondJointController.setGoal(targetAngleRadians);
  }

  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - targetAngle) < Units.degreesToRadians(8);
  }

  public boolean atSetpoint() {
    return SecondJointController.atSetpoint();
  }

  public void disable() {
    RightSecondJointMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SecondJoint Encoder", SecondJointEncoder.getPosition());
    SmartDashboard.putBoolean("SecondJoint nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("SecondJoint Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
