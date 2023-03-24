// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.controller.AsymmetricProfiledPIDController;
import frc.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.utils.controller.AsymmetricTrapezoidProfile.State;


public class BaseJoint extends SubsystemBase {
  private CANSparkMax RightBaseJointMotor;
  private CANSparkMax LeftBaseJointMotor;
  private AbsoluteEncoder BaseJointEncoder;

  private Constraints FarConstraints = new Constraints(10, 7, 4);
  private Constraints CloseConstraints = new Constraints(15, 15, 10);

  private AsymmetricProfiledPIDController BaseJointController = new AsymmetricProfiledPIDController(0,0,0, FarConstraints);
  
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

    BaseJointEncoder = RightBaseJointMotor.getAbsoluteEncoder(Type.kDutyCycle);
    BaseJointEncoder.setPositionConversionFactor(ArmConstants.kBaseJointPositionConversionFactor);
    BaseJointEncoder.setInverted(ArmConstants.kBaseJointEncoderInverted); //must be inverted
    BaseJointEncoder.setZeroOffset(1049.0689405);
    //todo set velocity conversion factor

    RightBaseJointMotor.burnFlash();
    LeftBaseJointMotor.burnFlash();

    BaseJointController.disableContinuousInput();
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ArmConstants.kBaseJointKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ArmConstants.kBaseJointGearRatio; //divide by gear ratio

    return kinematicAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(BaseJointEncoder.getPosition());
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    Constraints selectedConstraint = (Math.abs(targetAngleRadians - getKinematicAngle()) > Units.degreesToRadians(20)) ? FarConstraints : CloseConstraints;
    BaseJointController.setConstraints(selectedConstraint);
    SmartDashboard.putString("base joint selected constraint", selectedConstraint.equals(FarConstraints) ? "FAR CONSTRAINT" : "CLOSE CONSTRAINT");

    BaseJointController.setGoal(new State(targetAngleRadians, 0));
  }
  
  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - BaseJointController.getGoal().position) < Units.degreesToRadians(4);
  }

  public boolean atSetpoint() {
    return BaseJointController.atGoal();
  }

  private void setVoltage() {
    RightBaseJointMotor.setVoltage(
      BaseJointController.calculate(getKinematicAngle()));
  }

  @Override
  public void periodic() {
    setVoltage();

    // SmartDashboard.putNumber("BaseJoint Encoder Position", BaseJointEncoder.getPosition());
    // SmartDashboard.putNumber("BaseJoint Encoder Velocity", BaseJointEncoder.getVelocity());
    // SmartDashboard.putBoolean("BaseJoint nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("BaseJoint Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
