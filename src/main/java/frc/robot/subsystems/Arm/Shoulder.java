// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.utils.controller.AsymmetricProfiledPIDController;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.State;


public class Shoulder extends SubsystemBase {
  private CANSparkMax RightShoulderMotor;
  private CANSparkMax LeftShoulderMotor;
  private AbsoluteEncoder ShoulderEncoder;

  private AsymmetricProfiledPIDController ShoulderController = 
    new AsymmetricProfiledPIDController(0,0,0, ShoulderConstants.kFarConstraints); //MUST START AT 0 P
  
  /** Creates a new Shoulder. */
  public Shoulder() {
    LeftShoulderMotor = new CANSparkMax(ShoulderConstants.kLeftShoulderMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightShoulderMotor = new CANSparkMax(ShoulderConstants.kRightShoulderMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftShoulderMotor.follow(RightShoulderMotor, true);
    RightShoulderMotor.setInverted(ShoulderConstants.kShoulderEncoderInverted); //must be inverted
    RightShoulderMotor.setIdleMode(IdleMode.kBrake);
    LeftShoulderMotor.setIdleMode(IdleMode.kBrake);
    RightShoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderMotorCurrentLimit);
    LeftShoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderMotorCurrentLimit);

    ShoulderEncoder = RightShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    ShoulderEncoder.setPositionConversionFactor(ShoulderConstants.kShoulderPositionConversionFactor);
    ShoulderEncoder.setInverted(ShoulderConstants.kShoulderEncoderInverted); //must be inverted
    ShoulderEncoder.setZeroOffset(ShoulderConstants.kShoulderEncoderZeroOffset);
    //todo set velocity conversion factor

    RightShoulderMotor.burnFlash();
    LeftShoulderMotor.burnFlash();

    ShoulderController.disableContinuousInput();
  }

  private double convertEncoderTicksToKinematicAngle(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ShoulderConstants.kShoulderKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ShoulderConstants.kShoulderGearRatio; //divide by gear ratio

    return kinematicAngle;
  }

  public double getKinematicAngle() {
    return convertEncoderTicksToKinematicAngle(ShoulderEncoder.getPosition());
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    SmartDashboard.putNumber("Shoulder Target Angle", Units.radiansToDegrees(targetAngleRadians));
    ShoulderController.setP(ShoulderConstants.kShoulderP);
    Constraints selectedConstraint = 
      (Math.abs(targetAngleRadians - getKinematicAngle()) < Units.degreesToRadians(20)) ? 
      ShoulderConstants.kCloseConstraints : ShoulderConstants.kFarConstraints;
    ShoulderController.setConstraints(selectedConstraint);
    SmartDashboard.putString("Shoulder Selected Constraint", selectedConstraint.equals(ShoulderConstants.kFarConstraints) ? "FAR" : "CLOSE");

    ShoulderController.setGoal(new State(targetAngleRadians, 0));
  }
  
  public boolean nearGoal() {
    return Math.abs(getKinematicAngle() - ShoulderController.getGoal().position) < Units.degreesToRadians(4);
  }

  public boolean atGoal() {
    return ShoulderController.atGoal();
  }

  private void setCalculatedVoltage() {
    double voltage =
      ShoulderController.calculate(getKinematicAngle())
      + ShoulderConstants.kShoulderFeedForward.calculate(ShoulderController.getSetpoint().position, 0);
    SmartDashboard.putNumber("Shoulder Voltage", voltage);

    RightShoulderMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    setCalculatedVoltage();

    // SmartDashboard.putNumber("Shoulder Encoder Position", ShoulderEncoder.getPosition());
    // SmartDashboard.putNumber("Shoulder Encoder Velocity", ShoulderEncoder.getVelocity());
    
    SmartDashboard.putBoolean("Shoulder nearSetpoint", nearGoal());
    SmartDashboard.putNumber("Shoulder Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
