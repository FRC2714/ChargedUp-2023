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


public class Shoulder extends SubsystemBase {
  private CANSparkMax RightShoulderMotor;
  private CANSparkMax LeftShoulderMotor;
  private AbsoluteEncoder ShoulderEncoder;

  private Constraints FarConstraints = new Constraints(8, 6, 4);
  private Constraints CloseConstraints = new Constraints(7.5, 7.5, 5);

  private AsymmetricProfiledPIDController ShoulderController = new AsymmetricProfiledPIDController(5,0,0, FarConstraints);
  
  /** Creates a new Shoulder. */
  public Shoulder() {
    LeftShoulderMotor = new CANSparkMax(ArmConstants.kLeftShoulderMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    RightShoulderMotor = new CANSparkMax(ArmConstants.kRightShoulderMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftShoulderMotor.follow(RightShoulderMotor, true);

    RightShoulderMotor.setSmartCurrentLimit(ArmConstants.kShoulderMotorCurrentLimit);
    LeftShoulderMotor.setSmartCurrentLimit(ArmConstants.kShoulderMotorCurrentLimit);

    RightShoulderMotor.setInverted(ArmConstants.kShoulderEncoderInverted); //must be inverted
    RightShoulderMotor.setIdleMode(IdleMode.kBrake);
    LeftShoulderMotor.setIdleMode(IdleMode.kBrake);

    ShoulderEncoder = RightShoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    ShoulderEncoder.setPositionConversionFactor(ArmConstants.kShoulderPositionConversionFactor);
    ShoulderEncoder.setInverted(ArmConstants.kShoulderEncoderInverted); //must be inverted
    ShoulderEncoder.setZeroOffset(1049.0689405);
    //todo set velocity conversion factor

    RightShoulderMotor.burnFlash();
    LeftShoulderMotor.burnFlash();

    ShoulderController.disableContinuousInput();
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ArmConstants.kShoulderKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ArmConstants.kShoulderGearRatio; //divide by gear ratio

    return kinematicAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(ShoulderEncoder.getPosition());
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    Constraints selectedConstraint = (Math.abs(targetAngleRadians - getKinematicAngle()) > Units.degreesToRadians(20)) ? FarConstraints : CloseConstraints;
    ShoulderController.setConstraints(selectedConstraint);
    SmartDashboard.putString("base joint selected constraint", selectedConstraint.equals(FarConstraints) ? "FAR CONSTRAINT" : "CLOSE CONSTRAINT");

    ShoulderController.setGoal(new State(targetAngleRadians, 0));
  }
  
  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - ShoulderController.getGoal().position) < Units.degreesToRadians(4);
  }

  public boolean atSetpoint() {
    return ShoulderController.atGoal();
  }

  private void setCalculatedVoltage() {
    RightShoulderMotor.setVoltage(
      ShoulderController.calculate(getKinematicAngle()));
  }

  @Override
  public void periodic() {
    setCalculatedVoltage();

    // SmartDashboard.putNumber("Shoulder Encoder Position", ShoulderEncoder.getPosition());
    // SmartDashboard.putNumber("Shoulder Encoder Velocity", ShoulderEncoder.getVelocity());
    // SmartDashboard.putBoolean("Shoulder nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("Shoulder Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
