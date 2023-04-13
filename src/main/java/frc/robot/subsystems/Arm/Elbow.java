// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElbowConstants;
import frc.robot.utils.controller.AsymmetricProfiledPIDController;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.State;

public class Elbow extends SubsystemBase {
  private CANSparkMax ElbowMotor;
  private AbsoluteEncoder ElbowEncoder;

  private Constraints FarConstraints = new Constraints(12, 12, 9);
  private Constraints CloseConstraints = new Constraints(36, 36, 24);

  private AsymmetricProfiledPIDController ElbowController = new AsymmetricProfiledPIDController(0,0,0, FarConstraints);

  private ArmFeedforward ElbowFeedForward = new ArmFeedforward(0, 0.35, 4.38, 0.03);
  
  /** Creates a new Elbow. */
  public Elbow() {
    ElbowMotor = new CANSparkMax(ElbowConstants.kRightElbowMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);

    ElbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowMotorCurrentLimit);

    ElbowMotor.setInverted(ElbowConstants.kElbowMotorInverted);
    ElbowMotor.setIdleMode(IdleMode.kBrake);
    
    ElbowEncoder = ElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    ElbowEncoder.setPositionConversionFactor(ElbowConstants.kElbowPositionConversionFactor);
    ElbowEncoder.setInverted(ElbowConstants.kElbowEncoderInverted);
    ElbowEncoder.setZeroOffset(ElbowConstants.kElbowEncoderZeroOffset);
    //todo set velocity conversion factor

    // RightElbowMotor.setSoftLimit(SoftLimitDirection.kReverse, 20);
    // RightElbowMotor.setSoftLimit(SoftLimitDirection.kForward, 1240);

    ElbowMotor.burnFlash();

    
    ElbowController.disableContinuousInput();
    //ElbowController.setTolerance(Units.degreesToRadians(2), 0);
  }

  private double convertEncoderTicksToKinematicAngle(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ElbowConstants.kElbowKinematicOffset; //subtract kinematic offset 762.0
    kinematicAngle /= ElbowConstants.kElbowGearRatio; //divide by gear ratio

    return kinematicAngle;
  }

  public double getKinematicAngle() {
    return convertEncoderTicksToKinematicAngle(ElbowEncoder.getPosition());
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    SmartDashboard.putNumber("Elbow Target Angle", Units.radiansToDegrees(targetAngleRadians));
    if(ElbowController.getP() == 0) {ElbowController.setP(ElbowConstants.kElbowP);}
    Constraints selectedConstraint = 
      (Math.abs(targetAngleRadians - getKinematicAngle()) < Units.degreesToRadians(45)) ? 
      CloseConstraints : FarConstraints;
    ElbowController.setConstraints(selectedConstraint);
    SmartDashboard.putString("Elbow Selected Constraint", selectedConstraint.equals(FarConstraints) ? "FAR" : "CLOSE");

    ElbowController.setGoal(new State(targetAngleRadians, 0));
  }

  public boolean nearGoal() {
    return Math.abs(getKinematicAngle() - ElbowController.getGoal().position) < Units.degreesToRadians(8);
  }

  public boolean atGoal() {
    return ElbowController.atGoal();
  }

  private void setCalculatedVoltage() {
    double voltage = 
      ElbowController.calculate(getKinematicAngle())
      + ElbowFeedForward.calculate(ElbowController.getSetpoint().position, 0);
    SmartDashboard.putNumber("Elbow Voltage", voltage);

    ElbowMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    setCalculatedVoltage();

    // SmartDashboard.putNumber("Elbow Encoder Position", ElbowEncoder.getPosition());
    // SmartDashboard.putNumber("Elbow Encoder Velocity", ElbowEncoder.getVelocity());
    
    SmartDashboard.putBoolean("Elbow nearSetpoint", nearGoal());
    SmartDashboard.putNumber("Elbow Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
