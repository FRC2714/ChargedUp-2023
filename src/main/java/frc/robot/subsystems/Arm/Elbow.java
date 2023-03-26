// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.controller.AsymmetricProfiledPIDController;
import frc.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.utils.controller.AsymmetricTrapezoidProfile.State;

public class Elbow extends SubsystemBase {
  private CANSparkMax RightElbowMotor;
  private CANSparkMax LeftElbowMotor;
  private AbsoluteEncoder ElbowEncoder;

  private Constraints FarConstraints = new Constraints(12, 9, 6);
  private Constraints CloseConstraints = new Constraints(36, 36, 24);

  private AsymmetricProfiledPIDController ElbowController = new AsymmetricProfiledPIDController(0,0,0, FarConstraints);

  private ArmFeedforward ElbowFeedForward = new ArmFeedforward(0, 0.35, 4.38, 0.03);
  
  /** Creates a new Elbow. */
  public Elbow() {
    RightElbowMotor = new CANSparkMax(ArmConstants.kRightElbowMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftElbowMotor = new CANSparkMax(ArmConstants.kLeftElbowMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftElbowMotor.follow(RightElbowMotor, false);

    RightElbowMotor.setSmartCurrentLimit(ArmConstants.kElbowMotorCurrentLimit);
    LeftElbowMotor.setSmartCurrentLimit(ArmConstants.kElbowMotorCurrentLimit);

    RightElbowMotor.setInverted(false); //was true
    RightElbowMotor.setIdleMode(IdleMode.kBrake);
    LeftElbowMotor.setIdleMode(IdleMode.kBrake);
    
    ElbowEncoder = RightElbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    ElbowEncoder.setPositionConversionFactor(ArmConstants.kElbowPositionConversionFactor);
    ElbowEncoder.setInverted(ArmConstants.kElbowEncoderInverted);
    ElbowEncoder.setZeroOffset(ArmConstants.kElbowEncoderZeroOffset);
    //todo set velocity conversion factor

    // RightElbowMotor.setSoftLimit(SoftLimitDirection.kReverse, 20);
    // RightElbowMotor.setSoftLimit(SoftLimitDirection.kForward, 1240);

    RightElbowMotor.burnFlash();
    LeftElbowMotor.burnFlash();

    
    ElbowController.disableContinuousInput();
    //ElbowController.setTolerance(Units.degreesToRadians(2), 0);
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ArmConstants.kElbowKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ArmConstants.kElbowGearRatio; //divide by gear ratio

    return kinematicAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(ElbowEncoder.getPosition());
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    SmartDashboard.putNumber("Elbow Target Angle", Units.radiansToDegrees(targetAngleRadians));
    if(ElbowController.getP() == 0) {ElbowController.setP(5);}
    Constraints selectedConstraint = (Math.abs(targetAngleRadians - getKinematicAngle()) < Units.degreesToRadians(30)) ? CloseConstraints : FarConstraints;
    ElbowController.setConstraints(selectedConstraint);
    SmartDashboard.putString("Elbow Selected Constraint", selectedConstraint.equals(FarConstraints) ? "FAR" : "CLOSE");

    ElbowController.setGoal(new State(targetAngleRadians, 0));
  }

  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - ElbowController.getGoal().position) < Units.degreesToRadians(8);
  }

  public boolean atSetpoint() {
    return ElbowController.atGoal();
  }

  private void setCalculatedVoltage() {
    double voltage =
      ElbowController.calculate(getKinematicAngle()) +
      ElbowFeedForward.calculate(ElbowController.getSetpoint().position, 0);
    SmartDashboard.putNumber("Elbow Voltage", voltage);

    RightElbowMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    setCalculatedVoltage();

    // SmartDashboard.putNumber("Elbow Encoder Position", ElbowEncoder.getPosition());
    // SmartDashboard.putNumber("Elbow Encoder Velocity", ElbowEncoder.getVelocity());
    
    SmartDashboard.putBoolean("Elbow nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("Elbow Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
