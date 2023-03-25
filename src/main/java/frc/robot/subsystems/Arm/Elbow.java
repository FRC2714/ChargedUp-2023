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
  private Constraints CloseConstraints = new Constraints(15, 15, 10);

  private AsymmetricProfiledPIDController ElbowController = new AsymmetricProfiledPIDController(5,0,0, FarConstraints);

  //private ArmFeedforward elbowFeedForward = new ArmFeedforward(0, 0.12, 4.38);
  
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
    ElbowEncoder.setZeroOffset(230.2364949);
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

    return -kinematicAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(ElbowEncoder.getPosition());
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    Constraints selectedConstraint = (Math.abs(targetAngleRadians - getKinematicAngle()) > Units.degreesToRadians(45)) ? FarConstraints : CloseConstraints;
    ElbowController.setConstraints(selectedConstraint);
    SmartDashboard.putString("second joint selected constraint", selectedConstraint.equals(FarConstraints) ? "FAR CONSTRAINT" : "CLOSE CONSTRAINT");

    ElbowController.setGoal(new State(targetAngleRadians, 0));
  }

  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - ElbowController.getGoal().position) < Units.degreesToRadians(8);
  }

  public boolean atSetpoint() {
    return ElbowController.atGoal();
  }

  private void setCalculatedVoltage() {
    RightElbowMotor.setVoltage(
      ElbowController.calculate(getKinematicAngle())
      //elbowFeedForward.calculate(ElbowController.getSetpoint().position, 0)
      );
  }

  @Override
  public void periodic() {
    setCalculatedVoltage();

    // SmartDashboard.putNumber("Elbow Encoder Position", ElbowEncoder.getPosition());
    // SmartDashboard.putNumber("Elbow Encoder Velocity", ElbowEncoder.getVelocity());
    // SmartDashboard.putBoolean("Elbow nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("Elbow Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
