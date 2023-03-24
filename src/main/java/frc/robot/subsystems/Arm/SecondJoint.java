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

public class SecondJoint extends SubsystemBase {
  private CANSparkMax RightSecondJointMotor;
  private CANSparkMax LeftSecondJointMotor;
  private AbsoluteEncoder SecondJointEncoder;

  private Constraints FarConstraints = new Constraints(12, 9, 6);
  private Constraints CloseConstraints = new Constraints(15, 15, 10);

  private AsymmetricProfiledPIDController SecondJointController = new AsymmetricProfiledPIDController(5,0,0, FarConstraints);

  //private ArmFeedforward secondJointFeedForward = new ArmFeedforward(0, 0.12, 4.38);
  
  /** Creates a new SecondJoint. */
  public SecondJoint() {
    RightSecondJointMotor = new CANSparkMax(ArmConstants.kRightSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftSecondJointMotor = new CANSparkMax(ArmConstants.kLeftSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    LeftSecondJointMotor.follow(RightSecondJointMotor, false);

    RightSecondJointMotor.setSmartCurrentLimit(ArmConstants.kSecondJointMotorCurrentLimit);
    LeftSecondJointMotor.setSmartCurrentLimit(ArmConstants.kBaseJointMotorCurrentLimit);

    RightSecondJointMotor.setInverted(false); //was true
    RightSecondJointMotor.setIdleMode(IdleMode.kBrake);
    LeftSecondJointMotor.setIdleMode(IdleMode.kBrake);
    
    SecondJointEncoder = RightSecondJointMotor.getAbsoluteEncoder(Type.kDutyCycle);
    SecondJointEncoder.setPositionConversionFactor(ArmConstants.kSecondJointPositionConversionFactor);
    SecondJointEncoder.setInverted(ArmConstants.kSecondJointEncoderInverted);
    SecondJointEncoder.setZeroOffset(230.2364949);
    //todo set velocity conversion factor

    // RightSecondJointMotor.setSoftLimit(SoftLimitDirection.kReverse, 20);
    // RightSecondJointMotor.setSoftLimit(SoftLimitDirection.kForward, 1240);

    RightSecondJointMotor.burnFlash();
    LeftSecondJointMotor.burnFlash();

    
    SecondJointController.disableContinuousInput();
    //SecondJointController.setTolerance(Units.degreesToRadians(2), 0);
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ArmConstants.kSecondJointKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ArmConstants.kSecondJointGearRatio; //divide by gear ratio

    return -kinematicAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(SecondJointEncoder.getPosition());
  }

  public void setTargetKinematicAngleRadians(double targetAngleRadians) {
    Constraints selectedConstraint = (Math.abs(targetAngleRadians - getKinematicAngle()) > Units.degreesToRadians(45)) ? FarConstraints : CloseConstraints;
    SecondJointController.setConstraints(selectedConstraint);
    SmartDashboard.putString("second joint selected constraint", selectedConstraint.equals(FarConstraints) ? "FAR CONSTRAINT" : "CLOSE CONSTRAINT");

    SecondJointController.setGoal(new State(targetAngleRadians, 0));
  }

  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - SecondJointController.getGoal().position) < Units.degreesToRadians(8);
  }

  public boolean atSetpoint() {
    return SecondJointController.atGoal();
  }

  private void setVoltage() {
    RightSecondJointMotor.setVoltage(
      SecondJointController.calculate(getKinematicAngle())
      //secondJointFeedForward.calculate(SecondJointController.getSetpoint().position, 0)
      );
  }

  @Override
  public void periodic() {
    setVoltage();

    // SmartDashboard.putNumber("SecondJoint Encoder Position", SecondJointEncoder.getPosition());
    // SmartDashboard.putNumber("SecondJoint Encoder Velocity", SecondJointEncoder.getVelocity());
    // SmartDashboard.putBoolean("SecondJoint nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("SecondJoint Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
