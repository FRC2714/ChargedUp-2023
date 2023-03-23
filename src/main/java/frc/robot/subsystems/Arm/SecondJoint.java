// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.controller.ProfiledPositionController;
import frc.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.utils.controller.AsymmetricTrapezoidProfile.State;

public class SecondJoint extends SubsystemBase {
  private CANSparkMax RightSecondJointMotor;
  private CANSparkMax LeftSecondJointMotor;
  private AbsoluteEncoder SecondJointEncoder;

  private ProfiledPositionController SecondJointController;
  private SparkMaxPIDController sparkMaxPIDController;
  private Constraints SecondJointConstraints;

  private SparkMaxPIDController SecondJointPID;

  private State targetState = new State();

  private ArmFeedforward secondJointFeedForward = new ArmFeedforward(0, 0.12, 4.38);

  private double targetAngle;
  
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

    RightSecondJointMotor.setSoftLimit(SoftLimitDirection.kReverse, 20);
    RightSecondJointMotor.setSoftLimit(SoftLimitDirection.kForward, 1240);

    RightSecondJointMotor.burnFlash();
    LeftSecondJointMotor.burnFlash();

    SecondJointConstraints = new Constraints(1, 1, 1);
    sparkMaxPIDController = RightSecondJointMotor.getPIDController();
    SecondJointController = new ProfiledPositionController(sparkMaxPIDController, SecondJointConstraints);
    SecondJointController.disableContinuousInput();

    // SecondJointPID = RightSecondJointMotor.getPIDController();
    // SecondJointPID.setPositionPIDWrappingEnabled(false);
    // SecondJointPID.setFeedbackDevice(SecondJointEncoder);
    // SecondJointPID.setFF(ArmConstants.kSecondJointFF, 0);
    // SecondJointPID.setP(ArmConstants.kSecondJointP, 0);
    // SecondJointPID.setI(ArmConstants.kSecondJointI, 0);
    // SecondJointPID.setD(ArmConstants.kSecondJointD, 0);
    // SecondJointPID.setSmartMotionMaxVelocity(ArmConstants.kSecondJointMaxVelocity, 0);
    // SecondJointPID.setSmartMotionMaxAccel(ArmConstants.kSecondJointMaxAcceleration, 0);
    // SecondJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kSecondJointTolerance, 0);
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ArmConstants.kSecondJointKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ArmConstants.kSecondJointGearRatio; //divide by gear ratio

    //convert 0,360 to -180,180
    //kinematicAngle -= (Math.PI);

    return -kinematicAngle;
  }

  private double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    double sparkAngle = -kinematicAngle;

    //convert -180,180 to 0,360
    //sparkAngle += (2*Math.PI);

    //sparkAngle *= ArmConstants.kSecondJointGearRatio; //multiply by gear ratio
    sparkAngle += ArmConstants.kSecondJointKinematicOffset; //add kinematic offset

    return sparkAngle;
  }

  public double getKinematicAngle() {
    return convertAngleFromSparkMaxToKinematic(SecondJointEncoder.getPosition());
  }

  public void setTargetKinematicAngle(double targetAngleRadians) {
    this.targetAngle = targetAngleRadians;
    SmartDashboard.putNumber("SecondJoint Target Kinematic Angle", Units.radiansToDegrees(targetAngleRadians));
    //SmartDashboard.putNumber("SecondJoint Target SparkMax Position", convertAngleFromKinematicToSparkMax(targetAngle));
    //SecondJointPID.setReference(convertAngleFromKinematicToSparkMax(targetAngle), CANSparkMax.ControlType.kSmartMotion, 0);
    //SecondJointController.setGoal(targetAngleRadians);
    this.targetState = new State(targetAngleRadians, 0);
  }

  public boolean nearSetpoint() {
    return Math.abs(getKinematicAngle() - targetAngle) < Units.degreesToRadians(8);
  }

  public boolean atSetpoint() {
    return SecondJointController.atGoal();
  }

  public void disable() {
    RightSecondJointMotor.set(0);
  }

  @Override
  public void periodic() {
    SecondJointController.setReference(convertAngleFromKinematicToSparkMax(targetAngle), 
    getKinematicAngle(), 
    (setpoint) -> secondJointFeedForward.calculate(setpoint.position, setpoint.velocity));
    
    SecondJointController.postData();

    SmartDashboard.putNumber("SecondJoint Encoder Position", SecondJointEncoder.getPosition());
    SmartDashboard.putNumber("SecondJoint Encoder Velocity", SecondJointEncoder.getVelocity());
    SmartDashboard.putBoolean("SecondJoint nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("SecondJoint Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
