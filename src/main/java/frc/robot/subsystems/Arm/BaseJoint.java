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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.utils.controller.AsymmetricTrapezoidProfile;
import frc.utils.controller.ProfiledPositionController;
import frc.utils.controller.AsymmetricTrapezoidProfile.Constraints;
import frc.utils.controller.AsymmetricTrapezoidProfile.State;


public class BaseJoint extends SubsystemBase {
  private CANSparkMax RightBaseJointMotor;
  private CANSparkMax LeftBaseJointMotor;
  private AbsoluteEncoder BaseEncoder;

  private ProfiledPositionController BaseJointController;
  private SparkMaxPIDController sparkMaxPIDController;
  private Constraints BaseJointConstraints;

  private double targetAngle;
  private State targetState = new State();

  private ArmFeedforward basejointFeedForward = new ArmFeedforward(0, 0, 0);

  private SparkMaxPIDController BaseJointPID;
  
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

    BaseJointPID = RightBaseJointMotor.getPIDController();
    BaseJointPID.setPositionPIDWrappingEnabled(false);
    BaseJointPID.setFeedbackDevice(BaseEncoder);
    BaseJointPID.setFF(ArmConstants.kBaseJointFF, 0);
    BaseJointPID.setP(ArmConstants.kBaseJointP, 0);
    BaseJointPID.setI(ArmConstants.kBaseJointI, 0);
    BaseJointPID.setD(ArmConstants.kBaseJointD, 0);
    BaseJointPID.setSmartMotionMaxVelocity(ArmConstants.kBaseJointMaxVelocity, 0);
    BaseJointPID.setSmartMotionMaxAccel(ArmConstants.kBaseJointMaxAcceleration, 0);
    BaseJointPID.setSmartMotionAllowedClosedLoopError(ArmConstants.kBaseJointTolerance, 0);
  }

  private double convertAngleFromSparkMaxToKinematic(double sparkAngle) {
    double kinematicAngle = sparkAngle;

    kinematicAngle -= ArmConstants.kBaseJointKinematicOffset; //subtract kinematic offset
    kinematicAngle /= ArmConstants.kBaseJointGearRatio; //divide by gear ratio

    //convert 0,360 to -180,180
    //kinematicAngle -= (Math.PI);

    return kinematicAngle;
  }

  private double convertAngleFromKinematicToSparkMax(double kinematicAngle) {
    double sparkAngle = kinematicAngle;

    //convert -180,180 to 0,360
    //sparkAngle += (2*Math.PI);

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
    BaseJointPID.setReference(convertAngleFromKinematicToSparkMax(targetAngle), CANSparkMax.ControlType.kSmartMotion, 0);
    //targetState = new State(targetAngleRadians, 0);
  }

  public void setTargetState(State targetState) {
    this.targetState = targetState;
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
    // BaseJointController.setReference(
    //   targetAngle,
    //   getKinematicAngle(),
    //   (targetState) -> basejointFeedForward.calculate(targetState.position, targetState.velocity));
    
    // SmartDashboard.putNumber("targetState position", Units.radiansToDegrees(targetState.position));
    // SmartDashboard.putNumber("targetState velocity", targetState.velocity);

    SmartDashboard.putNumber("BaseJoint Encoder", BaseEncoder.getPosition());
    SmartDashboard.putBoolean("BaseJoint nearSetpoint", nearSetpoint());
    SmartDashboard.putNumber("BaseJoint Kinematic Angle", Units.radiansToDegrees(getKinematicAngle()));
  }
}
