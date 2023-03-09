// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private CANSparkMax WristMotor;

  private AbsoluteEncoder WristEncoder;
  private SparkMaxPIDController WristPID;

  private double targetAngle;
  private final double WristGearRatio = 60;

  /** Creates a new Claw. */
  public Wrist() {
    WristMotor = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);

    WristEncoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    WristEncoder.setPositionConversionFactor(2*Math.PI*WristGearRatio);
    WristEncoder.setInverted(false);

    WristPID = WristMotor.getPIDController();
    WristPID.setFeedbackDevice(WristEncoder);

    WristPID.setFF(0, 0);
    WristPID.setP(0, 0);
    WristPID.setI(0, 0);
    WristPID.setD(0, 0);
    WristPID.setSmartMotionMaxVelocity(0, 0);
    WristPID.setSmartMotionMaxAccel(0, 0);
    WristPID.setSmartMotionAllowedClosedLoopError(0, 0);

    WristPID.setPositionPIDWrappingEnabled(true);
    WristPID.setPositionPIDWrappingMinInput(0);
    WristPID.setPositionPIDWrappingMaxInput(2*Math.PI);
  }

  private double convertAngleFromRadiansToSparkMax(double radians) {
    return radians * WristGearRatio;
  }

  private double convertAngleFromSparkMaxToRadians(double sparkAngle) {
    return sparkAngle / WristGearRatio;
  }

  public double getCurrentAngle() {
    return convertAngleFromSparkMaxToRadians(WristEncoder.getPosition());
  }

  public void setTargetAngle(double targetRadians) {
    this.targetAngle = targetAngle;
    WristPID.setReference(convertAngleFromRadiansToSparkMax(targetRadians), ControlType.kSmartMotion, 0);
  }

  public boolean atSetpoint() {
    return Math.abs(getCurrentAngle() - targetAngle) < Units.degreesToRadians(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("current wrist angle", Units.radiansToDegrees(getCurrentAngle()));
    SmartDashboard.putBoolean("wrist at setpoint", atSetpoint());
  }
}
