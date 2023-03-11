// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
  private CANSparkMax WristMotor;

  private AbsoluteEncoder WristEncoder;
  private PIDController WristController;

  private double targetAngle = 0;
  

  /** Creates a new Claw. */
  public Wrist() {
    WristMotor = new CANSparkMax(WristConstants.kWristMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    WristMotor.setInverted(true);
    WristMotor.enableVoltageCompensation(12.0);

    WristEncoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    WristEncoder.setPositionConversionFactor(WristConstants.kWristPositionConversionFactor);
    WristEncoder.setInverted(WristConstants.kWristInverted);

    WristController = new PIDController(1, 0, 0.01);

    WristController.enableContinuousInput(0, 2*Math.PI);
    WristController.setTolerance(Units.degreesToRadians(1));
  }

  private void setControllerMotorOutput() {
    WristMotor.setVoltage(WristController.calculate(getCurrentAngleRadians())*12.0);
  }

  public double getCurrentAngleRadians() {
    return WristEncoder.getPosition() / WristConstants.kWristGearRatio;
  }

  public void setTargetAngle(double targetDegrees) {
    this.targetAngle = Units.degreesToRadians(targetDegrees);
    WristController.setSetpoint(Units.degreesToRadians(targetDegrees));
  }

  public Command setTargetAngleCommand(double targetDegrees) {
    return new InstantCommand(() -> setTargetAngle(targetDegrees));
  }

  public boolean atSetpoint() {
    return Math.abs(getCurrentAngleRadians() - targetAngle) < Units.degreesToRadians(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("wrist target angle", Units.radiansToDegrees(targetAngle));
    SmartDashboard.putNumber("current wrist angle", Units.radiansToDegrees(getCurrentAngleRadians()));
    SmartDashboard.putBoolean("wrist at setpoint", atSetpoint());
    setControllerMotorOutput();

  }
}
