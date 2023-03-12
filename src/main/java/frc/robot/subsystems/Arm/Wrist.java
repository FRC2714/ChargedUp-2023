// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnWristToAngle;

public class Wrist extends SubsystemBase {
  private CANSparkMax WristMotor;

  private AbsoluteEncoder WristEncoder;

  private double targetAngle = 0;
  

  /** Creates a new Claw. */
  public Wrist() {
    WristMotor = new CANSparkMax(WristConstants.kWristMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    WristMotor.setInverted(true);
    WristMotor.enableVoltageCompensation(12.0);

    WristMotor.burnFlash();

    WristEncoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    WristEncoder.setPositionConversionFactor(WristConstants.kWristPositionConversionFactor);
    WristEncoder.setInverted(WristConstants.kWristInverted);
  }

  public void setPower(double power) {
    WristMotor.setVoltage(power*12.0);
  }

  public double getAngleRadians() {
    return WristEncoder.getPosition() / WristConstants.kWristGearRatio;
  }

  public double getAngleDegrees() {
    return Units.radiansToDegrees(getAngleRadians());
  }

  public boolean atSetpoint() {
    return Math.abs(getAngleRadians() - targetAngle) < Units.degreesToRadians(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("wrist target angle", Units.radiansToDegrees(targetAngle));
    SmartDashboard.putNumber("current wrist angle", getAngleDegrees());

  }
}
