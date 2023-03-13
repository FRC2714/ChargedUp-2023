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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.TurnWristToAngle;

public class Wrist extends SubsystemBase {
  private CANSparkMax WristMotor;

  private AbsoluteEncoder WristEncoder;

  private double targetAngle = 0;
  private double wristAngle;
  

  /** Creates a new Claw. */
  public Wrist() {
    WristMotor = new CANSparkMax(WristConstants.kWristMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    WristMotor.setInverted(true);
    WristMotor.enableVoltageCompensation(12.0);

    WristMotor.burnFlash();

    WristEncoder = WristMotor.getAbsoluteEncoder(Type.kDutyCycle);
    WristEncoder.setPositionConversionFactor(WristConstants.kWristPositionConversionFactor);
    WristEncoder.setInverted(WristConstants.kWristInverted);
    WristEncoder.setZeroOffset(225.4339894);
  }

  public void setPower(double power) {
    WristMotor.setVoltage(power*12.0);
  }

  public double getAngleRadians() {
    this.wristAngle = WristEncoder.getPosition() / WristConstants.kWristGearRatio;
    return wristAngle;
  }

  public double getAngleDegrees() {
    return Units.radiansToDegrees(getAngleRadians());
  }

  public boolean atSetpoint() {
    return Math.abs(getAngleRadians() - targetAngle) < Units.degreesToRadians(1);
  }

  public Command FlipWrist() {
    double finalTargetAngleDegrees;
    if ((getAngleDegrees() >= 0 && getAngleDegrees() < 90) || (getAngleDegrees() <= 360 && getAngleDegrees() > 270)) { //when wrist is near 0
      finalTargetAngleDegrees = 180;
    } else {
      finalTargetAngleDegrees = 0;
    }
    return new SequentialCommandGroup(
      new WaitCommand(0.2).raceWith(new TurnWristToAngle(this, 90)),
      new WaitCommand(1.5).raceWith(new TurnWristToAngle(this, finalTargetAngleDegrees))
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("wrist target angle", Units.radiansToDegrees(targetAngle));
    SmartDashboard.putNumber("Wrist Angle Degrees", getAngleDegrees());

  }
}
