// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public class BaseJoint extends ProfiledPIDSubsystem {
  private CANSparkMax baseMotor1;
  private CANSparkMax baseMotor2;

  private AbsoluteEncoder baseEncoder;

  private double conversionFactor = 0;
  /** Creates a new BaseJoint. */
  public BaseJoint() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0))
    );
    baseMotor1 = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    baseMotor2 = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
    baseEncoder = baseMotor1.getAbsoluteEncoder(Type.kDutyCycle);

    baseMotor2.follow(baseMotor1, false);

  }

  public double getAngle() {
    return baseEncoder.getPosition() * conversionFactor;
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    baseMotor1.setVoltage(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Base Joint Encoder", getAngle());
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }
}
