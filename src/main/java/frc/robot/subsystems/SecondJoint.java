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
import frc.robot.Constants.ArmConstants;

public class SecondJoint extends ProfiledPIDSubsystem {
  private CANSparkMax SecondJointMotor;

  private AbsoluteEncoder SecondJointEncoder;
  //ticks per rev: 8192
  //need to convert
  private double conversionFactor = 1/8192;
  /** Creates a new BaseJoint. */
  public SecondJoint() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ArmConstants.kSecondJointP,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmConstants.kSecondJointMaxVelocity, ArmConstants.kSecondJointMaxAcceleration))
    );
    SecondJointMotor = new CANSparkMax(ArmConstants.kSecondJointMotorCanId, CANSparkMaxLowLevel.MotorType.kBrushless);
    SecondJointEncoder = SecondJointMotor.getAbsoluteEncoder(Type.kDutyCycle);

  }

  public double getAngle() {
    return SecondJointEncoder.getPosition() * conversionFactor;
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    //baseMotor1.setVoltage(output);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Base Joint Encoder", SecondJointEncoder.getPosition());
    SmartDashboard.putNumber("Base Joint Angle", getAngle());
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return getAngle();
  }
}
