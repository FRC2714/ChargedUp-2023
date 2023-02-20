// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 8;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ClawConstants {
    public static final int kClawMotorCanId = 14;

    public static final int kClawSolenoidForwardChannel = 15;
    public static final int kClawSolenoidReverseChannel = 14;

    public static final int kClawMotorCurrentLimit = 20; //amps 
  }

  public static final class IntakeConstants {
    public static final int kTopMotorCanId = 13;
    public static final int kBottomMotorCanId = 15;

    public static final int kPneumaticHubCanId = 1;
    public static final double kCompressorMinPressure = 90;
    public static final double kCompressorMaxPressure = 120;

    public static final int kLeftRetractionSolenoidForwardChannel = 10;
    public static final int kLeftRetractionSolenoidReverseChannel = 11;

    public static final int kRightRetractionSolenoidForwardChannel = 13;
    public static final int kRightRetractionSolenoidReverseChannel = 12;

    public static final int kIntakeSolenoidForwardChannel = 8;
    public static final int kIntakeSolenoidReverseChannel = 9;

    public static final int kTopMotorCurrentLimit = 30;
    public static final int kBottomMotorCurrentLimit = 30;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ArmConstants {
    //Spark IDs
    public static final int kLeftBaseJointMotorCanId = 9;
    public static final int kRightBaseJointMotorCanId = 10;
    public static final int kSecondJointMotorCanId = 11;

    //Physical constants
    public static final double kBaseJointGearRatio = 240;
    public static final double kSecondJointGearRatio = 225;

    public static final double kBaseJointLength = Units.inchesToMeters(28);
    public static final double kSecondJointLength = Units.inchesToMeters(25);

    //Encoder Conversion
    public static final double kBaseJointPositionConversionFactor = (2*Math.PI) * kBaseJointGearRatio; //Radians * Gear ratio
    public static final double kSecondJointPositionConversionFactor = (2*Math.PI) * kSecondJointGearRatio;

    public static final double kBaseJointKinematicOffset = 200; //difference from kinematic 0 to sparkmax 0 approx 45 deg
    public static final double kSecondJointKinematicOffset = 677; //difference from kinematic 0 to sparkmax 0 approx 160 deg

    public static final boolean kBaseJointInverted = true; //both base joint motor and encoder are inverted
    public static final boolean kSecondJointInverted = false; //econd joint motor and encoder are NOT inverted

    //Controller Constants
    public static final double kBaseJointMaxVelocity = 4000; //todo tune this
    public static final double kBaseJointMaxAcceleration = 3000;
    public static final double kBaseJointTolerance = 20;
    public static final double kBaseJointFF = 0; //0.025
    public static final double kBaseJointP = 0.00007;
    public static final double kBaseJointI = 0;
    public static final double kBaseJointD = 0.01; //todo tune this

    public static final double kSecondJointMaxVelocity = 5000;
    public static final double kSecondJointMaxAcceleration = 3000;
    public static final double kSecondJointTolerance = 15;
    public static final double kSecondJointFF = 0; //0.025
    public static final double kSecondJointP = 0.00013; //0.00013 works but violent bounce
    public static final double kSecondJointI = 0;
    public static final double kSecondJointD = 0.03;

    //Current Limits
    public static final int kBaseJointMotorCurrentLimit = 50; //amps
    public static final int kSecondJointMotorCurrentLimit = 40; //amps
  }
}
