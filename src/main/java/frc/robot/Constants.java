// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ArmForwardKinematicPosition;

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

    public static final double kPXController = 1.05;
    public static final double kPYController = 1.2;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    
    public static final TrapezoidProfile.Constraints kAutoControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
    
    public static final HashMap<String, Command> EventMap = new HashMap<>();
  }

  public static final class PneumaticsConstants {
    public static final int kPneumaticHubCanId = 1;
    public static final double kCompressorMinPressure = 90;
    public static final double kCompressorMaxPressure = 120;
  }

  public static final class ClawConstants {
    public static final int kClawMotorCanId = 14;

    public static final int kClawSolenoidForwardChannel = 7;
    public static final int kClawSolenoidReverseChannel = 6;

    public static final int kClawMotorCurrentLimit = 20; //amps 

    public static final double kNominalVoltage = 10.5;

    public static final double kIntakeMotorSpeed = 1;
    public static final double kOuttakeMotorSpeed = -0.1;
    public static final double kShootMotorSpeed = 0.6;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorCanId = 13;
    public static final int kPivotMotorCanId = 15;

    public static final int kIntakeMotorCurrentLimit = 30;
    public static final int kPivotMotorCurrentLimit = 30;

    public static final double kNominalVoltage = 12.8;
    public static final double kIntakeMotorSpeed = 0.85;
    public static final double kOuttakeMotorSpeed = 0.7;
    public static final double kShootMotorSpeed = 1.0;
  }

  public static final class LEDConstants {
    public static final int kBlinkinPort = 0;
    public static final int kBlinkin2Port = 1;

    public static final double kPurpleWave = 0.29;
    public static final double kYellowWave = 0.09;

    public static final double kPurple = 0.91;
    public static final double kYellow = 0.69;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class FieldConstants {
    public static final double kGoalHeight = Units.inchesToMeters(24);
  }

  public static final class CameraConstants {
    public static final double kMountingAngle = 35; // deg
    public static double kCameraHeight = Units.inchesToMeters(9.14);
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

    public static final double kBaseJointKinematicOffset = 45; //difference from kinematic 0 to sparkmax 0 approx 45 deg
    public static final double kSecondJointKinematicOffset = 630; //difference from kinematic 0 to sparkmax 0 approx 160 deg

    public static final boolean kBaseJointInverted = true; //both base joint motor and encoder are inverted
    public static final boolean kSecondJointInverted = false; //econd joint motor and encoder are NOT inverted

    //Controller Constants
    public static final double kBaseJointMaxVelocity = 5000;
    public static final double kBaseJointMaxAcceleration = 4000;
    public static final double kBaseJointTolerance = 6;
    public static final double kBaseJointFF = 0.00007;
    public static final double kBaseJointP = 0.00000;
    public static final double kBaseJointI = 0;
    public static final double kBaseJointD = 0.0000;

    public static final double kSecondJointMaxVelocity = 5000;
    public static final double kSecondJointMaxAcceleration = 3400;
    public static final double kSecondJointTolerance = 6;
    public static final double kSecondJointFF = 0.00004;
    public static final double kSecondJointP = 0.00006;
    public static final double kSecondJointI = 0.0;
    public static final double kSecondJointD = 0.00015;

    //Current Limits
    public static final int kBaseJointMotorCurrentLimit = 40; //amps
    public static final int kSecondJointMotorCurrentLimit = 50; //amps

    //Back to back transition
    public static final ArmForwardKinematicPosition kBackToBackIntermediatePosition = 
      new ArmForwardKinematicPosition(90, -60);

    //Back to Transfer transition
    public static final ArmForwardKinematicPosition kBackToTransferIntermediatePosition = 
      new ArmForwardKinematicPosition(50,150);

    //Back to front transition

    //Back to stow transition
    
    //Transfer to back transition
    public static final ArmForwardKinematicPosition kTransferToBackIntermediatePosition = 
      new ArmForwardKinematicPosition(60, 150);
    public static final ArmForwardKinematicPosition kTransferToBackIntermediate2Position = 
      new ArmForwardKinematicPosition(60, 90);

    //Transfer to transfer transition
    public static final ArmForwardKinematicPosition kTransferToTransferIntermediatePosition = 
      new ArmForwardKinematicPosition(75, 130);

    //Transfer to front transition
    public static final ArmForwardKinematicPosition kTransferToFrontIntermediatePosition = 
      new ArmForwardKinematicPosition(50, 90);

    //Transfer to stow transition
    public static final ArmForwardKinematicPosition kTransferToStowIntermediatePosition = 
      new ArmForwardKinematicPosition(50, 150);
    public static final ArmForwardKinematicPosition kTransferToStowIntermediate2Position = 
      new ArmForwardKinematicPosition(50, 90);

    //Front to back transition

    //Front to transfer transition
    public static final ArmForwardKinematicPosition kFrontToTransferIntermediatePosition = 
      new ArmForwardKinematicPosition(50, 100);
    public static final ArmForwardKinematicPosition kFrontToTransferIntermediate2Position = 
      new ArmForwardKinematicPosition(50, 150);

    //Front to front transition

    //Front to stow transition

    //Stow to back transition

    //Stow to transfer transition
    public static final ArmForwardKinematicPosition kStowToTransferIntermediatePosition = 
      new ArmForwardKinematicPosition(50, 120);
    public static final ArmForwardKinematicPosition kStowToTransferIntermediate2Position = 
      new ArmForwardKinematicPosition(50, 150);

    //Stow to front transition

    //Stow position 
    public static final ArmForwardKinematicPosition kStowPosition = 
      new ArmForwardKinematicPosition(142, -98);

    //Transfer position
    public static final ArmForwardKinematicPosition kTransferConeIntakePosition = 
      new ArmForwardKinematicPosition(85,150);
    //Transfer position
    public static final ArmForwardKinematicPosition kTransferCubeIntakePosition = 
      new ArmForwardKinematicPosition(103,130);

    //Back Cone Score positions
    public static final ArmForwardKinematicPosition kBackConeL1Position =
      new ArmForwardKinematicPosition(130, -120);
    public static final ArmForwardKinematicPosition kBackConeL2Position = 
      new ArmForwardKinematicPosition(101, -90);
    public static final ArmForwardKinematicPosition kBackConeL3Position = 
      new ArmForwardKinematicPosition(54, -14);

    //Back Cube Score positions
    public static final ArmForwardKinematicPosition kBackCubeL1Position = 
      new ArmForwardKinematicPosition(130, -120);//130, 124
    public static final ArmForwardKinematicPosition kBackCubeL2Position = 
      new ArmForwardKinematicPosition(108, -112);
    public static final ArmForwardKinematicPosition kBackCubeL3Position = 
      new ArmForwardKinematicPosition(68, -45);

    //Front Cone Score positions
    public static final ArmForwardKinematicPosition kFrontConeL2Position = 
      new ArmForwardKinematicPosition(125, 42);

    //Front Cube Score positions
    public static final ArmForwardKinematicPosition kFrontCubeL2Position = 
      new ArmForwardKinematicPosition(110, 81);
    public static final ArmForwardKinematicPosition kFrontCubeL3Position = 
      new ArmForwardKinematicPosition(140, 15);
    

    //Arm Intake positions
    public static final ArmForwardKinematicPosition kBackIntakePosition = 
      new ArmForwardKinematicPosition(90, -83);
    public static final ArmForwardKinematicPosition kFrontIntakePosition = 
      new ArmForwardKinematicPosition(104, 68);
    
  }
}
