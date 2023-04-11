// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ArmForwardKinematicPosition;
import frc.robot.utils.ShooterPreset;

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
    public static final double kMaxSpeedMetersPerSecond = 5.2; //4.8
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5); // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5); // Distance between front and back wheels on robot
    
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
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.05;
    public static final double kPYController = 1.05;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    
    public static final TrapezoidProfile.Constraints kAutoControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
  }

  public static final class TuningConstants {
    public static final boolean tuningMode = true;
  }

  public static final class InfrastructureConstants {
    public static final int kPowerDistributionHubCanId = 1;
    
    public static final int kPneumaticHubCanId = 1;
    public static final double kCompressorMinPressure = 90; //psi
    public static final double kCompressorMaxPressure = 120; //psi
  }

  public static final class ClawConstants {
    public static final int kClawMotorCanId = 14;

    public static final int kClawSolenoidForwardChannel = 7;
    public static final int kClawSolenoidReverseChannel = 6;

    public static final int kClawMotorCurrentLimit = 10; //amps 

    public static final double kNominalVoltage = 11;

    public static final double kIntakeMotorSpeed = 1;
    public static final double kOuttakeMotorSpeed = -0.1;
    public static final double kShootMotorSpeed = -0.4;
  }

  public static final class ShooterConstants {
    //Spark IDs
    public static final int kKickerMotorCanId = 13;
    public static final int kPivotMotorCanId = 15;

    public static final int kTopFlywheelMotorCanId = 16;
    public static final int kBottomFlywheelMotorCanId = 17;

    public static final double kPivotGearRatio = 50;
    public static final double kPivotPositionConversionFactor = (2*Math.PI) * kPivotGearRatio;

    //Current Limits
    public static final int kKickerMotorCurrentLimit = 30;
    public static final int kPivotMotorCurrentLimit = 30;
    public static final int kTopFlywheelMotorCurrentLimit = 30;
    public static final int kBottomFlywheelMotorCurrentLimit = 30;

    public static final double kNominalVoltage = 12;

    //Kicker motor speeds
    public static final double kIntakeMotorSpeed = 0.3;
    public static final double kOuttakeMotorSpeed = -0.7;
    public static final double kKickSpeed = -0.4;

    //Preset Angles
    public static final double kPivotHoldAngleDegrees = -40;

    //Shooter Presets
    public static final ShooterPreset kIntakePreset = 
      new ShooterPreset(115, -100);
    public static final ShooterPreset kOuttakePreset = 
      new ShooterPreset(90, 100);

    public static final ShooterPreset kRetractPreset = 
      new ShooterPreset(-95, 0);
    public static final ShooterPreset kHoldPreset = 
      new ShooterPreset(kPivotHoldAngleDegrees, 0);

    public static final ShooterPreset kLaunchCubePreset = 
      new ShooterPreset(45, 300);

    public static final ShooterPreset kCloseHighCubePreset = 
      new ShooterPreset(15, 70);
    public static final ShooterPreset kFarHighCubePreset = 
      new ShooterPreset(45, 0);

    public static final ShooterPreset kCloseMiddleCubePreset = 
      new ShooterPreset(20, 40);
    public static final ShooterPreset kFarMiddleCubePreset = 
      new ShooterPreset(45, 0);

  }

  public static final class LEDConstants {
    public static final int kArmBlinkinPort = 0;
    public static final int kBaseBlinkinPort = 1;

    public static final double kPurpleWave = 0.29;
    public static final double kYellowWave = 0.09;

    public static final double kPurple = 0.91;
    public static final double kYellow = 0.69;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LimelightConstants {
    // 20 degrees, 31.5 inches from the ground
    public static final double kBackLimelightMountingAngle = 35.0; //degrees
    public static final double kBackLimelightHeight = 9.14; //inches
    public static final Pose2d kBackLimelightPose = 
      new Pose2d(
        new Translation2d(0, kBackLimelightHeight), 
        new Rotation2d(kBackLimelightMountingAngle));

    public static final double kFrontLimelightMountingAngle = -20; //degrees
    public static final double kFrontLimelightHeight = 31.5; //inches
    public static final Pose2d kFrontLimelightPose = 
      new Pose2d(
        new Translation2d(0, kFrontLimelightHeight), 
        new Rotation2d(kFrontLimelightMountingAngle));

    public static final double kMiddleRetroTapeHeight = 24.5; // inches TODO convert to translation 2d
    public static final double kCubeLowHeight = -18; // inches
    // public static final double kCubeMiddleHeight = 5.5; // inches
    // public static final double kCubeHighHeight = 17; // inches

    public static final double kCubeMiddleHeight = -18;
    public static final double kCubeHighHeight = -18;
  }

  public static final class ArmConstants {
    //Spark IDs
    public static final int kRightShoulderMotorCanId = 10;
    public static final int kLeftShoulderMotorCanId = 9;

    public static final int kRightElbowMotorCanId = 11;
    public static final int kLeftElbowMotorCanId = 12;

    //Physical constants
    public static final double kShoulderGearRatio = 240;
    public static final double kElbowGearRatio = 225;

    public static final double kShoulderLength = Units.inchesToMeters(28);
    public static final double kElbowLength = Units.inchesToMeters(25);

    //Encoder Conversion
    public static final double kShoulderPositionConversionFactor = (2*Math.PI) * kShoulderGearRatio; //Radians * Gear ratio
    public static final double kElbowPositionConversionFactor = (2*Math.PI) * kElbowGearRatio;

    public static final double kShoulderEncoderZeroOffset = 180.9295889;
    public static final double kElbowEncoderZeroOffset = 1422;
    public static final double kShoulderKinematicOffset = 42.6; //difference from kinematic 0 to sparkmax 0 approx 45 deg
    public static final double kElbowKinematicOffset = 762.0; //difference from kinematic 0 to sparkmax 0 approx 160 deg

    public static final boolean kShoulderMotorInverted = true; //base joint encoder inverted
    public static final boolean kShoulderEncoderInverted = true; //base joint motor inverted

    public static final boolean kElbowMotorInverted = false; //second joint encoder inverted
    public static final boolean kElbowEncoderInverted = true; //second joint motor and encoder are NOT inverted

    //Controller Constants
    public static final double kShoulderP = 8;

    public static final double kElbowP = 7;

    //Current Limits
    public static final int kShoulderMotorCurrentLimit = 40; //amps
    public static final int kElbowMotorCurrentLimit = 30; //amps

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
      new ArmForwardKinematicPosition(126, -58); //123, -63

    //Transfer position
    public static final ArmForwardKinematicPosition kTransferPosition = 
      new ArmForwardKinematicPosition(82,135);

    //Back Cone Score positions
    public static final ArmForwardKinematicPosition kBackConeLowPosition =
      new ArmForwardKinematicPosition(130, -120);
    public static final ArmForwardKinematicPosition kBackConeMiddlePosition = 
      new ArmForwardKinematicPosition(101, -90);
    public static final ArmForwardKinematicPosition kBackConeHighPosition = 
      new ArmForwardKinematicPosition(57, -30);

    //Back Cube Score positions
    public static final ArmForwardKinematicPosition kBackCubeLowPosition = 
      new ArmForwardKinematicPosition(130, -120);//130, 124
    public static final ArmForwardKinematicPosition kBackCubeMiddlePosition = 
      new ArmForwardKinematicPosition(108, -112);
    public static final ArmForwardKinematicPosition kBackCubeHighPosition = 
      new ArmForwardKinematicPosition(68, -45);

    //Front Cone Score positions
    public static final ArmForwardKinematicPosition kFrontConeMiddlePosition = 
      new ArmForwardKinematicPosition(120, 42);

    //Front Cube Score positions
    public static final ArmForwardKinematicPosition kFrontCubeMiddlePosition = 
      new ArmForwardKinematicPosition(110, 81);
    public static final ArmForwardKinematicPosition kFrontCubeHighPosition = 
      new ArmForwardKinematicPosition(120, 15);
    

    //Arm Intake positions
    public static final ArmForwardKinematicPosition kBackIntakePosition = 
      new ArmForwardKinematicPosition(90, -86);
    public static final ArmForwardKinematicPosition kFrontIntakePosition = 
      new ArmForwardKinematicPosition(104, 68);
    
  }
}
