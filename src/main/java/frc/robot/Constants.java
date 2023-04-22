// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.PIDConstants;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ArmPreset;
import frc.robot.utils.ShooterPreset;
import frc.robot.utils.controller.AsymmetricTrapezoidProfile.Constraints;

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
    //x, y controller
    public static final double kTranslationControllerP = 3.0;
    public static final double kTranslationControllerD = 0.05;
    public static final PIDConstants kTranslationControllerConstants = 
      new PIDConstants(AutoConstants.kTranslationControllerP, 0.0, AutoConstants.kTranslationControllerD);

    //theta controller
    public static final double kThetaControllerP = 1.5;
    public static final double kThetaControllerD = 0.05;
    public static final PIDConstants kThetaControllerConstants = 
      new PIDConstants(AutoConstants.kThetaControllerP, 0.0, AutoConstants.kThetaControllerD);
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

    public static final int kClawMotorCurrentLimit = 12; //amps 

    public static final double kNominalVoltage = 11.5;

    public static final double kIntakeMotorSpeed = 1;
    public static final double kOuttakeMotorSpeed = -0.1;
    public static final double kShootMotorSpeed = -0.4;
  }

  public static final class ShooterConstants {
    //Kicker
    public static final int kKickerMotorCanId = 13;
    public static final int kKickerMotorCurrentLimit = 30;
    public static final double kKickerNominalVoltage = 12.0;
    public static final double kKickerIntakeMotorSpeed = 0.3;
    public static final double kKickerOuttakeMotorSpeed = -0.6;
    public static final double kKickerHoldMotorSpeed = 0.1;
    public static final double kKickSpeed = -0.4;

    //Pivot
    public static final int kPivotMotorCanId = 15;
    public static final double kPivotGearRatio = 50;
    public static final double kPivotPositionConversionFactor = (2*Math.PI) * kPivotGearRatio;
    public static final double kPivotEncoderZeroOffset = 220.0;
    public static final double kPivotKinematicOffset = 115;
    public static final int kPivotMotorCurrentLimit = 30;
    public static final double kPivotP = 3.0;
    public static final ArmFeedforward kPivotFeedforward = new ArmFeedforward(0, 0.49, 0.97, 0.01);
    
    //Flywheels
    public static final int kTopFlywheelMotorCanId = 16;
    public static final int kBottomFlywheelMotorCanId = 17;
    public static final double kFlywheelVelocityConversionFactor = (2*Math.PI) * 1.0;
    public static final int kTopFlywheelMotorCurrentLimit = 30;
    public static final int kBottomFlywheelMotorCurrentLimit = 30;

    //Preset Angles
    public static final double kPivotHoldAngleDegrees = -40;

    //Shooter Presets
    public static final ShooterPreset kIntakePreset = 
      new ShooterPreset(115, -100);
    public static final ShooterPreset kOuttakePreset = 
      new ShooterPreset(90, 50);

    public static final ShooterPreset kRetractPreset = 
      new ShooterPreset(-95, 0);
    public static final ShooterPreset kHoldPreset = 
      new ShooterPreset(kPivotHoldAngleDegrees, 0);

    public static final ShooterPreset kLaunchCubePreset = 
      new ShooterPreset(45, 200);

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

    public static final double kPurple = 0.91;
    public static final double kYellow = 0.69;
    public static final double kGreen = 0.77;
    public static final double kRed = 0.61;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class LimelightConstants {
    public static final Pose3d kBackLimelightPose = 
      new Pose3d(
        new Translation3d(15.75, 9.14, 0.0), //inches
        new Rotation3d(0.0, 35.0, 0.0));//degrees

    public static final double kMiddleRetroTapeHeight = 24.5; // inches
    public static final double kCubeLowHeight = -18; // inches
    public static final double kCubeMiddleHeight = 5.5; // inches
    public static final double kCubeHighHeight = 17; // inches
  }

  public static final class ShoulderConstants {
    public static final int kRightShoulderMotorCanId = 10;
    public static final int kLeftShoulderMotorCanId = 9;

    public static final double kShoulderGearRatio = 240.0;
    public static final double kShoulderLength = Units.inchesToMeters(28);
    public static final double kShoulderPositionConversionFactor = (2*Math.PI) * kShoulderGearRatio; //Radians * Gear ratio
    public static final double kShoulderEncoderZeroOffset = 623.8;
    public static final double kShoulderKinematicOffset = 105.0;
    public static final boolean kShoulderMotorInverted = true;
    public static final boolean kShoulderEncoderInverted = true;
    public static final double kShoulderP = 8.0;
    public static final int kShoulderMotorCurrentLimit = 40; //amps

    public static final Constraints kFarConstraints = new Constraints(16, 28, 24);
    public static final Constraints kCloseConstraints = new Constraints(30, 40, 30);

    public static final ArmFeedforward kShoulderFeedForward = new ArmFeedforward(0, 0.47, 4.68, 0.04);
  }

  public static final class ElbowConstants {
    public static final int kRightElbowMotorCanId = 11;

    public static final double kElbowGearRatio = 225.0;
    public static final double kElbowLength = Units.inchesToMeters(25);
    public static final double kElbowPositionConversionFactor = (2*Math.PI) * kElbowGearRatio;
    public static final double kElbowEncoderZeroOffset = 1422.0;
    public static final double kElbowKinematicOffset = 762.0;
    public static final boolean kElbowMotorInverted = false;
    public static final boolean kElbowEncoderInverted = true;
    public static final double kElbowP = 8.0;
    public static final int kElbowMotorCurrentLimit = 30; //amps

    public static final Constraints kFarConstraints = new Constraints(16, 28, 24);
    public static final Constraints kCloseConstraints = new Constraints(30, 40, 30);

    public static final ArmFeedforward kElbowFeedForward = new ArmFeedforward(0, 0.35, 4.38, 0.03);
  }

  public static final class ArmConstants {
    //Back to back transition
    public static final ArmPreset kBackToBackIntermediatePosition = 
      new ArmPreset(90, -60);

    //Back to Transfer transition
    public static final ArmPreset kBackToTransferIntermediatePosition = 
      new ArmPreset(50,150);

    //Back to front transition

    //Back to stow transition
    
    //Transfer to back transition
    public static final ArmPreset kTransferToBackIntermediatePosition = 
      new ArmPreset(60, 150);
    public static final ArmPreset kTransferToBackIntermediate2Position = 
      new ArmPreset(60, 90);

    //Transfer to transfer transition
    public static final ArmPreset kTransferToTransferIntermediatePosition = 
      new ArmPreset(75, 130);

    //Transfer to front transition
    public static final ArmPreset kTransferToFrontIntermediatePosition = 
      new ArmPreset(50, 90);

    //Transfer to stow transition
    public static final ArmPreset kTransferToStowIntermediatePosition = 
      new ArmPreset(50, 150);
    public static final ArmPreset kTransferToStowIntermediate2Position = 
      new ArmPreset(50, 90);

    //Front to back transition

    //Front to transfer transition
    public static final ArmPreset kFrontToTransferIntermediatePosition = 
      new ArmPreset(50, 100);
    public static final ArmPreset kFrontToTransferIntermediate2Position = 
      new ArmPreset(50, 150);

    //Front to front transition

    //Front to stow transition

    //Stow to back transition

    //Stow to transfer transition
    public static final ArmPreset kStowToTransferIntermediatePosition = 
      new ArmPreset(50, 120);
    public static final ArmPreset kStowToTransferIntermediate2Position = 
      new ArmPreset(50, 150);

    //Stow to front transition

    //Stow position 
    public static final ArmPreset kStowPosition = 
      new ArmPreset(126, -58); //123, -63

    //Transfer position
    public static final ArmPreset kTransferPosition = 
      new ArmPreset(82,135);

    //Back Cone Score positions
    public static final ArmPreset kBackConeLowPosition =
      new ArmPreset(130, -120);
    public static final ArmPreset kBackConeMiddlePosition = 
      new ArmPreset(101, -90);
    public static final ArmPreset kBackConeHighPosition = 
      new ArmPreset(57, -30);

    //Back Cube Score positions
    public static final ArmPreset kBackCubeLowPosition = 
      new ArmPreset(130, -120);//130, 124
    public static final ArmPreset kBackCubeMiddlePosition = 
      new ArmPreset(108, -112);
    public static final ArmPreset kBackCubeHighPosition = 
      new ArmPreset(68, -45);

    //Front Cone Score positions
    public static final ArmPreset kFrontConeMiddlePosition = 
      new ArmPreset(120, 42);

    //Front Cube Score positions
    public static final ArmPreset kFrontCubeMiddlePosition = 
      new ArmPreset(110, 81);
    public static final ArmPreset kFrontCubeHighPosition = 
      new ArmPreset(120, 15);

    //Arm Intake positions
    public static final ArmPreset kBackIntakePosition = 
      new ArmPreset(90, -86);
    public static final ArmPreset kFrontIntakePosition = 
      new ArmPreset(104, 68);
    
  }
}
