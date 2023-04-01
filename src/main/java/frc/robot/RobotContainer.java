// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.management.InstanceNotFoundException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.align.AlignToNode;
import frc.robot.commands.auto.NothingAuto;
import frc.robot.commands.auto.PathTestAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Infrastructure;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.BUTTON;
import frc.robot.subsystems.Superstructure.CargoType;
import frc.robot.subsystems.Superstructure.DPAD;
import frc.robot.subsystems.Superstructure.ScoreMode;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
	// The robot's subsystems
	private final Infrastructure m_infrastructure = new Infrastructure();
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final Limelight m_backLimelight = new Limelight(
		"limelight-back", 
		CameraConstants.kBackLimelightHeight, 
		CameraConstants.kBackLimelightMountingAngle);
	private final Limelight m_frontLimelight = new Limelight(
		"limelight-front", 
		CameraConstants.kFrontLimelightHeight, 
		CameraConstants.kFrontLimelightMountingAngle);
	private final Arm m_arm = new Arm();
	private final Shooter m_shooter = new Shooter(m_frontLimelight);
	private final Claw m_claw = new Claw();
	
	private final Superstructure m_superstructure = new Superstructure(m_arm, m_claw, m_shooter);

	// The driver's controller
	CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
	CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */

	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		m_robotDrive.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			new RunCommand(
				() -> m_robotDrive.drive(
					-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
					true, false),
				m_robotDrive));
	}

	public void setTeleopDefaultStates() {
		System.out.println("setTeleopDefaultStates()");
		new SequentialCommandGroup(
			m_superstructure.setCargoTypeCommand(CargoType.CONE),
			m_superstructure.setSubsystemState(DPAD.DOWN),
			m_backLimelight.setLEDCommand(false),
			m_frontLimelight.setLEDCommand(false),
			new InstantCommand(() -> m_claw.open())
		).schedule();
		}

	public void setAutoDefaultStates() {
		new InstantCommand(() -> m_robotDrive.zeroHeading()).schedule();
		m_backLimelight.setLEDCommand(false).schedule();
		//m_intake.pivotToHold().schedule();
	}

	public void updateTelemetry() {
		m_superstructure.updateTelemetry();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */

	private void configureButtonBindings() {
		DriverStation.silenceJoystickConnectionWarning(true);

		/////////////////////////////DRIVER CONTROLS/////////////////////////////////////////////////////////////
			
		//zero heading then autoalign on right bumper
		m_driverController.rightBumper()
			.whileTrue(new AlignToNode(m_robotDrive, m_backLimelight, m_superstructure));
		
		//hold to score on left bumper
		m_driverController.leftBumper()
			.onTrue(m_superstructure.ScoreCommand());

		//intake on right trigger while held 
		new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.25)
			.onTrue(m_superstructure.intakeRightTrigger()) //intake sequence
			.onFalse(m_superstructure.setSubsystemState(DPAD.UP)); //shooter to hold

		//outtake on left trigger while held
		new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.25)
			.whileTrue(m_superstructure.outtakeLeftTrigger())
			.whileFalse(m_superstructure.setSubsystemState(DPAD.UP).alongWith(m_shooter.stopCommand()));

		//release cube on y
		m_driverController.y()
			.onTrue(m_shooter.kick());

		// m_driverController.b()
		// 	.onTrue(new InstantCommand(() -> m_shooter.setTunable()));
		m_driverController.b()
			.onTrue(m_shooter.shootSequnce());

		//toggle claw intake on X
		m_driverController.x()
			.onTrue(m_claw.intakeAndToggleCommand());

		//reset gyro on left
		m_driverController.back()
			.onTrue(Commands.runOnce(m_robotDrive::zeroHeading, m_robotDrive));

		//align to hp on up
		m_driverController.povRight()
			.whileTrue(new InstantCommand(() -> m_robotDrive.setX()));

		/////////////////////////////OPERATOR CONTROLS/////////////////////////////////////////////////////////////

		//set arm on start
		m_operatorController.start()
			.onTrue(m_superstructure.setScoreModeCommand(ScoreMode.ARM));

		//set shooter on back
		m_operatorController.back()
			.onTrue(m_superstructure.setScoreModeCommand(ScoreMode.SHOOTER));

		// TUCK on up
		m_operatorController.povUp()
			.onTrue(m_superstructure.setSubsystemState(DPAD.UP));
		// TRANSFER on down
		m_operatorController.povDown()
			.onTrue(m_superstructure.setSubsystemState(DPAD.DOWN));
		// FRONT on left
		m_operatorController.povLeft()
		.onTrue(m_superstructure.setSubsystemState(DPAD.LEFT));
		// BACK on right
		m_operatorController.povRight()
			.onTrue(m_superstructure.setSubsystemState(DPAD.RIGHT));

		// level 3 on Y
		m_operatorController.y()
			.onTrue(m_superstructure.setScoreLevelCommand(BUTTON.Y));
		// level 2 on B
		m_operatorController.b()
			.onTrue(m_superstructure.setScoreLevelCommand(BUTTON.B));
		// intake on A
		m_operatorController.a()
			.onTrue(m_superstructure.setScoreLevelCommand(BUTTON.A));

		//toggle claw intake on X
		//m_operatorController.x()
			//.onTrue(new InstantCommand(() -> m_arm.setTargetPosition(30.0, 28.0)));
		
		// cone mode on right bumper
		m_operatorController.rightBumper()
			.onTrue(m_superstructure.setCargoTypeCommand(CargoType.CONE));

		// cube mode on left bumper
		m_operatorController.leftBumper()
			.onTrue(m_superstructure.setCargoTypeCommand(CargoType.CUBE));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getTestAuto() {
		// Create config for trajectory
		TrajectoryConfig config = new TrajectoryConfig(
				AutoConstants.kMaxSpeedMetersPerSecond,
				AutoConstants.kMaxAccelerationMetersPerSecondSquared)
			// Add kinematics to ensure max speed is actually obeyed
			.setKinematics(DriveConstants.kDriveKinematics);

		// An example trajectory to follow. All units in meters.
		Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
			// Start at the origin facing the +X direction
			new Pose2d(0, 0, new Rotation2d(0)),
			// Pass through these two interior waypoints, making an 's' curve path
			List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
			// End 3 meters straight ahead of where we started, facing forward
			new Pose2d(3, 0, new Rotation2d(0)),
			config);

		var thetaController = new ProfiledPIDController(
			AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
			exampleTrajectory,
			m_robotDrive::getPose, // Functional interface to feed supplier
			DriveConstants.kDriveKinematics,
			// Position controllers
			new PIDController(AutoConstants.kPXController, 0, 0),
			new PIDController(AutoConstants.kPYController, 0, 0),
			thetaController,
			m_robotDrive::setModuleStates,
			m_robotDrive);

		// Reset odometry to the starting pose of the trajectory.
		m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
	}

	public Command getNothingAuto() {
		return new NothingAuto();
	}

	public Command getComplexAuto() {
		return new PathTestAuto(m_robotDrive);
	}

	// public Command getOneCubeBalanceMiddleAuto() {
	// 	return new OneCubeBalanceMiddleAuto(m_robotDrive, m_armStateMachine, m_shooter, m_arm, m_claw, m_backLimelight);
	// }

	// public Command getOneConeBalanceMiddleAuto() {
	// 	return new OneConeBalanceMiddleAuto(m_robotDrive, m_armStateMachine, m_shooter, m_arm, m_claw, m_backLimelight);
	// }

	// public Command getTwoCargoOpenAuto() {
	// 	return new TwoCargoOpenAuto(m_robotDrive, m_armStateMachine, m_shooter, m_arm, m_claw, m_backLimelight);
	// }
}
