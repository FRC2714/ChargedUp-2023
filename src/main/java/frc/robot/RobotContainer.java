// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ScoreCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.align.SmoothAlign;
import frc.robot.commands.auto.NothingAuto;
import frc.robot.commands.auto.MIDDLE.OneConeBalanceMiddleAuto;
import frc.robot.commands.auto.MIDDLE.OneCubeBalanceMiddleAuto;
import frc.robot.commands.auto.OPEN.TwoCargoOpenAuto;
import frc.robot.commands.auto.OPEN.TwoCubeOpenAuto;
import frc.robot.commands.auto.OPEN.TwoCubeOpenAutoStop;
import frc.robot.commands.auto.TERRAIN.OneCubeTerrainAuto;
import frc.robot.commands.auto.PathTestAuto;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Arm.ArmStateMachine.ScoreLevel;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final Limelight m_limelight = new Limelight();
	private final Arm m_arm = new Arm();
	private final Intake m_intake = new Intake();
	private final Claw m_claw = new Claw();
	private final LEDs m_leds = new LEDs();
	
	private final ArmStateMachine m_armStateMachine = new ArmStateMachine(m_arm, m_leds, m_intake, m_claw);

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
					true, true),
				m_robotDrive));

	}

	public void setTeleopDefaultStates() {
		m_armStateMachine.setCargoTypeCommand(CargoType.CONE).schedule();
		m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.INTAKE).schedule();
		m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER).schedule();
		m_limelight.setLEDCommand(false).schedule();
		//m_intake.pivotToHold().schedule();
	}

	public void setAutoDefaultStates() {
		new InstantCommand(() -> m_robotDrive.zeroHeading());
		m_limelight.setLEDCommand(false).schedule();
		//m_intake.pivotToHold().schedule();
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
			.whileTrue(new SmoothAlign(m_robotDrive, m_limelight, m_armStateMachine));
		
		//hold to score on left bumper
		m_driverController.leftBumper()
			.onTrue(new ScoreCommand(m_armStateMachine, m_claw))
			.onFalse(m_claw.stopOpen());

		//intake on right trigger while held 
		new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.3)
			.onTrue(m_intake.deployAndIntake())
			.onFalse(m_intake.pivotToHold());

		//outtake on left trigger while held
		new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.3)
			.whileTrue(m_intake.pivotThenOuttake())
			.whileFalse(m_intake.holdAndStop());

		//shoot on b
		m_driverController.b()
			.onTrue(m_intake.pivotThenShoot())
			.onFalse(m_intake.holdAndStop());

		//turn to 180 on y
		m_driverController.y()
			.onTrue(new TurnToAngle(m_robotDrive, 180));

		//toggle claw intake on X
		m_driverController.x()
			.onTrue(Commands.runOnce(m_claw::intakeAndToggle, m_claw));

		//reset gyro on left
		m_driverController.back()
			.onTrue(Commands.runOnce(m_robotDrive::zeroHeading, m_robotDrive));

		//align to hp on up
		m_driverController.povUp()
			.whileTrue(new InstantCommand(() -> m_robotDrive.setX()));

		/////////////////////////////OPERATOR CONTROLS/////////////////////////////////////////////////////////////

		//manual raise arm on start
		// m_operatorController.start()

		// //manual lower arm on back
		// m_operatorController.back()

		// TUCK on up
		m_operatorController.povUp()
			.onTrue(m_armStateMachine.setTargetArmStateCommand(ArmState.STOW));
		// BACK on right
		m_operatorController.povRight()
			.onTrue(m_armStateMachine.setTargetArmStateCommand(ArmState.BACK));
		// TRANSFER on down
		m_operatorController.povDown()
			.onTrue(m_armStateMachine.setTargetArmStateCommand(ArmState.TRANSFER));
		// FRONT on left
		m_operatorController.povLeft()
			.onTrue(m_armStateMachine.setTargetArmStateCommand(ArmState.FRONT));

		// level 3 on Y
		m_operatorController.y()
			.onTrue(m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.THREE));
		// level 2 on B
		m_operatorController.b()
			.onTrue(m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.TWO));
		// intake on A
		m_operatorController.a()
			.onTrue(m_armStateMachine.setTargetScoreLevelCommand(ScoreLevel.INTAKE));

		//toggle claw intake on X
		// m_operatorController.x()
		// 	.toggleOnTrue(Commands.startEnd(m_claw::intakeOpen, m_claw::intakeClose, m_claw));
		
		// cone mode on right bumper
		m_operatorController.rightBumper()
			.onTrue(m_armStateMachine.setCargoTypeCommand(CargoType.CONE));

		// cube mode on left bumper
		m_operatorController.leftBumper()
			.onTrue(m_armStateMachine.setCargoTypeCommand(CargoType.CUBE));
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

	public Command getOneCubeBalanceMiddleAuto() {
		return new OneCubeBalanceMiddleAuto(m_robotDrive, m_armStateMachine, m_intake, m_arm, m_claw, m_limelight);
	}

	public Command getOneConeBalanceMiddleAuto() {
		return new OneConeBalanceMiddleAuto(m_robotDrive, m_armStateMachine, m_intake, m_arm, m_claw, m_limelight);
	}

	public Command getTwoCubeOpenAuto() {
		return new TwoCubeOpenAuto(m_robotDrive, m_armStateMachine, m_intake, m_arm, m_claw, m_limelight);
	}

	public Command getTwoCargoOpenAuto() {
		return new TwoCargoOpenAuto(m_robotDrive, m_armStateMachine, m_intake, m_arm, m_claw, m_limelight);
	}

	public Command getTwoCubeOpenAutoStop() {
		return new TwoCubeOpenAutoStop(m_robotDrive, m_armStateMachine, m_intake, m_arm, m_claw, m_limelight);
	}

	public Command getOneCubeTerrainAuto() {
		return new OneCubeTerrainAuto(m_robotDrive, m_armStateMachine, m_intake, m_arm, m_claw, m_limelight);
	}
}
