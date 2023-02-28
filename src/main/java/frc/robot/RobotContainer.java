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
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.Autoalign;
import frc.robot.commands.ZeroHeading;
import frc.robot.commands.auto.NothingAuto;
import frc.robot.commands.auto.ComplexAuto;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;
import frc.robot.subsystems.Arm.ArmStateMachine.IntakeMode;
import frc.robot.subsystems.Arm.ArmStateMachine.ScoreLevel;

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
	
	private final ArmStateMachine m_armStatemachine = new ArmStateMachine(m_arm, m_leds);

	// The driver's controller
	XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
	XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

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
		m_armStatemachine.setTargetArmStateCommand(ArmState.TRANSFER).schedule();
		m_armStatemachine.setTargetScoreLevelCommand(ScoreLevel.THREE).schedule();
		m_armStatemachine.setCargoTypeCommand(CargoType.CONE).schedule();
		m_armStatemachine.setIntakeModeCommand(IntakeMode.FLOOR).schedule();
		m_limelight.setLEDCommand(false).schedule();
	}

	public void setAutoDefaultStates() {
		m_armStatemachine.setTargetArmStateCommand(ArmState.TRANSFER).schedule();
		m_armStatemachine.setTargetScoreLevelCommand(ScoreLevel.THREE).schedule();
		m_armStatemachine.setCargoTypeCommand(CargoType.CONE).schedule();
		m_armStatemachine.setIntakeModeCommand(IntakeMode.FLOOR).schedule();
		m_limelight.setLEDCommand(false).schedule();
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
		new JoystickButton(m_driverController, Button.kRightBumper.value)
			.whileTrue(new WaitCommand(0.4).deadlineWith(new ZeroHeading(m_robotDrive))
			.andThen(new Autoalign(m_robotDrive, m_limelight)));
		
		//autobalance on left Bumper
		new JoystickButton(m_driverController, Button.kLeftBumper.value)
			.whileTrue(new AutoBalance(m_robotDrive));

		//intake on right trigger while held 
		new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
			.whileTrue(m_intake.intakeCommand())
			.whileFalse(m_intake.stopCommand());
		//outtake on left trigger while held
		new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
			.whileTrue(m_intake.outtakeCommand())
			.whileFalse(m_intake.stopCommand());

		//toggle deploy on X
		new JoystickButton(m_driverController, Button.kX.value)
			.toggleOnTrue(Commands.startEnd(m_intake::deploy, m_intake::retract, m_intake));
		
		new JoystickButton(m_driverController, Button.kB.value)
			.toggleOnTrue(Commands.startEnd(m_intake::open, m_intake::close, m_intake));

		new JoystickButton(m_driverController, Button.kA.value)
			.toggleOnTrue(Commands.startEnd(m_claw::intakeOpen, m_claw::intakeClose, m_claw));

		//reset gyro on y
		new JoystickButton(m_driverController, Button.kY.value)
			.onTrue(Commands.runOnce(m_robotDrive::zeroHeading, m_robotDrive));


		/////////////////////////////OPERATOR CONTROLS/////////////////////////////////////////////////////////////

		//hold to score on right trigger
		new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.2)
			.onTrue(m_claw.score())
			.onFalse(m_claw.stopOpen());
		
		//hold to shoot on left trigger
		new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.2)
			.onTrue(m_claw.shootCommand())
			.onFalse(m_claw.stopOpen());

		
		//Raise arm on up
		new POVButton(m_operatorController, 0)
			.onTrue(new InstantCommand(() -> m_arm.raiseCurrentPosition(5)));
		// back on right
		new POVButton(m_operatorController, 90)
			.onTrue(m_armStatemachine.setTargetArmStateCommand(ArmState.BACK));
		// transfer on down
		new POVButton(m_operatorController, 180)
			.onTrue(m_armStatemachine.setTargetArmStateCommand(ArmState.TRANSFER));
		// front on left
		new POVButton(m_operatorController, 270)
			.onTrue(m_armStatemachine.setTargetArmStateCommand(ArmState.FRONT));

		//toggle claw intake on right bumper
		new JoystickButton(m_operatorController, Button.kRightBumper.value)
			.toggleOnTrue(Commands.startEnd(m_claw::intakeOpen, m_claw::intakeClose, m_claw));

		// toggle FLOOR or HP on start
		// new JoystickButton(m_operatorController, Button.kStart.value)
		// 	.toggleOnTrue(m_armStatemachine.setIntakeModeCommand(IntakeMode.FLOOR))
		// 	.toggleOnFalse(m_armStatemachine.setIntakeModeCommand(IntakeMode.HP));
		
		// nothing on back
		//new JoystickButton(m_operatorController, Button.kBack.value)

		// level 3 on Y
		new JoystickButton(m_operatorController, Button.kY.value)
			.onTrue(m_armStatemachine.setTargetScoreLevelCommand(ScoreLevel.THREE));
		// level 2 on B
		new JoystickButton(m_operatorController, Button.kB.value)
			.onTrue(m_armStatemachine.setTargetScoreLevelCommand(ScoreLevel.TWO));
		// intake on A
		new JoystickButton(m_operatorController, Button.kA.value)
			.onTrue(m_armStatemachine.setTargetScoreLevelCommand(ScoreLevel.INTAKE));
		
		// toggle CONE or CUBE mode on X
		new JoystickButton(m_operatorController, Button.kStart.value)
			.onTrue(m_armStatemachine.setCargoTypeCommand(CargoType.CONE).andThen(m_intake.closeCommand()));

		new JoystickButton(m_operatorController, Button.kBack.value)
			.onTrue(m_armStatemachine.setCargoTypeCommand(CargoType.CUBE).andThen(m_intake.openCommand()));
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
    return new ComplexAuto(m_robotDrive);
  }
}
