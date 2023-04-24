// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.auto.NothingAuto;
import frc.robot.commands.auto.PathTestAuto;
import frc.robot.commands.auto.Cable.CubeThreeCargoCableAuto;
import frc.robot.commands.auto.Cable.TwoCargoCableAuto;
import frc.robot.commands.auto.Center.OneConeBalanceCenterAuto;
import frc.robot.commands.auto.Center.OneConeBalanceMobilityCenterAuto;
import frc.robot.commands.auto.Center.TwoCargoBalanceCenterAuto;
import frc.robot.commands.auto.Center.TwoCargoBalanceCenterMirrorAuto;
import frc.robot.commands.auto.Open.ThreeCargoOpenAuto;
import frc.robot.commands.auto.Open.TwoCargoBalanceOpenAuto;
import frc.robot.commands.auto.Open.TwoCargoOpenAuto;
import frc.robot.subsystems.Infrastructure;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.Drive.DriveSubsystem;
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
	private final DriveSubsystem m_drivetrain = new DriveSubsystem();
	private final Limelight m_limelight = new Limelight();
	private final Arm m_arm = new Arm();
	private final Claw m_claw = new Claw();
	private final LED m_led = new LED();
	private final Shooter m_shooter = new Shooter();
	
	private final Superstructure m_superstructure = new Superstructure(m_drivetrain, m_arm, m_claw, m_shooter, m_limelight, m_led);

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
		m_drivetrain.setDefaultCommand(
			// The left stick controls translation of the robot.
			// Turning is controlled by the X axis of the right stick.
			new RunCommand(
				() -> m_drivetrain.drive(
					-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
					-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
					-0.8*MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
					true, false),
				m_drivetrain));

		m_infrastructure.enableCompressor();
	}

	public void setTeleopDefaultStates() {
		System.out.println("setTeleopDefaultStates()");
		new SequentialCommandGroup(
			m_superstructure.setCargoTypeCommand(CargoType.CONE),
			m_superstructure.setSubsystemState(DPAD.DOWN),
			m_limelight.setLEDCommand(false),
			new InstantCommand(() -> m_claw.setClawClose()),
			new InstantCommand(() -> m_drivetrain.resetModules())
		).schedule();
		}

	public void setAutoDefaultStates() {
		Commands.runOnce(m_drivetrain::zeroHeading, m_drivetrain).schedule();
		m_limelight.setLEDCommand(false).schedule();
	}

	public void updateTelemetry() {
		m_superstructure.periodic();
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
			
		//autoalign on right bumper
		m_driverController.rightBumper()
			.whileTrue(m_superstructure.getAlign());
		
		//press to score on left bumper
		m_driverController.leftBumper()
			.onTrue(m_superstructure.ScoreCommand())
			.onFalse(m_shooter.stopCommand());

		//intake on right trigger while held 
		new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.25)
			.onTrue(m_superstructure.shooterIntakeSequence()) //intake sequence
			.onFalse(m_superstructure.setSubsystemState(DPAD.UP)); //shooter to hold

		//outtake on left trigger while held
		new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.25)
			.onTrue(m_superstructure.shooterOuttakeSequence())
			.onFalse(m_superstructure.setSubsystemState(DPAD.UP).alongWith(m_shooter.stopCommand()));

		//release cube on y
		m_driverController.y()
			.onTrue(m_shooter.setKickerOuttakeCommand(ShooterConstants.kKickSpeed))
			.onFalse(m_shooter.stopCommand());

		//claw cone intake on X
		m_driverController.x()
			.onTrue(m_claw.intakeCone());

		//reset gyro on back
		m_driverController.back()
			.onTrue(Commands.runOnce(m_drivetrain::zeroHeading, m_drivetrain));

		//lock wheels on right
		m_driverController.povRight()
			.whileTrue(new RunCommand(
            	() -> m_drivetrain.setX(),
            	m_drivetrain));

		//turn to 180 on down
		m_driverController.povDown() 
			.whileTrue(new TurnToAngle(m_drivetrain, 180));

		/////////////////////////////OPERATOR CONTROLS/////////////////////////////////////////////////////////////

		//set arm mode on start
		m_operatorController.start()
			.onTrue(m_superstructure.setScoreModeCommand(ScoreMode.ARM));

		//set shooter mode on back
		m_operatorController.back()
			.onTrue(m_superstructure.setScoreModeCommand(ScoreMode.SHOOTER));

		//stow/hold on up
		m_operatorController.povUp()
			.onTrue(m_superstructure.setSubsystemState(DPAD.UP));
		//transfer/retract on down
		m_operatorController.povDown()
			.onTrue(m_superstructure.setSubsystemState(DPAD.DOWN));
		//front/manual on left
		m_operatorController.povLeft()
			.onTrue(m_superstructure.setSubsystemState(DPAD.LEFT));
		//back/dynamic on right
		m_operatorController.povRight()
			.onTrue(m_superstructure.setSubsystemState(DPAD.RIGHT));

		//HIGH on Y
		m_operatorController.y()
			.onTrue(m_superstructure.setScoreLevelCommand(BUTTON.Y));
		//MIDDLE on B
		m_operatorController.b()
			.onTrue(m_superstructure.setScoreLevelCommand(BUTTON.B));
		//INTAKE on A
		m_operatorController.a()
			.onTrue(m_superstructure.setScoreLevelCommand(BUTTON.A));
		//LOW on X
		m_operatorController.x()
			.onTrue(m_superstructure.setScoreLevelCommand(BUTTON.X));
		
		// cone mode on right bumper
		m_operatorController.rightBumper()
			.onTrue(m_superstructure.setCargoTypeCommand(CargoType.CONE));

		// cube mode on left bumper
		m_operatorController.leftBumper()
			.onTrue(m_superstructure.setCargoTypeCommand(CargoType.CUBE));
	}

	public Command getNothingAuto() {
		return new NothingAuto();
	}

	public Command getPathTestAuto() {
		return new PathTestAuto(m_drivetrain);
	}

	public Command getOneConeBalanceCenterAuto() {
		return new OneConeBalanceCenterAuto(m_drivetrain, m_superstructure);
	}

	public Command getOneConeBalanceMobilityCenterAuto() {
		return new OneConeBalanceMobilityCenterAuto(m_drivetrain, m_superstructure);
	}

	public Command getTwoCargoBalanceCenterAuto() {
		return new TwoCargoBalanceCenterAuto(m_drivetrain, m_superstructure, m_shooter);
	}

	public Command getTwoCargoBalanceCenterMirrorAuto() {
		return new TwoCargoBalanceCenterMirrorAuto(m_drivetrain, m_superstructure, m_shooter);
	}

	public Command getTwoCargoOpenAuto() {
		return new TwoCargoOpenAuto(m_drivetrain, m_superstructure, m_shooter);
	}

	public Command getTwoCargoBalanceOpenAuto() {
		return new TwoCargoBalanceOpenAuto(m_drivetrain, m_superstructure, m_shooter);
	}

	public Command getThreeCargoOpenAuto() {
		return new ThreeCargoOpenAuto(m_drivetrain, m_superstructure, m_shooter);
	}

	public Command getTwoCargoCableAuto() {
		return new TwoCargoCableAuto(m_drivetrain, m_superstructure, m_shooter);
	}

	public Command getThreeCargoCableAuto() {
		return new CubeThreeCargoCableAuto(m_drivetrain, m_superstructure, m_shooter);
	}
}
