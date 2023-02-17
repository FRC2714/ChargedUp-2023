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
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.NothingAuto;
import frc.robot.commands.auto.ComplexAuto;
import frc.robot.commands.auto.MarkerAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake();
  private final Claw m_claw = new Claw();

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
    
    m_arm.setDefaultCommand(
      m_arm.transfer());
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
      //swing out and score
      // new JoystickButton(m_driverController, Button.kY.value)
      //   .onTrue(m_arm.swingOutLevelTwo());
      DriverStation.silenceJoystickConnectionWarning(true);
      
      //level 3 on y
      new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(m_arm.scoreConeLevelThree());
      //swing out level 2 on b
      new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(m_arm.swingOutLevelTwo());
      //swing out on a
      new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(m_arm.swingOut());
      //transfer out on x
      new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(m_arm.transfer());
      
      //deploy and intake cone on right bumper
      new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(m_intake.intakeCone());
      //retract and stop on left bumper
      new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(m_intake.retractAndStop());
        
      //claw intake cone on 90
      new POVButton(m_driverController, 90)
        .whileTrue(m_claw.intakeCone());
      //claw intake cube on 270
      new POVButton(m_driverController, 270)
        .whileTrue(m_claw.intakeCube());
      //stop claw
      new POVButton(m_driverController, 180)
        .whileTrue(new InstantCommand(() -> m_claw.stop(), m_claw));

      //open on y
      new JoystickButton(m_operatorController, Button.kY.value)
        .whileTrue(new InstantCommand(() -> m_intake.open(), m_intake));
      //close on b
      new JoystickButton(m_operatorController, Button.kB.value)
        .whileTrue(new InstantCommand(() -> m_intake.close(), m_intake));
      //retract on x
      new JoystickButton(m_operatorController, Button.kX.value)
        .whileTrue(new InstantCommand(() -> m_intake.retract(), m_intake));
      //deploy on a
      new JoystickButton(m_operatorController, Button.kA.value)
        .whileTrue(new InstantCommand(() -> m_intake.deploy(), m_intake));
      //intake on right bumper
      new JoystickButton(m_operatorController, Button.kRightBumper.value)
        .whileTrue(new InstantCommand(() -> m_intake.intake(), m_intake));
        //stop on left bumper
      new JoystickButton(m_operatorController, Button.kLeftBumper.value)
      .whileTrue(new InstantCommand(() -> m_intake.stop(), m_intake));

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
      exampleTrajectory, //exampleTrajectory
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

  public Command ComplexAuto() {
    return new ComplexAuto(m_robotDrive);
  }

  public Command MarkerAuto() {
    return new MarkerAuto(m_robotDrive, m_intake);
  }
}
