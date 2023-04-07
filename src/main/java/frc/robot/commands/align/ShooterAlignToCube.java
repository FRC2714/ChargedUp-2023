// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;

public class ShooterAlignToCube extends CommandBase {
  private DriveSubsystem m_robotDrive;
  private Limelight m_frontLimelight;

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  private double kPXControllerCube = 0.65;
  private double kPYControllerCube = 1.1;

  private double kPThetaController = 1;

  private double xControllerGoalCube = 0.80;

  //private double thetaControllerkP

  /** Creates a new SmoothAlign. */
  public ShooterAlignToCube(DriveSubsystem m_robotDrive, Limelight m_frontLimelight) {
    this.m_robotDrive = m_robotDrive;
    this.m_frontLimelight = m_frontLimelight;

    // Use addRequirements() here to declare subsystem dependencies.
    xController = new ProfiledPIDController(kPXControllerCube, 0, 0, AutoConstants.kAutoControllerConstraints);
    yController = new ProfiledPIDController(kPYControllerCube, 0, 0, AutoConstants.kAutoControllerConstraints);
    thetaController = new ProfiledPIDController(kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    addRequirements(m_robotDrive);

    xController.setGoal(xControllerGoalCube);
    xController.setTolerance(0,0);

    yController.setGoal(0);
    yController.setTolerance(Units.degreesToRadians(0),0);

    thetaController.setGoal(Units.degreesToRadians(180));
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_frontLimelight.setAprilTagPipeline();
    m_frontLimelight.setLED(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.drive(
        xController.calculate(m_frontLimelight.getDistanceToGoalMeters()), 
        yController.calculate(m_frontLimelight.getXOffsetRadians()), 
        thetaController.calculate(m_robotDrive.getHeadingRadians()), 
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_frontLimelight.setLED(false);
    m_robotDrive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
