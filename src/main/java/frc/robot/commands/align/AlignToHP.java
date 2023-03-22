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

public class AlignToHP extends CommandBase {
  private DriveSubsystem m_robotDrive;
  private Limelight m_limelight;

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  private double kPThetaController = 1;

  private double xOffsetMeters = 0.50;
  private double yOffsetDegrees = Units.degreesToRadians(-22);

  //TODO point of interest tracking
  
  /** Creates a new SmoothAlign. */
  public AlignToHP(DriveSubsystem m_robotDrive, Limelight m_limelight) {
    this.m_robotDrive = m_robotDrive;
    this.m_limelight = m_limelight;

    // Use addRequirements() here to declare subsystem dependencies.
    xController = new ProfiledPIDController(0.8, 0, 0, AutoConstants.kAutoControllerConstraints);
    yController = new ProfiledPIDController(0.7, 0, 0, AutoConstants.kAutoControllerConstraints);
    thetaController = new ProfiledPIDController(kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    addRequirements(m_robotDrive);

    xController.setGoal(xOffsetMeters);
    xController.setTolerance(0.03,0);

    yController.setGoal(yOffsetDegrees);
    yController.setTolerance(Units.degreesToRadians(0),0);

    thetaController.setGoal(Units.degreesToRadians(180));
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setAprilTagPipeline();
    m_limelight.setLED(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.drive(
        -xController.calculate(m_limelight.getDistanceToGoalMeters()), 
        -yController.calculate(m_limelight.getXOffsetRadians()), 
        thetaController.calculate(m_robotDrive.getHeadingRadians()), 
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setLED(false);
    m_robotDrive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
