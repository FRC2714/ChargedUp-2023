// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class AlignToCone extends CommandBase {
  private DriveSubsystem m_robotDrive;
  private Limelight m_backLimelight;

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  private double kPXControllerCone = 0.7;
  private double kPYControllerCone = 1.1;

  private double kPThetaController = 1.5;

  private double xControllerGoalCone = 0.42;

  //private double thetaControllerkP

  /** Creates a new SmoothAlign. */
  public AlignToCone(DriveSubsystem m_robotDrive, Limelight m_backLimelight) {
    this.m_robotDrive = m_robotDrive;
    this.m_backLimelight = m_backLimelight;

    // Use addRequirements() here to declare subsystem dependencies.
    xController = new ProfiledPIDController(kPXControllerCone, 0, 0, new Constraints(0.7, 0.2));
    yController = new ProfiledPIDController(kPYControllerCone, 0, 0, new Constraints(3, 3));
    thetaController = new ProfiledPIDController(kPThetaController, 0, 0, new Constraints(5, 10));
    
    addRequirements(m_robotDrive);

    xController.setGoal(xControllerGoalCone);
    xController.setTolerance(0,0);

    yController.setGoal(0);
    yController.setTolerance(Units.degreesToRadians(0),0);

    thetaController.setGoal(0);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_backLimelight.setRetroPipeline();
    m_backLimelight.setLED(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_backLimelight.isTargetVisible()) {
      m_robotDrive.drive(
        xController.calculate(m_backLimelight.getDistanceToGoalMeters()), 
        yController.calculate(m_backLimelight.getXOffsetRadians()), 
        thetaController.calculate(m_robotDrive.getHeadingRadians()), 
        true,
        false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_backLimelight.setLED(false);
    m_robotDrive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
