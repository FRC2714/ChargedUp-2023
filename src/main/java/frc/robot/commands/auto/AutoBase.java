// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoBase extends SequentialCommandGroup {
  private DriveSubsystem m_robotDrive;

  /** Creates a new AutoBase. */
  public AutoBase(DriveSubsystem m_DriveSubsystem) {
    m_robotDrive = m_DriveSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }

  public PPSwerveControllerCommand CustomPathControllerCommand(PathPlannerTrajectory trajectory) {
    return new PPSwerveControllerCommand(
        trajectory, 
        m_robotDrive::getPose, 
        DriveConstants.kDriveKinematics, 
        new PIDController(AutoConstants.kPXController, 0, 0), 
        new PIDController(AutoConstants.kPYController, 0, 0), 
        new PIDController(AutoConstants.kPThetaController, 0, 0), 
        m_robotDrive::setModuleStates, 
        m_robotDrive);
  }
}
