// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PPTestAuto2 extends AutoBase {
  DriveSubsystem drivetrain;

  PathConstraints autoPathConstraints = 
    new PathConstraints(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  List<PathPlannerTrajectory> autoPathGroup = 
    PathPlanner.loadPathGroup(
      "SCurve",
      autoPathConstraints);
  
  PathPlannerTrajectory firstTrajectory = PathPlanner.loadPath("SCurve", autoPathConstraints);
  PathPlannerTrajectory secondTrajectory = PathPlanner.loadPath("SCurve", autoPathConstraints);
  
  /** Creates a new TestAuto. */
  public PPTestAuto2(DriveSubsystem drivetrain) {
    super(drivetrain);

    drivetrain.resetOdometry(firstTrajectory.getInitialHolonomicPose()); //getIntialPose
    addCommands(
      CustomPathControllerCommand(firstTrajectory),
      new InstantCommand(drivetrain::setX, drivetrain),
      new WaitCommand(2),
      new InstantCommand(drivetrain::resetModules, drivetrain),
      CustomPathControllerCommand(secondTrajectory)
    );

  }
}
