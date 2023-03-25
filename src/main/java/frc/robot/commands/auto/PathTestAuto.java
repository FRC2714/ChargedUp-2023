// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class PathTestAuto extends AutoBase {
	DriveSubsystem m_robotDrive;

	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"ComplexAuto",
			new PathConstraints(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared));

	public PathTestAuto(DriveSubsystem m_robotDrive) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder(); 

		addCommands(
			autoBuilder.fullAuto(autoPathGroup)
		);

	}
}