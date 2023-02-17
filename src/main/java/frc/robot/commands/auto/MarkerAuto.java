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
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class MarkerAuto extends AutoBase {
    DriveSubsystem m_robotDrive;

	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"MarkerAuto",
			new PathConstraints(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared));

	public MarkerAuto(DriveSubsystem m_robotDrive, Intake m_intake) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();
        AutoConstants.EventMap.put("deployintake", m_intake.intakeCone());


		addCommands(
			autoBuilder.followPathGroupWithEvents(autoPathGroup)
		);

	}
}