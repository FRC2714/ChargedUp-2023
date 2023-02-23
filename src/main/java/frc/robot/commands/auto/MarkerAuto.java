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
import frc.robot.subsystems.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class MarkerAuto extends AutoBase {
    DriveSubsystem m_robotDrive;
	Intake m_intake;
	Arm m_arm;

	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"MarkerAuto",
			new PathConstraints(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared));

	public MarkerAuto(DriveSubsystem m_robotDrive, Intake m_intake, Arm m_arm) {
		super(m_robotDrive);
		this.m_intake = m_intake;
		this.m_arm = m_arm;

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();
        AutoConstants.EventMap.put("deployIntake", m_intake.intakeCone());
		AutoConstants.EventMap.put("armUp", m_arm.swingOutCommand());
		AutoConstants.EventMap.put("retractIntake", m_intake.retractAndStop());

		addCommands(
			autoBuilder.fullAuto(autoPathGroup)
		);

	}
}