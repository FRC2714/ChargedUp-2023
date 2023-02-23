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
import frc.robot.commands.Autoalign;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class TwoCargoOpenAuto extends AutoBase {
    DriveSubsystem m_robotDrive;
	Intake m_intake;
	Arm m_arm;

	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"MarkerAuto",
			new PathConstraints(
			AutoConstants.kMaxSpeedMetersPerSecond,
			AutoConstants.kMaxAccelerationMetersPerSecondSquared));

	public TwoCargoOpenAuto(DriveSubsystem m_robotDrive, Intake m_intake, Arm m_arm, Limelight m_limelight) {
		super(m_robotDrive);
		this.m_intake = m_intake;
		this.m_arm = m_arm;

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();
        AutoConstants.EventMap.put("auto align", new Autoalign(m_robotDrive, m_limelight));
		AutoConstants.EventMap.put("intake cone", m_intake.intakeCone());
		//AutoConstants.EventMap.put("handoff cone", transfer command); //todo

		addCommands(

			autoBuilder.fullAuto(autoPathGroup)
		);

	}
}