// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.Center;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.auto.AutoBase;
import frc.robot.subsystems.Superstructure.DPAD;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class OneConeBalanceCenterAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"1ConeBalanceCENTER",
			new PathConstraints(1.2, 1.7));

	public OneConeBalanceCenterAuto(DriveSubsystem m_drivetrain, Superstructure m_superstructure) {
		super(m_drivetrain);

		SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder();

		addCommands(
			m_superstructure.scorePreloadedCone(4.5),

      		m_superstructure.setSubsystemState(DPAD.DOWN),
			new WaitCommand(1),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup),

			//Autobalance
			new AutoBalance(m_drivetrain, false),
			new WaitCommand(5)
				.alongWith(new RunCommand(() -> m_drivetrain.setX(), m_drivetrain))
		);

	}
}