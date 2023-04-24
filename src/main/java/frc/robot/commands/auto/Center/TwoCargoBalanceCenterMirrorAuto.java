// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.Center;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.auto.AutoBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.BUTTON;
import frc.robot.subsystems.Superstructure.DPAD;
import frc.robot.subsystems.Superstructure.ScoreMode;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.utils.ShooterPreset;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class TwoCargoBalanceCenterMirrorAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"2CargoBalanceCENTERmirror",
			new PathConstraints(1.2, 2.5),
			new PathConstraints(1.2, 2.5),
			new PathConstraints(2.1, 2.5),
			new PathConstraints(2.1, 2.5),
			new PathConstraints(1.2, 2.5)
			);

	private final HashMap<String, Command> AutoEventMap = new HashMap<>();

	public TwoCargoBalanceCenterMirrorAuto(DriveSubsystem m_drivetrain, Superstructure m_superstructure, Shooter m_shooter) {
		super(m_drivetrain);

		AutoEventMap.put("intake cube", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.X),
				m_superstructure.setSubsystemState(DPAD.LEFT)));
		AutoEventMap.put("set shooter custom",
			new SequentialCommandGroup(
				m_shooter.setPreset(new ShooterPreset(40, 150))));

		SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder(AutoEventMap);

		addCommands(
			m_superstructure.scorePreloadedCone(3.1),

            m_superstructure.setScoreModeCommand(ScoreMode.SHOOTER),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup),

			//Autobalance
			new AutoBalance(m_drivetrain, true),
			m_shooter.setKickerOuttakeCommand(ShooterConstants.kKickSpeed),
			new AutoBalance(m_drivetrain, true),
			new WaitCommand(5)
				.alongWith(new RunCommand(() -> m_drivetrain.setX(), m_drivetrain))
		);

	}
}