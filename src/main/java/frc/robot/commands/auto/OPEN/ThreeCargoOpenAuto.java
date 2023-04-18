// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.OPEN;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.auto.AutoBase;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.BUTTON;
import frc.robot.subsystems.Superstructure.DPAD;
import frc.robot.subsystems.Superstructure.ScoreMode;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ThreeCargoOpenAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"3CargoOPEN",
			new PathConstraints(
			3.0,
			3.0));

	public ThreeCargoOpenAuto(DriveSubsystem m_robotDrive, Superstructure m_superstructure, Shooter m_shooter) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = getSwerveAutoBuilder();

    	AutoEventMap.put("intake cube", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.X),
				m_superstructure.setSubsystemState(DPAD.LEFT)));
		AutoEventMap.put("set shooter high", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.Y),
				m_superstructure.setSubsystemState(DPAD.RIGHT)));
        AutoEventMap.put("set shooter mid", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.B),
				m_superstructure.setSubsystemState(DPAD.RIGHT)));
		AutoEventMap.put("shoot cube", 
			new SequentialCommandGroup(
				m_shooter.setKickerOuttakeCommand(ShooterConstants.kKickSpeed),
				new WaitCommand(0.2),
				m_shooter.stopCommand()));
		AutoEventMap.put("retract shooter", 
			m_superstructure.setSubsystemState(DPAD.DOWN));

		addCommands(
			m_superstructure.scorePreloadedCone(3.0), //Score First Cone

      		m_superstructure.setScoreModeCommand(ScoreMode.SHOOTER),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup)
		);
	}
}