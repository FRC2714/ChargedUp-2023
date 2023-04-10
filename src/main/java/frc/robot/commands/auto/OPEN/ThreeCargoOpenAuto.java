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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.auto.AutoBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.BUTTON;
import frc.robot.subsystems.Superstructure.DPAD;
import frc.robot.subsystems.Superstructure.ScoreMode;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class ThreeCargoOpenAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"3CargoOPEN",
			new PathConstraints(
			3.5,
			3.5));

	public ThreeCargoOpenAuto(DriveSubsystem m_robotDrive, Superstructure m_superstructure, Shooter m_shooter, Arm m_arm, Claw m_claw, Limelight m_backLimelight) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();

    	AutoConstants.AutoEventMap.put("intake cube", 
			m_superstructure.intakeRightTrigger());
		AutoConstants.AutoEventMap.put("set shooter high", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.Y),
				m_superstructure.setSubsystemState(DPAD.RIGHT)));
        AutoConstants.AutoEventMap.put("set shooter mid", 
			new SequentialCommandGroup(
				m_superstructure.setScoreLevelCommand(BUTTON.B),
				m_superstructure.setSubsystemState(DPAD.RIGHT)));
		AutoConstants.AutoEventMap.put("shoot cube", 
			new SequentialCommandGroup(
				m_shooter.setKickerCommand(ShooterConstants.kKickSpeed),
				new WaitCommand(0.2),
				m_shooter.stopCommand()
			));
		AutoConstants.AutoEventMap.put("retract shooter", 
			m_superstructure.setSubsystemState(DPAD.DOWN));

		addCommands(
			m_superstructure.scorePreloadedCone(3.5), //Score First Cone

      		m_superstructure.setScoreModeCommand(ScoreMode.SHOOTER),
			new WaitCommand(0.5),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup),

			//Autobalance
			new AutoBalance(m_robotDrive)
		);
	}
}