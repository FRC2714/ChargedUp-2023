// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.MIDDLE;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.align.AlignToCube;
import frc.robot.commands.auto.AutoBase;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.BUTTON;
import frc.robot.subsystems.Superstructure.CargoType;
import frc.robot.subsystems.Superstructure.DPAD;
import frc.robot.subsystems.Superstructure.ScoreMode;
import frc.robot.subsystems.Arm.Claw;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class OneCubeBalanceMiddleAuto extends AutoBase {
	List<PathPlannerTrajectory> autoPathGroup =
		PathPlanner.loadPathGroup(
			"1CubeBalanceMIDDLE",
			new PathConstraints(
			1.2,
			1.7));

	public OneCubeBalanceMiddleAuto(DriveSubsystem m_robotDrive, Superstructure m_superstructure, Shooter m_shooter, Arm m_arm, Claw m_claw, Limelight m_backLimelight) {
		super(m_robotDrive);

		SwerveAutoBuilder autoBuilder = CustomSwerveAutoBuilder();

		addCommands(
			m_claw.intakeCone(),
			m_superstructure.setScoreModeCommand(ScoreMode.ARM),
			m_superstructure.setCargoTypeCommand(CargoType.CUBE),
			m_superstructure.setScoreLevelCommand(BUTTON.Y),
			m_superstructure.setSubsystemState(DPAD.RIGHT),
			new WaitCommand(4.5).raceWith(new AlignToCube(m_robotDrive, m_backLimelight)),
			m_claw.scoreCube(),
			
            new WaitCommand(0.5),
      		m_superstructure.setSubsystemState(DPAD.DOWN),

			//Follow Path
			autoBuilder.fullAuto(autoPathGroup),

			//Autobalance
			new AutoBalance(m_robotDrive),
			m_robotDrive.stopModules()
		);

	}
}