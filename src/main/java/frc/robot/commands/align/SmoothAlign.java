// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;

public class SmoothAlign extends CommandBase {
  private DriveSubsystem m_robotDrive;
  private Limelight m_limelight;
  private ArmStateMachine m_armStateMachine;

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  private double kPXControllerCone = 0.8;
  private double kPYControllerCone = 1.1;

  private double kPXControllerCube = 0.65;
  private double kPYControllerCube = 1.1;

  private double kPThetaController = 1;

  private double xControllerGoalCone = 0.37;
  private double xControllerGoalCube = 0.40;

  //private double thetaControllerkP

  /** Creates a new SmoothAlign. */
  public SmoothAlign(DriveSubsystem m_robotDrive, Limelight m_limelight, ArmStateMachine m_armStateMachine) {
    this.m_robotDrive = m_robotDrive;
    this.m_limelight = m_limelight;
    this.m_armStateMachine = m_armStateMachine;

    // Use addRequirements() here to declare subsystem dependencies.
    xController = new ProfiledPIDController(kPXControllerCone, 0, 0, AutoConstants.kAutoControllerConstraints);
    yController = new ProfiledPIDController(kPYControllerCone, 0, 0, AutoConstants.kAutoControllerConstraints);
    thetaController = new ProfiledPIDController(kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
    addRequirements(m_robotDrive);

    xController.setGoal(xControllerGoalCone);
    xController.setTolerance(0.05,0);

    yController.setGoal(0);
    yController.setTolerance(Units.degreesToRadians(0),0);

    thetaController.setGoal(0);
    thetaController.setTolerance(Units.degreesToRadians(0),0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_armStateMachine.getCargoType() == CargoType.CONE) {
      m_limelight.setRetroPipeline();
      xController.setGoal(xControllerGoalCone);
      xController.setP(kPXControllerCone);
      yController.setP(kPYControllerCone);
    } else {
      m_limelight.setAprilTagPipeline();
      xController.setGoal(xControllerGoalCube);
      xController.setP(kPXControllerCube);
      yController.setP(kPYControllerCube);
    }
    m_limelight.setLED(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.drive(
        xController.calculate(m_limelight.getDistanceToGoalMeters()), 
        yController.calculate(m_limelight.getXOffsetRadians()), 
        thetaController.calculate(m_robotDrive.getHeadingRadians()), 
        true,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_limelight.setAprilTagPipeline();
    m_limelight.setLED(false);
    m_robotDrive.drive(0, 0, 0, true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
