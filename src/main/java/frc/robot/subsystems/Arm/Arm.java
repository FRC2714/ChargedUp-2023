// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.utils.ArmPreset;

public class Arm {
  private Shoulder m_shoulder = new Shoulder();
  private Elbow m_elbow = new Elbow();

  private double shoulderLength = ShoulderConstants.kShoulderLength;//meters
  private double elbowLength = ElbowConstants.kElbowLength;

  //private Translation2d inverseTarget = new Translation2d();

  private Translation2d estimatedPosition = new Translation2d();

  /** Creates a new Arm. */
  public Arm() {}

  private void setPreset(ArmPreset armPreset) {
    m_shoulder.setTargetKinematicAngleRadians(armPreset.ShoulderAngleRadians);
    m_elbow.setTargetKinematicAngleRadians(armPreset.ElbowAngleRadians);
  }

  public InstantCommand setPresetCommand(ArmPreset armPreset) {
    return new InstantCommand(() -> setPreset(armPreset));
  }

  public void setTargetPosition(Translation2d inverseTarget) {
    setPreset(calculateInverseKinematics(inverseTarget));
  }

  private ArmPreset calculateInverseKinematics(Translation2d inverseTarget) {
    double q2 = -Math.acos((shoulderLength*shoulderLength + elbowLength*elbowLength - inverseTarget.getX()*inverseTarget.getX() - inverseTarget.getY()*inverseTarget.getY())/(-2*shoulderLength*elbowLength)) + 2*Math.PI;

    //other solution
    //q2 = Math.acos((-shoulderLength*shoulderLength - elbowLength*elbowLength + targetX*targetX + targetY*targetY)/(2*shoulderLength*elbowLength));
    SmartDashboard.putNumber("q2", Units.radiansToDegrees(q2));

    // shoulderAngle = 
    //     Math.atan(targetX / targetY) -
    //     Math.atan((elbowLength * Math.sin(elbowAngle)) / (shoulderLength + elbowLength * Math.cos(elbowAngle)));

    double q1 = -Math.atan2(inverseTarget.getY(), inverseTarget.getX()) - Math.atan((elbowLength*Math.sin(q2))/(shoulderLength + elbowLength*Math.cos(q2))) + Math.PI/2.0;
    SmartDashboard.putNumber("q1", Units.radiansToDegrees(q1));

    return new ArmPreset(q1, q2);
  }

  private void calculateForwardKinematics() {
    estimatedPosition = new Translation2d(
      shoulderLength*Math.sin(m_shoulder.getKinematicAngle()) + elbowLength*Math.sin(m_shoulder.getKinematicAngle()+m_elbow.getKinematicAngle()),
      shoulderLength*Math.cos(m_elbow.getKinematicAngle()) + elbowLength*Math.cos(m_shoulder.getKinematicAngle()+m_elbow.getKinematicAngle())
    );
    SmartDashboard.putNumber("Arm Estimated X", Units.metersToInches(estimatedPosition.getX()));
    SmartDashboard.putNumber("Arm Estimated Y", Units.metersToInches(estimatedPosition.getY()));
  }

  public boolean isShoulderAtGoal() {
    return m_shoulder.atGoal();
  }

  ///////////////////////////TRANSITIONS/////////////////////////////////////////////////////
  
  //Back to Back
  public Command BackToBack(ArmPreset backScoreLevelPosition) {
    CommandBase sequence = new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_elbow.nearSetpoint()).deadlineWith(setPresetCommand(ArmConstants.kBackToBackIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(backScoreLevelPosition)));
      //new WaitUntilCommand(() -> shoulder.atSetpoint() && elbow.atSetpoint()));
    sequence.addRequirements(m_shoulder, m_elbow);
    return sequence;
  }

  //Back to Transfer
  public Command BackToTransfer(ArmPreset transferScoreLevelPosition) {
    CommandBase sequence = new SequentialCommandGroup(
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(transferScoreLevelPosition)));
    sequence.addRequirements(m_shoulder, m_elbow);
    return sequence;
  }

  //Back to Front
  public Command BackToFront(ArmPreset frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(frontScoreLevelPosition)));
  }

  //Back to Stow
  public Command BackToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(ArmConstants.kStowPosition)));
  }

  //Transfer to Back
  public Command TransferToBack(ArmPreset backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(backScoreLevelPosition)));
  }

  //Transfer to Front
  public Command TransferToFront(ArmPreset frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(frontScoreLevelPosition)));
  }

  //Transfer to Transfer
  public Command TransferToTransfer(ArmPreset transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(transferScoreLevelPosition)));
  }

  //Transfer to stow
  public Command TransferToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(ArmConstants.kStowPosition)));
  }

  //Front to Back
  public Command FrontToBack(ArmPreset backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(backScoreLevelPosition)));
  }

  //Front to Transfer
  public Command FrontToTransfer(ArmPreset transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(transferScoreLevelPosition)));
  }

  //Front to Front
  public Command FrontToFront(ArmPreset frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(frontScoreLevelPosition)));
  }

  //Front to Stow
  public Command FrontToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(ArmConstants.kStowPosition)));
  }

  //Stow to Back
  public Command StowToBack(ArmPreset backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(backScoreLevelPosition)));
  }

  //Stow to transfer
  public Command StowToTransfer(ArmPreset transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(transferScoreLevelPosition)));
  }

  //stow to front
  public Command StowToFront(ArmPreset frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setPresetCommand(frontScoreLevelPosition)));
  }

  public void updateTelemetry() {
    // This method will be called once per scheduler run
    // estimateCurrentXY();
    // calculateInverseKinematics(estimatedX, estimatedY);
  }
}
