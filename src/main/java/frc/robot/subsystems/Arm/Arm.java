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
import frc.robot.utils.ArmForwardKinematicPosition;

public class Arm {
  private Shoulder m_shoulder = new Shoulder();
  private Elbow m_elbow = new Elbow();

  private double shoulderLength = ArmConstants.kShoulderLength;//meters
  private double elbowLength = ArmConstants.kElbowLength;

  private double q1;
  private double q2;

  //private Translation2d inverseTarget = new Translation2d();

  private Translation2d estimatedPosition = new Translation2d();

  /** Creates a new Arm. */
  public Arm() {}

  private void setForwardKinematics(double shoulderAngle, double secondAngle) {
    m_shoulder.setTargetKinematicAngleRadians(shoulderAngle);
    m_elbow.setTargetKinematicAngleRadians(secondAngle);
  }

  private void setForwardKinematics(ArmForwardKinematicPosition forwardKinematicsPosition) {
    m_shoulder.setTargetKinematicAngleRadians(forwardKinematicsPosition.ShoulderAngleRadians);
    m_elbow.setTargetKinematicAngleRadians(forwardKinematicsPosition.ElbowAngleRadians);
  }

  public InstantCommand setForwardKinematicsCommand(ArmForwardKinematicPosition forwardKinematicsPosition) {
    return new InstantCommand(() -> setForwardKinematics(forwardKinematicsPosition));
  }

  public void setTargetPosition(Translation2d inverseTarget) {
    calculateInverseKinematics(inverseTarget);
    //setInverseKinematics();
  }

  private void calculateQ2(Translation2d inverseTarget) {
    q2 = -Math.acos((shoulderLength*shoulderLength + elbowLength*elbowLength - inverseTarget.getX()*inverseTarget.getX() - inverseTarget.getY()*inverseTarget.getY())/(-2*shoulderLength*elbowLength)) + 2*Math.PI;

    //other solution
    //q2 = Math.acos((-shoulderLength*shoulderLength - elbowLength*elbowLength + targetX*targetX + targetY*targetY)/(2*shoulderLength*elbowLength));
    SmartDashboard.putNumber("q2", Units.radiansToDegrees(q2));
  }

  private void calculateQ1(Translation2d inverseTarget) {
    // shoulderAngle = 
    //     Math.atan(targetX / targetY) -
    //     Math.atan((elbowLength * Math.sin(elbowAngle)) / (shoulderLength + elbowLength * Math.cos(elbowAngle)));

    q1 = -Math.atan2(inverseTarget.getY(), inverseTarget.getX()) - Math.atan((elbowLength*Math.sin(q2))/(shoulderLength + elbowLength*Math.cos(q2))) + Math.PI/2.0;
    SmartDashboard.putNumber("q1", Units.radiansToDegrees(q1));
  }

  private ArmForwardKinematicPosition calculateInverseKinematics(Translation2d inverseTarget) {
    calculateQ2(inverseTarget);
    calculateQ1(inverseTarget);
    return new ArmForwardKinematicPosition(q1, q2);
  }

  // private void setInverseKinematics() {
  //   setForwardKinematics(new ArmForwardKinematicPosition(q1, q2));
  // }

  private void estimateCurrentXY() {
    estimatedPosition = new Translation2d(
      shoulderLength*Math.sin(m_shoulder.getKinematicAngle()) + elbowLength*Math.sin(m_shoulder.getKinematicAngle()+m_elbow.getKinematicAngle()),
      shoulderLength*Math.cos(m_elbow.getKinematicAngle()) + elbowLength*Math.cos(m_shoulder.getKinematicAngle()+m_elbow.getKinematicAngle())
    );
    SmartDashboard.putNumber("Arm Estimated X", Units.metersToInches(estimatedPosition.getX()));
    SmartDashboard.putNumber("Arm Estimated Y", Units.metersToInches(estimatedPosition.getY()));
  }

  public void raiseCurrentPosition(double degrees) {
    if(m_elbow.getKinematicAngle() < 0) { //when second joint is -
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle() + Units.degreesToRadians(degrees)); // add degrees
    } else {
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle() - Units.degreesToRadians(degrees)); // subtract degrees
    }
  }

  public void lowerCurrentPosition(double degrees) {
    if(m_elbow.getKinematicAngle() > 0) { //when second joint is +
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle() + Units.degreesToRadians(degrees)); // add degrees
    } else {
      setForwardKinematics(m_shoulder.getKinematicAngle(), m_elbow.getKinematicAngle() - Units.degreesToRadians(degrees)); // subtract degrees
    }
  }

  public boolean isShoulderAtGoal() {
    return m_shoulder.atGoal();
  }

  ///////////////////////////TRANSITIONS/////////////////////////////////////////////////////
  
  //Back to Back
  public Command BackToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    CommandBase sequence = new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_elbow.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToBackIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
      //new WaitUntilCommand(() -> shoulder.atSetpoint() && elbow.atSetpoint()));
    sequence.addRequirements(m_shoulder, m_elbow);
    return sequence;
  }

  //Back to Transfer
  public Command BackToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    CommandBase sequence = new SequentialCommandGroup(
      //new WaitUntilCommand(() -> elbow.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
    sequence.addRequirements(m_shoulder, m_elbow);
    return sequence;
  }

  //Back to Front
  public Command BackToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Back to Stow
  public Command BackToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Transfer to Back
  public Command TransferToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Transfer to Front
  public Command TransferToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Transfer to Transfer
  public Command TransferToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Transfer to stow
  public Command TransferToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Front to Back
  public Command FrontToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Front to Transfer
  public Command FrontToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Front to Front
  public Command FrontToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Front to Stow
  public Command FrontToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Stow to Back
  public Command StowToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Stow to transfer
  public Command StowToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //stow to front
  public Command StowToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> m_shoulder.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  public void updateTelemetry() {
    // This method will be called once per scheduler run
    // estimateCurrentXY();
    // calculateInverseKinematics(estimatedX, estimatedY);
  }
}
