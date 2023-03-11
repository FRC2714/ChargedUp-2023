// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm.ArmStateMachine.ArmState;
import frc.robot.utils.ArmForwardKinematicPosition;

public class Arm extends SubsystemBase {
  private BaseJoint baseJoint = new BaseJoint();
  private SecondJoint secondJoint = new SecondJoint();

  private double a1 = ArmConstants.kBaseJointLength;//meters
  private double a2 = ArmConstants.kSecondJointLength;

  private double q1; //units are wrong, convert to radians?? need to make them relative to ground
  private double q2;

  private double targetX;
  private double targetY;

  private double estimatedX;
  private double estimatedY;

  /** Creates a new Superstructure. */
  public Arm() {
  }

  private void setForwardKinematics(double baseAngle, double secondAngle) {
    baseJoint.setTargetKinematicAngle(baseAngle);
    secondJoint.setTargetKinematicAngle(secondAngle);
  }

  private void setForwardKinematics(ArmForwardKinematicPosition forwardKinematicsPosition) {
    baseJoint.setTargetKinematicAngle(forwardKinematicsPosition.getBaseAngleRadians());
    secondJoint.setTargetKinematicAngle(forwardKinematicsPosition.getSecondAngleRadians());
  }

  public InstantCommand setForwardKinematicsCommand(ArmForwardKinematicPosition forwardKinematicsPosition) {
    return new InstantCommand(() -> setForwardKinematics(forwardKinematicsPosition));
  }

  public Command setForwardKinematicsUntil(Boolean condition, ArmForwardKinematicPosition forwardKinematicsPosition) {
    return new WaitUntilCommand(() -> condition).deadlineWith(setForwardKinematicsCommand(forwardKinematicsPosition));
  }

  public void setTargetPosition(double X, double Y) {
    targetX = X;
    targetY = Y;
  }

  private void calculateQ2() {
    q2 = -Math.abs(Math.acos(
        ((targetX * targetX) + (targetY * targetY) - (a1 * a1) - (a2 * a2)) /
            (2 * a1 * a2)));
  }

  private void calculateQ1() {
    q1 = Math.abs(
        Math.atan(targetX / targetY) +
            Math.atan((a2 * Math.sin(q2)) / (a1 + a2 * Math.sin(q2))));
  }

  private void calculateInverseKinematics() {
    calculateQ1();
    calculateQ2();
  }

  private void setInverseKinematics() {
    baseJoint.setTargetKinematicAngle(q1);
    secondJoint.setTargetKinematicAngle(q2);
  }

  private void estimateCurrentXY() {
    estimatedX = a1*Math.cos(baseJoint.getKinematicAngle()) + a2*Math.cos(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
    estimatedY = a2*Math.sin(secondJoint.getKinematicAngle()) + a2*Math.sin(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
    SmartDashboard.putNumber("Arm Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Arm Estimated Y", Units.metersToInches(estimatedY));
  }

  public ArmState estimateCurrentArmState() {
    if (secondJoint.getKinematicAngle() < 0) {return ArmState.BACK;} //if second joint is -, arm state is BACK
    else if (secondJoint.getKinematicAngle() > 0 && baseJoint.getKinematicAngle() < 100) {return ArmState.TRANSFER;}
    else {return ArmState.FRONT;}
  }

  public void raiseCurrentPosition(double degrees) {
    if(secondJoint.getKinematicAngle() < 0) { //when second joint is -
      setForwardKinematics(baseJoint.getKinematicAngle(), secondJoint.getKinematicAngle()+Units.degreesToRadians(degrees)); // add degrees
    } else {
      setForwardKinematics(baseJoint.getKinematicAngle(), secondJoint.getKinematicAngle()-Units.degreesToRadians(degrees)); // subtract degrees
    }
  }

  public void lowerCurrentPosition(double degrees) {
    if(secondJoint.getKinematicAngle() > 0) { //when second joint is +
      setForwardKinematics(baseJoint.getKinematicAngle(), secondJoint.getKinematicAngle()+Units.degreesToRadians(degrees)); // add degrees
    } else {
      setForwardKinematics(baseJoint.getKinematicAngle(), secondJoint.getKinematicAngle()-Units.degreesToRadians(degrees)); // subtract degrees
    }
  }

  ///////////////////////////TRANSITIONS/////////////////////////////////////////////////////
  
  //Back to Back
  public Command BackToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToBackIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Back to Transfer
  public Command BackToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Back to Front
  public Command BackToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Back to Stow
  public Command BackToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Transfer to Back
  public Command TransferToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToBackIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToBackIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Transfer to Front
  public Command TransferToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToFrontIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Transfer to Transfer
  public Command TransferToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Transfer to stow
  public Command TransferToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToStowIntermediatePosition)),
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToStowIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Front to Back
  public Command FrontToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Front to Transfer
  public Command FrontToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kFrontToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kFrontToTransferIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Front to Front
  public Command FrontToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Front to Stow
  public Command FrontToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Stow to Back
  public Command StowToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Stow to transfer
  public Command StowToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowToTransferIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //stow to front
  public Command StowToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // calculateInverseKinematics();
    // estimateCurrentXY();
  }
}
