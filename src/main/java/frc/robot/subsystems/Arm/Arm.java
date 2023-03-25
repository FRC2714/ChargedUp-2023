// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.ArmForwardKinematicPosition;

public class Arm extends SubsystemBase {
  private BaseJoint baseJoint= new BaseJoint();
  private SecondJoint secondJoint= new SecondJoint();

  private double baseJointLength = ArmConstants.kBaseJointLength;//meters
  private double secondJointLength = ArmConstants.kSecondJointLength;

  private double q1; //units are wrong, convert to radians?? need to make them relative to ground
  private double q2;

  private double targetX;
  private double targetY;

  private double estimatedX;
  private double estimatedY;

  /** Creates a new Arm. */
  public Arm() {}

  private void setForwardKinematics(double baseJointAngle, double secondAngle) {
    baseJoint.setTargetKinematicAngleRadians(baseJointAngle);
    secondJoint.setTargetKinematicAngleRadians(secondAngle);
  }

  private void setForwardKinematics(ArmForwardKinematicPosition forwardKinematicsPosition) {
    baseJoint.setTargetKinematicAngleRadians(forwardKinematicsPosition.getBaseAngleRadians());
    secondJoint.setTargetKinematicAngleRadians(forwardKinematicsPosition.getSecondAngleRadians());
  }

  public InstantCommand setForwardKinematicsCommand(ArmForwardKinematicPosition forwardKinematicsPosition) {
    return new InstantCommand(() -> setForwardKinematics(forwardKinematicsPosition));
  }

  private Command setForwardKinematicsUntil(BooleanSupplier condition, ArmForwardKinematicPosition forwardKinematicsPosition) {
    return new WaitUntilCommand(condition).deadlineWith(setForwardKinematicsCommand(forwardKinematicsPosition));
  }

  public void setTargetPosition(double X, double Y) {
    targetX = X;
    targetY = Y;
  }

  private void calculateQ2() {
    // secondJointAngle = -Math.acos(
    //     -((Math.pow(baseJointLength, 2)) + (Math.pow(secondJointLength, 2) - (Math.pow(targetX, 2)) - (Math.pow(targetY, 2)))) /
    //         (2 * baseJointLength * secondJointLength)) + (2*Math.PI);

    //second solution
    q2 = Math.acos((-baseJointLength*baseJointLength - secondJointLength*secondJointLength + targetX*targetX + targetY*targetY)/(2*baseJointLength*secondJointLength));
  }

  private void calculateQ1() {
    // baseJointAngle = 
    //     Math.atan(targetX / targetY) -
    //     Math.atan((secondJointLength * Math.sin(secondJointAngle)) / (baseJointLength + secondJointLength * Math.cos(secondJointAngle)));

    q1 = - Math.atan2(targetY, targetX) - Math.atan2(secondJointLength*Math.sin(q2), baseJointLength+secondJointLength*Math.cos(q2)) + Math.PI/2.0;
  }

  private void calculateInverseKinematics() {
    calculateQ2();
    calculateQ1();
  }

  private void setInverseKinematics() {
    baseJoint.setTargetKinematicAngleRadians(q1);
    secondJoint.setTargetKinematicAngleRadians(q2);
  }

  private void estimateCurrentXY() {
    estimatedX = baseJointLength*Math.sin(baseJoint.getKinematicAngle()) + secondJointLength*Math.sin(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
    estimatedY = secondJointLength*Math.cos(secondJoint.getKinematicAngle()) + secondJointLength*Math.cos(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
    SmartDashboard.putNumber("Arm Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Arm Estimated Y", Units.metersToInches(estimatedY));
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
      new WaitUntilCommand(() -> secondJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToBackIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Back to Transfer
  public Command BackToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kBackToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Back to Front
  public Command BackToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Back to Stow
  public Command BackToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Transfer to Back
  public Command TransferToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToBackIntermediatePosition)),
      //new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToBackIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Transfer to Front
  public Command TransferToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToFrontIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Transfer to Transfer
  public Command TransferToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint() && secondJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToTransferIntermediatePosition)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Transfer to stow
  public Command TransferToStow() {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToStowIntermediatePosition)),
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kTransferToStowIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Front to Back
  public Command FrontToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Front to Transfer
  public Command FrontToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kFrontToTransferIntermediatePosition)),
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kFrontToTransferIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //Front to Front
  public Command FrontToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  //Front to Stow
  public Command FrontToStow() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowPosition)));
  }

  //Stow to Back
  public Command StowToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(backScoreLevelPosition)));
  }

  //Stow to transfer
  public Command StowToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowToTransferIntermediatePosition)),
      //new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(setForwardKinematicsCommand(ArmConstants.kStowToTransferIntermediate2Position)),
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(transferScoreLevelPosition)));
  }

  //stow to front
  public Command StowToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.nearSetpoint()).deadlineWith(setForwardKinematicsCommand(frontScoreLevelPosition)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateInverseKinematics();
    estimateCurrentXY();

    SmartDashboard.putNumber("q1", q1);
    SmartDashboard.putNumber("q2", q2);

    SmartDashboard.putNumber("BaseJoint Kinematic Angle", Units.radiansToDegrees(baseJoint.getKinematicAngle()));
    SmartDashboard.putNumber("SecondJoint Kinematic Angle", Units.radiansToDegrees(secondJoint.getKinematicAngle()));
  }
}
