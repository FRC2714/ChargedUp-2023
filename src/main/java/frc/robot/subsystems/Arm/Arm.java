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
import frc.utils.ArmForwardKinematicPosition;

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
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kBackToBackIntermediatePosition),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), backScoreLevelPosition));
  }

  //Back to Transfer
  public Command BackToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kBackToTransferIntermediatePosition),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), transferScoreLevelPosition));
  }

  //Back to Front
  public Command BackToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint(), frontScoreLevelPosition));
  }

  //Back to Stow
  public Command BackToStow() {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint(), ArmConstants.kStowPosition));
  }

  //Transfer to Back
  public Command TransferToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kTransferToBackIntermediatePosition),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), ArmConstants.kTransferToBackIntermediate2Position),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), backScoreLevelPosition));
  }

  //Transfer to Front
  public Command TransferToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kTransferToFrontIntermediatePosition),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), frontScoreLevelPosition));
  }

  //Transfer to Transfer
  public Command TransferToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint() && secondJoint.atSetpoint(), ArmConstants.kTransferToTransferIntermediatePosition),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), transferScoreLevelPosition));
  }

  //Transfer to stow
  public Command TransferToStow() {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kTransferToStowIntermediatePosition),
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kTransferToStowIntermediate2Position),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), ArmConstants.kStowPosition));
  }

  //Front to Back
  public Command FrontToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint(), backScoreLevelPosition));
  }

  //Front to Transfer
  public Command FrontToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint() && secondJoint.atSetpoint(), ArmConstants.kFrontToTransferIntermediatePosition),
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kFrontToTransferIntermediate2Position),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), transferScoreLevelPosition));
  }

  //Front to Front
  public Command FrontToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint(), frontScoreLevelPosition));
  }

  //Front to Stow
  public Command FrontToStow() {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint(), ArmConstants.kStowPosition));
  }

  //Stow to Back
  public Command StowToBack(ArmForwardKinematicPosition backScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint(), backScoreLevelPosition));
  }

  //Stow to transfer
  public Command StowToTransfer(ArmForwardKinematicPosition transferScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kStowToTransferIntermediatePosition),
      setForwardKinematicsUntil(secondJoint.atSetpoint(), ArmConstants.kStowToTransferIntermediate2Position),
      setForwardKinematicsUntil(baseJoint.atSetpoint(), transferScoreLevelPosition));
  }

  //stow to front
  public Command StowToFront(ArmForwardKinematicPosition frontScoreLevelPosition) {
    return new SequentialCommandGroup(
      setForwardKinematicsUntil(baseJoint.atSetpoint(), frontScoreLevelPosition));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // calculateInverseKinematics();
    // estimateCurrentXY();
  }
}
