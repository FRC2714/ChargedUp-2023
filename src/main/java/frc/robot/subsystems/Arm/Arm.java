// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
  // private Joint baseJoint;
  // private Joint secondJoint;

  private BaseJoint baseJoint= new BaseJoint();
  private SecondJoint secondJoint= new SecondJoint();

  private ProfiledPIDController baseJointController;
  private ProfiledPIDController secondJointController;

  private double baseJointLength = ArmConstants.kBaseJointLength;//meters
  private double secondJointLength = ArmConstants.kSecondJointLength;

  private double baseJointAngle; //units are wrong, convert to radians?? need to make them relative to ground
  private double secondJointAngle;

  private double targetX;
  private double targetY;

  private double estimatedX;
  private double estimatedY;

  /** Creates a new Superstructure. */
  public Arm() {
    baseJointController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
    secondJointController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));

    // baseJoint = new Joint(
    //   ArmConstants.kRightBaseJointMotorCanId, 
    //   ArmConstants.kLeftBaseJointMotorCanId, 
    //   true, 
    //   ArmConstants.kBaseJointMotorInverted, 
    //   ArmConstants.kBaseJointPositionConversionFactor, 
    //   ArmConstants.kBaseJointEncoderInverted,
    //   ArmConstants.kBaseJointEncoderZeroOffset,
    //   baseJointController,
    //   4,
    //   ArmConstants.kBaseJointKinematicOffset,
    //   ArmConstants.kBaseJointGearRatio);

    // secondJoint = new Joint(
    //   ArmConstants.kRightSecondJointMotorCanId, 
    
    //   ArmConstants.kLeftSecondJointMotorCanId, 
    //   false, 
    //   ArmConstants.kSecondJointMotorInverted, 
    //   ArmConstants.kSecondJointPositionConversionFactor, 
    //   ArmConstants.kSecondJointEncoderInverted,
    //   ArmConstants.kSecondJointEncoderZeroOffset,
    //   secondJointController,
    //   8,
    //   ArmConstants.kSecondJointKinematicOffset,
    //   ArmConstants.kSecondJointGearRatio);

  }

  private void setForwardKinematics(double baseJointAngle, double secondAngle) {
    baseJoint.setTargetKinematicAngle(baseJointAngle);
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
    secondJointAngle = -Math.abs(Math.acos(
        ((targetX * targetX) + (targetY * targetY) - (baseJointLength * baseJointLength) - (secondJointLength * secondJointLength)) /
            (2 * baseJointLength * secondJointLength)));
  }

  private void calculateQ1() {
    baseJointAngle = Math.abs(
        Math.atan(targetX / targetY) +
            Math.atan((secondJointLength * Math.sin(secondJointAngle)) / (baseJointLength + secondJointLength * Math.sin(secondJointAngle))));
  }

  private void calculateInverseKinematics() {
    calculateQ1();
    calculateQ2();
  }

  private void setInverseKinematics() {
    baseJoint.setTargetKinematicAngle(baseJointAngle);
    secondJoint.setTargetKinematicAngle(secondJointAngle);
  }

  private void estimateCurrentXY() {
    estimatedX = baseJointLength*Math.cos(baseJoint.getKinematicAngle()) + secondJointLength*Math.cos(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
    estimatedY = secondJointLength*Math.sin(secondJoint.getKinematicAngle()) + secondJointLength*Math.sin(baseJoint.getKinematicAngle()+secondJoint.getKinematicAngle());
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
    // calculateInverseKinematics();
    // estimateCurrentXY();

    SmartDashboard.putNumber("BaseJoint Kinematic Angle", Units.radiansToDegrees(baseJoint.getKinematicAngle()));
    SmartDashboard.putNumber("SecondJoint Kinematic Angle", Units.radiansToDegrees(secondJoint.getKinematicAngle()));

    //SmartDashboard.putNumber("BaseJoint Target Angle", Units.radiansToDegrees(baseJoint.getTargetKinematicAngle()));
    //SmartDashboard.putNumber("SecondJoint Target Angle", Units.radiansToDegrees(secondJoint.getTargetKinematicAngle()));
  }
}
