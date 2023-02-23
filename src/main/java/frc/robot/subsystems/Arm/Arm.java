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

  private void setFowardKinematics(double baseAngle, double secondAngle) {
    baseJoint.setTargetKinematicAngle(baseAngle);
    secondJoint.setTargetKinematicAngle(secondAngle);
  }

  private void setFowardKinematics(ArmForwardKinematicPosition forwardKinematicsPosition) {
    baseJoint.setTargetKinematicAngle(forwardKinematicsPosition.getBaseAngleRadians());
    secondJoint.setTargetKinematicAngle(forwardKinematicsPosition.getSecondAngleRadians());
  }

  public void setTargetPosition(double X, double Y) {
    targetX = X;
    targetY = Y;
  }

  private void calculateQ2() {
    q2 = -Math.abs(Math.acos(
        ((targetX*targetX)+(targetY*targetY)-(a1*a1)-(a2*a2)) /
        (2*a1*a2)));
  }

  private void calculateQ1() {
    q1 = Math.abs(
      Math.atan(targetX/targetY) +
      Math.atan((a2*Math.sin(q2)) / (a1+a2*Math.sin(q2))));
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
    SmartDashboard.putNumber("Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Estimated Y", Units.metersToInches(estimatedY));
  }

  public ArmState estimateCurrentArmState() {
    if (secondJoint.getKinematicAngle() < 0) {return ArmState.BACK;} //if second joint is -, arm state is BACK
    else if (secondJoint.getKinematicAngle() > 0 && baseJoint.getKinematicAngle() < 100) {return ArmState.TRANSFER;}
    else {return ArmState.FRONT;}
  }

  //define position commands
  public Command backToTransferIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToTransferIntermediatePosition));
  }

  public Command backToTransferIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToTransferIntermediate2Position));
  }

  public Command transferCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferPosition));
  }

  public Command transferToBackIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToBackIntermediatePosition));
  }

  public Command levelTwoCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kLevelTwoPosition));
  }

  public Command levelThreeCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kLevelThreePosition));
  }
  
  //define position sequences
  public Command backToTransfer() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(backToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(backToTransferIntermediate2Command()),
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(transferCommand()));
  }

  /* 
  public Command transferToBackLevelTwo() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(transferCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(swingOutCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(transferToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(backLevelTwoCommand()));
  }
  
  public Command transferToBackLevelThree() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(transferCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(swingOutCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(transferToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(backLevelThreeCommand()));
  }
  */

  public Command transferToBack(Command scoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(transferCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(backToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(transferToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(scoreLevelCommand));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateInverseKinematics();
    estimateCurrentXY();
  }
}
