// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  //back to transfer transition
  public Command BackToTransferIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToTransferIntermediatePosition));
  }
  public Command BackToTransferIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToTransferIntermediate2Position));
  }

  //back to back transition
  public Command BackToBackIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToBackIntermediatePosition));
  }

  //Back to front
  public Command BackToFrontIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToFrontIntermediatePosition));
  }
  public Command BackToFrontIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToFrontIntermediate2Position));
  }

  //transfer to back transition
  public Command TransferToBackIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToBackIntermediatePosition));
  }
  public Command TransferToBackIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToBackIntermediate2Position));
  }
  public Command TransferToBackIntermediate3Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToBackIntermediate3Position));
  }
  public Command TransferToBackIntermediate4Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToBackIntermediate4Position));
  }

  //Transfer to front
  public Command TransferToFrontIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToFrontIntermediatePosition));
  }
  public Command TransferToFrontIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToFrontIntermediate2Position));
  }

  //Front to transfer transition
  public Command FrontToTransferIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontToTransferIntermediatePosition));
  }
  public Command FrontToTransferIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontToTransferIntermediate2Position));
  }

  //Front to back
  public Command FrontToBackIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontToBackIntermediatePosition));
  }
  public Command FrontToBackIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontToBackIntermediate2Position));
  }
  

  //transfer position
  public Command TransferCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferPosition));
  }

  //back cone score levels
  public Command ConeL1Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kConeL1Position));
  }
  public Command ConeL2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kConeL2Position));
  }
  public Command ConeL3Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kConeL3Position));
  }

  //back cube score levels
  public Command CubeL1Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kCubeL1Position));
  }
  public Command CubeL2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kCubeL2Position));
  }
  public Command CubeL3Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kCubeL3Position));
  }

  //arm intake 
  public Command BackIntakeCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackIntakePosition));
  }
  public Command FrontIntakeCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontIntakePosition));
  }



  
  
  //Back to Transfer
  public Command BackToTransfer() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(BackToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(BackToTransferIntermediate2Command()),
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(TransferCommand()));
  }

  //Back to Back
  public Command BackToBack(Command scoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(BackToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(scoreLevelCommand));
  }

  //Back to front
  public Command BackToFront() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(BackToFrontIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(BackToFrontIntermediate2Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(FrontIntakeCommand()));
  }

  //Transfer to Back
  public Command TransferToBack(Command scoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(TransferToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(TransferToBackIntermediate2Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(TransferToBackIntermediate3Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(TransferToBackIntermediate4Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(scoreLevelCommand));
  }

  //Transfer to front
  public Command TransferToFront() {
    return new SequentialCommandGroup(

      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(TransferToFrontIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(TransferToFrontIntermediate2Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(FrontIntakeCommand()));
  }

  //Front to transfer transition
  public Command FrontToTransfer() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(FrontToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(FrontToTransferIntermediate2Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(TransferCommand()));
  }
  
  //Front to back
  public Command FrontToBack(Command scoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(FrontToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(FrontToBackIntermediate2Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(scoreLevelCommand));
  }

  

  




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateInverseKinematics();
    estimateCurrentXY();
  }
}
