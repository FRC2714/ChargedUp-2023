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
    SmartDashboard.putNumber("Estimated X", Units.metersToInches(estimatedX));
    SmartDashboard.putNumber("Estimated Y", Units.metersToInches(estimatedY));
  }

  public ArmState estimateCurrentArmState() {
    if (secondJoint.getKinematicAngle() < 0) {return ArmState.BACK;} //if second joint is -, arm state is BACK
    else if (secondJoint.getKinematicAngle() > 0 && baseJoint.getKinematicAngle() < 100) {return ArmState.TRANSFER;}
    else {return ArmState.FRONT;}
  }

  public void raiseCurrentPosition(double degrees) {
    setFowardKinematics(baseJoint.getKinematicAngle(), secondJoint.getKinematicAngle()+Units.degreesToRadians(degrees));
  }


  //////////////////////////////INTERMEDIATE POSITIONS/////////////////////////////

  //back to transfer intermediate positions
  private Command BackToTransferIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToTransferIntermediatePosition));
  }

  //back to back intermediate positions
  private Command BackToBackIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackToBackIntermediatePosition));
  }

  //Back to font intermediate positions

  //Back to Tuck intermediate positions

  //transfer to back intermediate positions
  private Command TransferToBackIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToBackIntermediatePosition));
  }
  private Command TransferToBackIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToBackIntermediate2Position));
  }

  //transfer to transfer intermediate positions
  private Command TransferToTransferIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToTransferIntermediatePosition));
  }

  //Transfer to front intermediate positions
  private Command TransferToFrontIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferToFrontIntermediatePosition));
  }

  //Transfer to tuck intermediate positions

  //Front to transfer intermediate positions
  private Command FrontToTransferIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontToTransferIntermediatePosition));
  }
  private Command FrontToTransferIntermediate2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontToTransferIntermediate2Position));
  }

  //Front to back intermediate positions

  //font to tuck intermediate positions

  //tuck to back intermediate positions

  //tuck to transfer intermediate positions
  private Command TuckToTransferIntermediateCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTuckToTransferIntermediatePosition));
  }

  //tuck to front intermediate positions

  //tuck position
  public Command TuckCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTuckPosition));
  }

  //transfer position
  public Command TransferConeIntakeCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferConeIntakePosition));
  }
  public Command TransferCubeIntakeCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kTransferCubeIntakePosition));
  }

  //back cone score levels
  public Command BackConeL1Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackConeL1Position));
  }
  public Command BackConeL2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackConeL2Position));
  }
  public Command BackConeL3Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackConeL3Position));
  }

  //back cube score levels
  public Command BackCubeL1Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackCubeL1Position));
  }
  public Command BackCubeL2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackCubeL2Position));
  }
  public Command BackCubeL3Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackCubeL3Position));
  }

  //front cone levels
  public Command FrontConeL2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontConeL2Position));
  }

  //front cube levels
  public Command FrontCubeL2Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontCubeL2Position));
  }
  public Command FrontCubeL3Command() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontCubeL3Position));
  }


  //arm intake 
  public Command BackIntakeCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kBackIntakePosition));
  }
  public Command FrontIntakeCommand() {
    return new InstantCommand(() -> setFowardKinematics(ArmConstants.kFrontIntakePosition));
  }

  ///////////////////////////TRANSITIONS/////////////////////////////////////////////////////
  
  //Back to Back
  public Command BackToBack(Command backScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(BackToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(backScoreLevelCommand));
  }

  //Back to Transfer
  public Command BackToTransfer(Command transferScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(BackToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(transferScoreLevelCommand));
  }

  //Back to Front
  public Command BackToFront(Command frontScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(frontScoreLevelCommand));
  }

  //Back to Tuck
  public Command BackToTuck() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(TuckCommand()));
  }

  //Transfer to Back
  public Command TransferToBack(Command backScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(TransferToBackIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(TransferToBackIntermediate2Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(backScoreLevelCommand));
  }

  //Transfer to Front
  public Command TransferToFront(Command frontScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(TransferToFrontIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(frontScoreLevelCommand));
  }

  //Transfer to Transfer
  public Command TransferToTransfer(Command transferScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(TransferToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(transferScoreLevelCommand));
  }

  //Transfer to tuck
  public Command TransferToTuck() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(TuckCommand()));
  }

  //Front to Back
  public Command FrontToBack(Command backScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(backScoreLevelCommand));
  }

  //Front to Transfer
  public Command FrontToTransfer(Command transferScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(FrontToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> secondJoint.atSetpoint()).deadlineWith(FrontToTransferIntermediate2Command()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(transferScoreLevelCommand));
  }

  //Front to Front
  public Command FrontToFront(Command frontScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(frontScoreLevelCommand));
  }

  //Front to Tuck
  public Command FrontToTuck() {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(TuckCommand()));
  }

  //Tuck to Back
  public Command TuckToBack(Command backScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(backScoreLevelCommand));
  }

  //Tuck to transfer
  public Command TuckToTransfer(Command transferScoreLevelCommand) {
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint() && secondJoint.atSetpoint()).deadlineWith(TuckToTransferIntermediateCommand()),
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(transferScoreLevelCommand));
  }

  //tuck to front
  public Command TuckToFront(Command frontScoreLevelCommand) {
    System.out.println("tuck to front");
    return new SequentialCommandGroup(
      new WaitUntilCommand(() -> baseJoint.atSetpoint()).deadlineWith(frontScoreLevelCommand));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    calculateInverseKinematics();
    estimateCurrentXY();
  }
}
