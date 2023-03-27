// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Superstructure.CargoType;
import frc.robot.subsystems.Superstructure.ArmScoreLevel;

public class ArmStateMachine extends SubsystemBase {
  private Arm m_arm;
  private Claw m_claw;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT, STOW
  }

  public ArmState currentArmState = ArmState.BACK; //will default to TRANSFER 
  public ArmState targetArmState = ArmState.BACK; // default to TRANSFER 

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm, Claw m_claw) {
    this.m_arm = m_arm;
    this.m_claw = m_claw;
  }

  private void setTargetArmState(ArmState targetArmState) {
    if(this.targetArmState != targetArmState || targetArmState != ArmState.STOW) {
      currentArmState = this.targetArmState;
      this.targetArmState = targetArmState;
    }
  }

  // private void callArmCommand() {
  //   getArmCommand().withInterruptBehavior(InterruptionBehavior.kCancelSelf).schedule();
  // }

  public Command setTargetArmStateCommand(ArmState targetArmState) {
    return new InstantCommand(() -> setTargetArmState(targetArmState));
  }

  private Command nothingCommand() {
    return new InstantCommand();
  }

  public Command getArmCommand(ArmScoreLevel armScoreLevel, CargoType cargoType) {
    switch (targetArmState) {
      case BACK: {
        switch (currentArmState) { // when target arm state = BACK
          case BACK: {
            switch (cargoType) { 
              // when current is back
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.BackToBack(ArmConstants.kBackConeL3Position);
                  case TWO:
                    return m_arm.setForwardKinematicsCommand(ArmConstants.kBackConeL2Position);
                  case ONE:
                    return m_arm.BackToBack(ArmConstants.kBackConeL1Position);
                  case INTAKE:
                    return m_arm.setForwardKinematicsCommand(ArmConstants.kBackIntakePosition).andThen(m_claw.intakeOpenCommand());
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.BackToBack(ArmConstants.kBackCubeL3Position);
                  case TWO:
                    return m_arm.BackToBack(ArmConstants.kBackCubeL2Position);
                  case ONE:
                    return m_arm.BackToBack(ArmConstants.kBackCubeL1Position);
                  case INTAKE:
                    return m_arm.BackToBack(ArmConstants.kBackIntakePosition).andThen(m_claw.intakeOpenCommand());
                }
              }
            }
          }
          case TRANSFER: { // When current is TRANSFER
            switch (cargoType) { 
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.TransferToBack(ArmConstants.kBackConeL3Position);
                  case TWO:
                    return m_arm.TransferToBack(ArmConstants.kBackConeL2Position);
                  case ONE:
                    return m_arm.TransferToBack(ArmConstants.kBackConeL1Position);
                  case INTAKE:
                    return m_arm.TransferToBack(ArmConstants.kBackIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.TransferToBack(ArmConstants.kBackCubeL3Position);
                  case TWO:
                    return m_arm.TransferToBack(ArmConstants.kBackCubeL2Position);
                  case ONE:
                    return m_arm.TransferToBack(ArmConstants.kBackCubeL1Position);
                  case INTAKE:
                    return m_arm.TransferToBack(ArmConstants.kBackIntakePosition);
                }
              }
            }
          }
          case FRONT: { // when current is FRONT
            switch (cargoType) { 
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.FrontToBack(ArmConstants.kBackConeL3Position);
                  case TWO:
                    return m_arm.FrontToBack(ArmConstants.kBackConeL2Position);
                  case ONE:
                    return m_arm.FrontToBack(ArmConstants.kBackConeL1Position);
                  case INTAKE:
                    return m_arm.FrontToBack(ArmConstants.kBackIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.FrontToBack(ArmConstants.kBackCubeL3Position);
                  case TWO:
                    return m_arm.FrontToBack(ArmConstants.kBackCubeL2Position);
                  case ONE:
                    return m_arm.FrontToBack(ArmConstants.kBackCubeL1Position);
                  case INTAKE:
                    return m_arm.FrontToBack(ArmConstants.kBackIntakePosition);
                }
              }
            }
          }
          case STOW: { // when current is STOW
            switch (cargoType) { 
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.StowToBack(ArmConstants.kBackConeL3Position);
                  case TWO:
                    return m_arm.StowToBack(ArmConstants.kBackConeL2Position);
                  case ONE:
                    return m_arm.StowToBack(ArmConstants.kBackConeL1Position);
                  case INTAKE:
                    return m_arm.StowToBack(ArmConstants.kBackIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.StowToBack(ArmConstants.kBackCubeL3Position);
                  case TWO:
                    return m_arm.StowToBack(ArmConstants.kBackCubeL2Position);
                  case ONE:
                    return m_arm.StowToBack(ArmConstants.kBackCubeL1Position);
                  case INTAKE:
                    return m_arm.StowToBack(ArmConstants.kBackIntakePosition);
                }
              }
            }
          }
        }
      }
      case TRANSFER: { // when target arm state = TRANSFER
        switch (currentArmState) { 
          case BACK: { //when current arm state = BACK
            switch (cargoType) {
              case CONE:
                return m_arm.BackToTransfer(ArmConstants.kTransferConeIntakePosition);
              case CUBE:
                return m_arm.BackToTransfer(ArmConstants.kTransferCubeIntakePosition);
            }
          }
          case TRANSFER: { //when current arm state = TRANSFER
            switch (cargoType) {
              case CONE:
                return m_arm.setForwardKinematicsCommand(ArmConstants.kTransferConeIntakePosition);
              case CUBE:
                return m_arm.TransferToTransfer(ArmConstants.kTransferCubeIntakePosition);
            }
          }
          case FRONT: { //when current arm state = FRONT
            switch (cargoType) {
              case CONE:
                return m_arm.FrontToTransfer(ArmConstants.kTransferConeIntakePosition);
              case CUBE:
                return m_arm.FrontToTransfer(ArmConstants.kTransferCubeIntakePosition);
            }
          }
          case STOW: { //when current arm state = STOW
            switch (cargoType) {
              case CONE:
                return m_arm.StowToTransfer(ArmConstants.kTransferConeIntakePosition);
              case CUBE:
                return m_arm.StowToTransfer(ArmConstants.kTransferCubeIntakePosition);
            }
          }
        }
      }
      case FRONT: { // when target arm state = FRONT
        switch (currentArmState) { 
          case BACK: { // when current state is back
            switch (cargoType) { 
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.BackToFront(ArmConstants.kFrontConeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.BackToFront(ArmConstants.kFrontIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.BackToFront(ArmConstants.kFrontCubeL3Position);
                  case TWO:
                    return m_arm.BackToFront(ArmConstants.kFrontCubeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.BackToFront(ArmConstants.kFrontIntakePosition);
                }
              }
            }
          }
          case TRANSFER: { // when current state is transfer
            switch (cargoType) { 
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.TransferToFront(ArmConstants.kFrontConeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.TransferToFront(ArmConstants.kFrontIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.TransferToFront(ArmConstants.kFrontCubeL3Position);
                  case TWO:
                    return m_arm.TransferToFront(ArmConstants.kFrontCubeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.TransferToFront(ArmConstants.kFrontIntakePosition);
                }
              }
            }
          }
          case FRONT: {
            switch (cargoType) {
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.FrontToFront(ArmConstants.kFrontConeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.FrontToFront(ArmConstants.kFrontIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.FrontToFront(ArmConstants.kFrontCubeL3Position);
                  case TWO:
                    return m_arm.FrontToFront(ArmConstants.kFrontCubeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.FrontToFront(ArmConstants.kFrontIntakePosition);
                }
              }
            }
          }
          case STOW: {
            switch (cargoType) {
              case CONE: {
                switch (armScoreLevel) {
                  case THREE:
                    return nothingCommand();
                  case TWO:
                    return m_arm.StowToFront(ArmConstants.kFrontConeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.StowToFront(ArmConstants.kFrontIntakePosition);
                }
              }
              case CUBE: {
                switch (armScoreLevel) {
                  case THREE:
                    return m_arm.StowToFront(ArmConstants.kFrontCubeL3Position);
                  case TWO:
                    return m_arm.StowToFront(ArmConstants.kFrontCubeL2Position);
                  case ONE:
                    return nothingCommand();
                  case INTAKE:
                    return m_arm.StowToFront(ArmConstants.kFrontIntakePosition);
                }
              }
            }
          }
        }
      }
      case STOW: {
        switch (currentArmState) {
          case BACK:
            return m_arm.BackToStow();
          case TRANSFER:
           return m_arm.TransferToStow();
          case FRONT:
            return m_arm.FrontToStow();
          case STOW:
            return nothingCommand();
        }
      }
    }
    return nothingCommand();
  }
  
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    SmartDashboard.putString("Target Arm State", targetArmState.toString());
    SmartDashboard.putString("Current Arm State", currentArmState.toString());
  }
}
