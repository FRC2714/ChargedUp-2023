// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class ArmStateMachine extends SubsystemBase {
  private Arm m_arm;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT
  }

  public enum ScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE, NONE
  }

  public ArmState currentArmState = ArmState.BACK; //will default to TRANSFER 
  //m_arm.estimateCurrentArmState()??
  public ScoreLevel currentScoreLevel = ScoreLevel.THREE; //default to THREE

  public ArmState targetArmState = ArmState.BACK; // default to TRANSFER 
  public ScoreLevel targetScoreLevel = ScoreLevel.THREE; // default to THREE

  public CargoType cargoType = CargoType.CONE; // default of cube

  private boolean armStateChanges = false; //default to false
  private boolean scoreLevelChanges = false; //default to false

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm) {
    this.m_arm = m_arm;
  }

  private void setTargetState(ArmState targetArmState, ScoreLevel targetScoreLevel) {
    armStateChanges = this.targetArmState != targetArmState;
    currentArmState = this.targetArmState;
    this.targetArmState = targetArmState;
    
    scoreLevelChanges = this.targetScoreLevel != targetScoreLevel;
    currentScoreLevel = this.targetScoreLevel;
    this.targetScoreLevel = targetScoreLevel;

    getArmCommand().withInterruptBehavior(InterruptionBehavior.kCancelIncoming).schedule();
  }

  private void setCargoType(CargoType cargoType) {
    this.cargoType = cargoType;
  }

  public CargoType getCargoType() {
    return cargoType;
  }

  public Command setTargetStateCommand(ArmState targetArmState, ScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setTargetState(targetArmState, targetScoreLevel));
  }

  public Command setCargoTypeCommand(CargoType cargoType) {
    return new InstantCommand(() -> setCargoType(cargoType));
  }

  public Command getArmCommand() {
    if(armStateChanges) { // if arm state changes
      switch(targetArmState) {
        case TRANSFER: {// when target arm state = TRANSFER
          switch(currentArmState) {
            case BACK: return m_arm.BackToTransfer();
            case FRONT: return m_arm.FrontToTransfer();
          }
        }
        case BACK: // when target arm state = BACK
          switch(currentArmState) {
            case TRANSFER: { // When current is TRANSFER
              switch (cargoType) {
                case CONE: switch(targetScoreLevel) {
                  case THREE: return m_arm.TransferToBack(m_arm.ConeL3Command());
                  case TWO: return m_arm.TransferToBack(m_arm.ConeL2Command());
                  case ONE: return m_arm.TransferToBack(m_arm.ConeL1Command());
                  case INTAKE: return m_arm.TransferToBack(m_arm.BackIntakeCommand());
                }
                case CUBE: switch(targetScoreLevel) {
                  case THREE: return m_arm.TransferToBack(m_arm.CubeL3Command());
                  case TWO: return m_arm.TransferToBack(m_arm.CubeL2Command());
                  case ONE: return m_arm.TransferToBack(m_arm.CubeL1Command());
                  case INTAKE: return m_arm.TransferToBack(m_arm.BackIntakeCommand());
                }
              }
            }
            case FRONT: { //when current is FRONT
              switch (cargoType) {
                case CONE: switch(targetScoreLevel) {
                  case THREE: return m_arm.FrontToBack(m_arm.ConeL3Command());
                  case TWO: return m_arm.FrontToBack(m_arm.ConeL2Command());
                  case ONE: return m_arm.FrontToBack(m_arm.ConeL1Command());
                  case INTAKE: return m_arm.FrontToBack(m_arm.BackIntakeCommand());
                }
                case CUBE: switch(targetScoreLevel) {
                  case THREE: return m_arm.FrontToBack(m_arm.CubeL3Command());
                  case TWO: return m_arm.FrontToBack(m_arm.CubeL2Command());
                  case ONE: return m_arm.FrontToBack(m_arm.CubeL1Command());
                  case INTAKE: return m_arm.FrontToBack(m_arm.BackIntakeCommand());
                }
              }
            }
          }
        case FRONT: {// when target arm state = FRONT
          switch(currentArmState) {
            case BACK: return m_arm.BackToFront();
            case TRANSFER: return m_arm.TransferToFront();
          }
        }
      }
    } else if(!armStateChanges && currentArmState == ArmState.BACK && scoreLevelChanges) { // when arm state does not change and = BACK + score level changes
      switch (cargoType) {
        case CONE: {
          switch(targetScoreLevel) {
            case THREE: return m_arm.BackToBack(m_arm.ConeL3Command());
            case TWO: return m_arm.BackToBack(m_arm.ConeL2Command());
            case ONE: return m_arm.BackToBack(m_arm.ConeL1Command());
            case INTAKE: return m_arm.BackToBack(m_arm.BackIntakeCommand());
          }
        }
        case CUBE: {
          switch(targetScoreLevel) {
            case THREE: return m_arm.BackToBack(m_arm.CubeL3Command());
            case TWO: return m_arm.BackToBack(m_arm.CubeL2Command());
            case ONE: return m_arm.BackToBack(m_arm.CubeL1Command());
            case INTAKE: return m_arm.BackToBack(m_arm.BackIntakeCommand());
          }
        }
      }
    }
    return new PrintCommand("nothing"); // if neither arm state or score level change, do nothing
  }
  
  @Override
  public void periodic() {
    //This method will be called once per scheduler run
    SmartDashboard.putString("Target Arm State", targetArmState.toString());
    SmartDashboard.putString("Current Arm State", currentArmState.toString());
    
    SmartDashboard.putString("Target Score Level", targetScoreLevel.toString());
    SmartDashboard.putString("Current Score Level", currentScoreLevel.toString());

    SmartDashboard.putString("CargoType", cargoType.toString());

    SmartDashboard.putBoolean("Arm State Changes?", armStateChanges);
    SmartDashboard.putBoolean("Score Level Changes?", scoreLevelChanges);
  }
}
