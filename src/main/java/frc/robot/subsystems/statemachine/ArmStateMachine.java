// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm;

public class ArmStateMachine extends SubsystemBase {
  private Arm m_arm;
  
  public enum ArmState {
    SCORE, TRANSFER, INTAKE_BACK, INTAKE_FRONT
  }

  public enum ScoreLevel {
    THREE, TWO, ONE, FRONT
  }

  public enum CargoType {
    CONE, CUBE
  }

  public ArmState currentArmState;
  public ScoreLevel currentScoreLevel;

  public ArmState targetArmState;
  public ScoreLevel targetScoreLevel;

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm) {
    this.m_arm = m_arm;
  }
  
  public void setTargetArmState(ArmState targetArmState) {
    currentArmState = this.targetArmState;
    this.targetArmState = targetArmState;
    //if (targetArmState != ArmState.SCORE) {targetScoreLevel = ScoreLevel.NONE;}
  }

  public void setTargetScoreLevel(ScoreLevel targetScoreLevel) {
    currentScoreLevel = this.targetScoreLevel;
    this.targetScoreLevel = targetScoreLevel;
  }

  public void setTargetState(ArmState targetArmState, ScoreLevel targetScoreLevel) {
    setTargetArmState(targetArmState);
    setTargetScoreLevel(targetScoreLevel);
  }

  private boolean armStateChanges() {
    return targetArmState == currentArmState;
  }

  private boolean scoreLevelChanges() {
    return targetScoreLevel == currentScoreLevel;
  }

  public Command getArmCommand() {
    if(armStateChanges()) {
      switch(targetArmState) {
        case TRANSFER:
          switch(currentArmState) {
            case SCORE: return m_arm.scoreToTransfer();
            //case INTAKE_BACK: return m_arm.intakeFrontToTransfer();
          }
        case SCORE:
          switch(currentArmState) {
            case TRANSFER:
              switch(targetScoreLevel) {
                case THREE: return m_arm.transferToLevelThree();
                case TWO: return m_arm.transferToLevelTwo();
                //case ONE: return m_arm.transferToLevelOne();
              }
            case SCORE:
              switch(targetScoreLevel) {
                case THREE: return m_arm.scoreConeLevelThree();
                case TWO: return m_arm.scoreConeLevelTwo();
                // case ONE: return m_arm.scoreConeLevelOne();
              }
          }
        case INTAKE_BACK: {
          
        }
      }
    }
    /* 
    if (armStateChanges() && targetArmState == ArmState.TRANSFER) {
      switch(currentArmState) {
        case SCORE: return m_arm.scoreToTransfer();
        //case INTAKE_BACK: return m_arm.intakeFrontToTransfer();
      }
    } else if (armStateChanges() && targetArmState == ArmState.SCORE) {
      if (currentArmState == ArmState.TRANSFER) {
        switch(targetScoreLevel) {
          case THREE: return m_arm.transferToLevelThree();
          case TWO: return m_arm.transferToLevelTwo();
          //case ONE: return m_arm.transferToLevelOne();
        }
      } else if (currentArmState == ArmState.SCORE) {
        switch(targetScoreLevel) {
          case THREE: return m_arm.scoreConeLevelThree();
          case TWO: return m_arm.scoreConeLevelTwo();
          // case ONE: return m_arm.scoreConeLevelOne();
        }
      } /*else if (currentArmState == ArmState.INTAKE_FRONT) {
      } else if (currentArmState == ArmState.INTAKE_BACK) {
      }/* */
     /* else if (armStateChanges() && targetArmState == ArmState.INTAKE_FRONT) {
      if (currentArmState == ArmState.SCORE) {}
    }*/

    return null;
  }
  





  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
