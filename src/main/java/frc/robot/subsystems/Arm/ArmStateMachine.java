// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmStateMachine extends SubsystemBase {
  private Arm m_arm;
  
  public enum ArmState {
    BACK, TRANSFER, FRONT
  }

  public enum ScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE
  }

  public ArmState currentArmState = ArmState.TRANSFER; //will default to TRANSFER 
  //m_arm.estimateCurrentArmState()??
  public ScoreLevel currentScoreLevel = ScoreLevel.THREE; //default to THREE

  public ArmState targetArmState = ArmState.TRANSFER; // default to TRANSFER 
  public ScoreLevel targetScoreLevel = ScoreLevel.THREE; // default to THREE

  private boolean armStateChanges = false; //default to false
  private boolean scoreLevelChanges = false; //default to false

  /** Creates a new StateMachine. */
  public ArmStateMachine(Arm m_arm) {
    this.m_arm = m_arm;
  }
  
  private void setTargetArmState(ArmState targetArmState) {
    armStateChanges = this.targetArmState != targetArmState;

    currentArmState = this.targetArmState;
    this.targetArmState = targetArmState;
  }

  private void setTargetScoreLevel(ScoreLevel targetScoreLevel) {
    scoreLevelChanges = this.targetScoreLevel != targetScoreLevel;

    currentScoreLevel = this.targetScoreLevel;
    this.targetScoreLevel = targetScoreLevel;
  }

  public Command setTargetStateCommand(ArmState targetArmState, ScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setTargetArmState(targetArmState)).alongWith(
      new InstantCommand(() -> setTargetScoreLevel(targetScoreLevel)));
  }

  public Command setTargetStateCommand(ArmState targetArmState) {
    return new InstantCommand(() -> setTargetArmState(targetArmState));
  }

  public Command setTargetStateCommand(ScoreLevel targetScoreLevel) {
    return new InstantCommand(() -> setTargetScoreLevel(targetScoreLevel));
  }

  public Command getArmCommand() {
    if(armStateChanges) { // if arm state changes
      switch(targetArmState) {
        case TRANSFER: // when target arm state = TRANSFER
          switch(currentArmState) {
            case BACK: return m_arm.backToTransfer();
            //case FRONT: return m_arm.frontToTransfer();
          }
        case BACK: // when target arm state = BACK
          switch(currentArmState) {
            case TRANSFER:
              switch(targetScoreLevel) {
                case THREE: return m_arm.transferToBack(m_arm.levelThreeCommand());
                case TWO: return m_arm.transferToBack(m_arm.levelTwoCommand());
                //case ONE: return m_arm.transferToBack(m_arm.backLevelOneCommand());
                //case INTAKE: return m_arm.transferToBack(m_arm.backIntakeCommand());
              }
            case FRONT:
              switch(targetScoreLevel) {
                //case THREE: return m_arm.frontToBack(m_arm.backLevelThreeCommand());
                //case TWO: return m_arm.frontToBack(m_arm.backLevelTwoCommand());
              }
          }
        case FRONT: // when target arm state = FRONT
          switch(currentArmState) {
            //case BACK: return m_arm.backToFront();
            //case TRANSFER: return m_arm.transferToFront();
          }
      }
    } else if(currentArmState == ArmState.BACK && !armStateChanges && scoreLevelChanges) { // when arm state = BACK does not change + score level changes
      switch(targetScoreLevel) {
        case THREE: return m_arm.levelThreeCommand();
        case TWO: return m_arm.levelTwoCommand();
        // case ONE: return m_arm.backLevelOne();
        //case INTAKE: return m_arm.backIntake();
      }
    }
    return null; // if neither arm state or score level change, do nothing
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Target Arm State", targetArmState.toString());
    SmartDashboard.putString("Current Arm State", currentArmState.toString());
    
    SmartDashboard.putString("Target Score Level", targetScoreLevel.toString());
    SmartDashboard.putString("Current Score Level", currentScoreLevel.toString());

    SmartDashboard.putBoolean("Arm State Changes?", armStateChanges);
    SmartDashboard.putBoolean("Score Level Changes?", scoreLevelChanges);
  }
}
