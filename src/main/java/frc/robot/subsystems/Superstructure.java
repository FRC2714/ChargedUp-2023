// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmStateMachine;
import frc.robot.subsystems.Shooter.ShooterStateMachine;

public class Superstructure extends SubsystemBase {
  public enum ScoreLevel {
    THREE, TWO, ONE, INTAKE
  }

  public enum CargoType {
    CONE, CUBE
  }

  public ScoreLevel scoreLevel = ScoreLevel.INTAKE;
  public CargoType cargoType = CargoType.CONE;

  /** Creates a new Superstructure. */
  public Superstructure(ArmStateMachine m_armStateMachine, ShooterStateMachine m_shooterStateMachine) {}

  public ScoreLevel getScoreLevel() {
    return this.scoreLevel;
  }

  public CargoType getCargoType() {
    return this.cargoType;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
