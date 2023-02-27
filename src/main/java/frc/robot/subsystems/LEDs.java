// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;
import frc.robot.subsystems.Arm.ArmStateMachine.IntakeMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LEDs extends SubsystemBase {
  private Spark Blinkin;

  /** Creates a new Blinkin. */
  public LEDs() {
    Blinkin = new Spark(LEDConstants.kBlinkinPort);
  }

  private void set(double val) {
    SmartDashboard.putNumber("set val: ", val);
    Blinkin.set(val);
  }

  public void setPurpleWave() {
    set(LEDConstants.kPurpleWave);
  }

  public void setYellowWave() {
    set(LEDConstants.kYellowWave);
  }

  public void setPurple() {
    set(LEDConstants.kPurple);
  }

  public void setYellow() {
    set(LEDConstants.kYellow);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
