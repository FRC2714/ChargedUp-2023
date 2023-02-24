// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin extends SubsystemBase {
  private Spark LED;

  /** Creates a new Blinkin. */
  public Blinkin() {
    LED = new Spark(LEDConstants.kLEDPort);
  }

  private void set(double val) {
    LED.set(val);
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
