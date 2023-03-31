// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LED extends SubsystemBase {
  private Spark blinkin;

  /** Creates a new LEDs. */
  public LED(int BlinkinPort) {
    blinkin = new Spark(BlinkinPort);
  }

  private void set(double val) {
    blinkin.set(val);
  }

  public void setPurple() {
    set(LEDConstants.kPurple);
  }

  public void setYellow() {
    set(LEDConstants.kYellow);
  }

  public void setGreen() {
    set(0.77);
  }

  public void setRed() {
    set(0.61);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
