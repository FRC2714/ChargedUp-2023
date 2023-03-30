// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Superstructure.CargoType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDs extends SubsystemBase {
  private Spark m_armBlinkin;
  private Spark m_baseBlinkin;

  /** Creates a new LEDs. */
  public LEDs() {
    m_armBlinkin = new Spark(LEDConstants.kArmBlinkinPort);
    m_baseBlinkin = new Spark(LEDConstants.kBaseBlinkinPort);
  }

  private void set(double val) {
    m_armBlinkin.set(val);
    m_baseBlinkin.set(val);
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

  public Command setColorCargoType(CargoType cargoType) {
    return cargoType == CargoType.CONE ? new InstantCommand(() -> setYellow()) : new InstantCommand(() -> setPurple());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
