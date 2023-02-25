// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDs extends SubsystemBase {
  private Spark Blinkin;

  /** Creates a new Blinkin. */
  public LEDs() {
    Blinkin = new Spark(LEDConstants.kBlinkinPort);
  }

  private void set(double val) {
    Blinkin.set(val);
  }

  private void setPurple() {
    set(LEDConstants.kPurple);
  }

  private void setYellow() {
    set(LEDConstants.kYellow);
  }

  public Command setColorCargoType(CargoType cargoType) {
    if (cargoType == CargoType.CONE) {
      return new InstantCommand(() -> setYellow());
    } else {
      return new InstantCommand(() -> setPurple());
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
