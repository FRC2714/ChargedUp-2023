// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.Arm.ArmStateMachine.CargoType;
import frc.robot.subsystems.Arm.ArmStateMachine.IntakeMode;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDs extends SubsystemBase {
  private Spark Blinkin;

  /** Creates a new Blinkin. */
  public LEDs() {
    Blinkin = new Spark(LEDConstants.kBlinkinPort);
  }

  private void set(double val) {
    System.out.println("val:" + val);
    Blinkin.set(val);
  }

  public void updateColor(CargoType cargoType, IntakeMode intakeMode) {
    System.out.println("intake mode" + intakeMode.toString());
    System.out.println("cargoType" + cargoType.toString());
    switch (intakeMode) {
      case HP: {
        switch(cargoType) {
          case CUBE: {set(LEDConstants.kPurpleWave);}
          case CONE: {set(LEDConstants.kYellowWave);}
        }
      }
      case FLOOR: {
        switch(cargoType) {
          case CUBE: {set(LEDConstants.kPurple);}
          case CONE: {set(LEDConstants.kYellow);}
        }
      }
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
