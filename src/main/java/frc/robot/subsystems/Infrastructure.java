// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.InfrastructureConstants;

public class Infrastructure extends SubsystemBase {
  private final PneumaticHub m_pneumaticHub;
  private final PowerDistribution m_powerDistributionHub;

  /** Creates a new Infrastructure. */
  public Infrastructure() {
    m_pneumaticHub = new PneumaticHub(InfrastructureConstants.kPneumaticHubCanId);
    m_powerDistributionHub = new PowerDistribution(InfrastructureConstants.kPowerDistributionHubCanId, ModuleType.kRev);

    enableCompressor();
  }

  public void enableCompressor() {
    m_pneumaticHub.enableCompressorAnalog(InfrastructureConstants.kCompressorMinPressure, InfrastructureConstants.kCompressorMaxPressure);
  }

  public void disableCompressor() {
    m_pneumaticHub.disableCompressor();
  }

  public boolean isCompressorRunning() {
    return m_pneumaticHub.getCompressor();
  }

  public double getPressure() {
    return m_pneumaticHub.getPressure(0); //0 = analog channel
  }

  public double getVoltage() {
    return m_powerDistributionHub.getVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pneumatic Pressure", getPressure());
    SmartDashboard.putBoolean("Compressor Running?", isCompressorRunning());
  }
}
