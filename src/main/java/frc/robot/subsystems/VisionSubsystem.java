// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;

public class VisionSubsystem extends SubsystemBase implements Loggable {

  public PhotonCamera m_driverCamera = new PhotonCamera("driverCamera");
  public PhotonCamera m_tagCamera = new PhotonCamera("tagCamera");

  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    m_driverCamera.setDriverMode(true);
    m_tagCamera.setDriverMode(false);
    m_tagCamera.setPipelineIndex(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
