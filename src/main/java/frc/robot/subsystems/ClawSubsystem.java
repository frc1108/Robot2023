// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.REVPHConstants;

public class ClawSubsystem extends SubsystemBase {

  private final DoubleSolenoid m_piston = new DoubleSolenoid(
    PneumaticsModuleType.REVPH,
    REVPHConstants.kForwardClaw,
    REVPHConstants.kReverseClaw);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    m_piston.set(Value.kReverse);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CommandBase gripCommand() {
    return runOnce(() -> m_piston.set(Value.kForward)).withName("Claw Closed");
  }

  public CommandBase releaseCommand() {
    return runOnce(() -> m_piston.set(Value.kReverse)).withName("Claw Open");
  }
}
