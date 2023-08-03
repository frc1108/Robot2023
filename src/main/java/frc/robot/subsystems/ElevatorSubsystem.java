// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.REVPHConstants;
import io.github.oblarg.oblog.Loggable;

public class ElevatorSubsystem extends Subsystem implements Loggable {

  private final DoubleSolenoid m_piston = new DoubleSolenoid(
                                            PneumaticsModuleType.REVPH,
                                            REVPHConstants.kForwardElevator,
                                            REVPHConstants.kReverseElevator);
  
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_piston.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command upCommand() {
    return runOnce(() -> m_piston.set(Value.kForward)).withName("Elevator Out");
  }

  public Command downCommand() {
    return runOnce(() -> m_piston.set(Value.kReverse)).withName("Elevator In");
  }
}
