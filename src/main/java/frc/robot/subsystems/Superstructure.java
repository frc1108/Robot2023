// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.SliderConstants;

public class Superstructure extends Subsystem {
  private final ArmSubsystem m_arm;
  private final ExtenderSubsystem m_slide;
  private final ElevatorSubsystem m_elevator;
  private final ClawSubsystem m_claw;

  /** Creates a new Superstructure. */
  public Superstructure(ArmSubsystem arm, ExtenderSubsystem slide,
                        ElevatorSubsystem elevator, ClawSubsystem claw) {
  m_arm = arm;
  m_slide = slide;
  m_elevator = elevator;
  m_claw = claw;
  }

// 1
  public Command scoreCubeAutoCommand(){
    return Commands.sequence(
      Commands.print("Score auto cube"),
      m_claw.gripCommand(),
      Commands.waitSeconds(0.25),
      m_elevator.upCommand(),
      Commands.waitSeconds(1.35),
      m_arm.setArmGoalCommand(ArmConstants.kArmHighCubeOffsetRads),
      Commands.waitSeconds(0.3),
      m_slide.setSliderGoalCommand(SliderConstants.kSliderHighCubeMeters),
      Commands.waitSeconds(1),
      m_claw.releaseCommand(),
      Commands.waitSeconds(0.1),
      m_slide.setSliderGoalCommand(SliderConstants.kSliderStowMeters),
      Commands.waitSeconds(1),
      m_elevator.downCommand(),
      Commands.waitSeconds(0.1),
      m_arm.setArmGoalCommand(ArmConstants.kArmOffsetRads+Units.degreesToRadians(15))
    );
  }

// 6  
  public Command overTopSubstationCommand(){
    return Commands.none();
  }

// 2
  public Command floorPickupCommand(){
    return Commands.none();
  }

// 5
  public Command highGoalCommand(){
    return Commands.none();
  }

// 4 
  public Command midGoalCommand(){
    return Commands.none();
  }

// 3
  public Command lowGoalCommand(){
    return Commands.none();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
