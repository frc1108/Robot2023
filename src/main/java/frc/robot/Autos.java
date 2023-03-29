// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Superstructure;

@SuppressWarnings("unused")
public final class Autos {

  private final DriveSubsystem m_swerve;
  private final Superstructure m_superS;
  private HashMap<String, Command> eventMap;
  private final SwerveAutoBuilder autoBuilder;

  // Autonomous selector on dashboard
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private GenericEntry kAutoStartDelaySeconds;
  
  public Autos(DriveSubsystem swerve,Superstructure superS) {
    this.m_swerve = swerve;
    this.m_superS = superS;

    eventMap = new HashMap<>();
    setMarkers();

    autoBuilder =
      new SwerveAutoBuilder(
        m_swerve::getPose,
        m_swerve::resetOdometry,
        DriveConstants.kDriveKinematics,
        new PIDConstants(AutoConstants.kPXController, 0.0, 0.0),
        new PIDConstants(AutoConstants.kPThetaController, 0, 0),
        m_swerve::setModuleStates,
        eventMap,
        m_swerve);

    // Autonomous selector options
    kAutoStartDelaySeconds = Shuffleboard.getTab("Live")
                                         .add("Auto Delay", 0)
                                         .withWidget(BuiltInWidgets.kNumberSlider)
                                         .withProperties((Map.of("Min", 0, "Max", 10, "Block increment", 1)))
                                         .getEntry();
    
    autoChooser.setDefaultOption("Nothing", Commands.none());
    autoChooser.addOption("CubeBalance", cubeBB());
    autoChooser.addOption("SpeedBump",speedBump());
    autoChooser.addOption("AutoBalance",m_swerve.autoBalance());
    autoChooser.addOption("High Cube",m_superS.scoreCubeAutoCommand());
    autoChooser.addOption("Center Cube", cubeCenter());

    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  private void setMarkers() {
    eventMap = buildEventMap();
  }

    public CommandBase none() {
      return Commands.none();
    }
  
    public CommandBase example() {
      return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Example Path",
                                  new PathConstraints(4, 3)));
    }
    public CommandBase cubeBB() {
      return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Cube Balance",
                                  new PathConstraints(4, 3)));
    }
    
    public CommandBase speedBump() {
      return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Speedbump",
      new PathConstraints(4, 3)));      
    }

    public CommandBase cubeCenter() {
      return m_superS.scoreCubeAutoCommand().andThen(m_swerve.autoBalance());
    }
  
    private HashMap<String, Command> buildEventMap() {
      return new HashMap<>(
          Map.ofEntries(
              Map.entry("scoreCubeHigh", m_superS.scoreCubeAutoCommand().alongWith(Commands.print("Cube Score"))),
              //Map.entry("scoreCubeHigh", Commands.print("Cube Score")),
              //Commands.print("Cube Score")),
              Map.entry("autoBalance", m_swerve.autoBalance())));
              // Commands.print("Auto Balance"))));
    }

      /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
      Commands.waitSeconds(kAutoStartDelaySeconds.getDouble(0)),
      autoChooser.getSelected());
  }

  public void resetAutoHeading() {
    m_swerve.zeroHeading();
  }
  }