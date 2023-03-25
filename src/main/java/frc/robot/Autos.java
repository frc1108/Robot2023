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
  private final SendableChooser<Command> autoChooser;
  private HashMap<String, Command> eventMap;
  private final SwerveAutoBuilder autoBuilder;
  
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

    autoChooser = new SendableChooser<Command>();
    autoChooser.setDefaultOption("None", none());

    SmartDashboard.putData("Auto Chooser",autoChooser);
  }

  private void setMarkers() {
    eventMap = buildEventMap();
  }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
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
              //Map.entry("scoreCubeHigh", m_superS.scoreCubeAutoCommand().alongWith(Commands.print("Cube Score"))),
              Map.entry("scoreCubeHigh", Commands.print("Cube Score")),
              //Commands.print("Cube Score")),
              Map.entry("autoBalance", m_swerve.autoBalance())));
              // Commands.print("Auto Balance"))));
    }
  }