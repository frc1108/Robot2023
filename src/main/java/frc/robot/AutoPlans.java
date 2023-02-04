// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

/** The AutoPlans class contains command factories for
 * autonomous trajectories using PathPlanner with
 * event mapping to for robot actions. */
public class AutoPlans {

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static CommandBase examplePathCommand(DriveSubsystem drive) {

    PathPlannerTrajectory examplePath = PathPlanner
        .loadPath(
           "Example Path",
           new PathConstraints(
               AutoConstants.kMaxSpeedMetersPerSecond,
               AutoConstants.kMaxAccelerationMetersPerSecondSquared));

    return drive.followTrajectoryCommand(examplePath, true)
                       .andThen(() -> drive.drive(0, 0, 0, false, false));
  }

}
