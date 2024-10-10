// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.common.Controls;
import frc.robot.common.MatchMode;
import frc.robot.common.Utils;
import frc.robot.common.crescendo.tables.ShotTable;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  public static Field2d FIELD = new Field2d();
  public static MatchMode MODE = MatchMode.DISABLED;
  public static ShotTable SHOT_TABLE = new ShotTable();
  public static Timer TIMER = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(true);
    Logger.recordMetadata("Game", "Crescendo");
    Logger.recordMetadata("Year", "2024");
    Logger.recordMetadata("Robot", "Zenith");
    Logger.recordMetadata("Team", "8592");

    if (isReal()) { // If running on a real robot
        String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
        String path = "/U/"+time+".wpilog";
        Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick
        Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        Logger.start();
    }
    else { // If simulated
        SmartDashboard.putData(FIELD);
        FIELD.setRobotPose(new Pose2d());
    }
  }

  @Override
  public void robotPeriodic() {
    TIMER.start();

    CommandScheduler.getInstance().run();
    m_robotContainer.logData();
    Controls.logControlsToShuffleboard();
  }

  @Override
  public void disabledInit() {
    MODE = MatchMode.DISABLED;
    m_robotContainer.onInit();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    MODE = MatchMode.AUTONOMOUS;

    m_robotContainer.onInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      SwerveSubsystem.getInstance().setStartPose(
        Utils.flipPoseToRed(m_robotContainer.getStartPose(), 
          DriverStation.getAlliance().get() == Alliance.Red
        )
      );
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    MODE = MatchMode.TELEOP;
    m_robotContainer.onInit();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    MODE = MatchMode.TEST;
    m_robotContainer.onInit();

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
