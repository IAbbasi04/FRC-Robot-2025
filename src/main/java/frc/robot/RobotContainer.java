// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commands.*;
import frc.robot.common.*;
import frc.robot.common.DriveScaler.ScaleType;
import frc.robot.subsystems.*;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem.FeederState;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.LEDSubsystem.LEDMode;
import frc.robot.subsystems.power.PowerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private SubsystemList m_runningSubsystems; // A list of all subsystems that are running on the robot
  private AutonomousSelector m_autoSelector; // Selects which autonomous we want to run
  private static CommandXboxController driverController, operatorController; // The driver and operator controllers for teleop

  private DoubleSupplier translateX = () -> -driverController.getLeftX();
  private DoubleSupplier translateY = () -> -driverController.getLeftY();
  private DoubleSupplier rotate = () -> -driverController.getRightX();

  private DriveScaler xScaler = new DriveScaler(
      ScaleType.LINEAR, true
    ).withSlewLimit(Constants.INPUT.SLEW_RATE_TRANSLATE);

  private DriveScaler yScaler = new DriveScaler(
      ScaleType.LINEAR, true
    ).withSlewLimit(Constants.INPUT.SLEW_RATE_TRANSLATE);

  private DriveScaler rotateScaler = new DriveScaler(
      ScaleType.LINEAR, true
    );//.withSlewLimit(Constants.INPUT.SLEW_RATE_ROTATE);

  public RobotContainer(boolean logToShuffleboard) {
    this.m_runningSubsystems = new SubsystemList(
      // Add all running subsystems here
      PowerSubsystem.getInstance(),
      VisionSubsystem.getInstance(),
      SwerveSubsystem.getInstance(),
      LEDSubsystem.getInstance(),
      IntakeSubsystem.getInstance(),
      ShooterSubsystem.getInstance(),
      ElevatorSubsystem.getInstance(),
      FeederSubsystem.getInstance()
    );

    boolean enableShuffleboardLogging = (Robot.isSimulation() || !DriverStation.isFMSAttached()) && logToShuffleboard;
    this.m_runningSubsystems.initializeLoggers(enableShuffleboardLogging);
    this.m_runningSubsystems.enableShuffleboardLogging(enableShuffleboardLogging);
    this.m_autoSelector = new AutonomousSelector();

    this.setDefaults();
    this.configureBindings();
    this.registerNamedCommands();
    RobotContainer.initializeAutoBuilder();
    Controls.initializeShuffleboardLogs(logToShuffleboard);
  }

  /**
   * The driver controller as a command-usable controller
   */
  public static CommandXboxController getDriverController() {
    return driverController;
  }

  /**
   * The operator controller as a command-usable controller
   */
  public static CommandXboxController getOperatorController() {
    return operatorController;
  }

  /**
   * Prepares the auto builder for autonomous and command creation
   */
  public static void initializeAutoBuilder() {
    AutoBuilder.configureHolonomic(
        SwerveSubsystem.getInstance()::getCurrentPose, 
        SwerveSubsystem.getInstance()::setStartPose,
        SwerveSubsystem.getInstance()::getCurrentSpeeds, 
        SwerveSubsystem.getInstance()::driveFieldCentric,
        new HolonomicPathFollowerConfig(
            Constants.SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
            Constants.SWERVE.DRIVE_TRAIN_RADIUS,
            new ReplanningConfig()
        ),
        () -> {
            if (DriverStation.getAlliance().isPresent()) {
                return DriverStation.getAlliance().get() == Alliance.Red;
            }
            return false; // Never flip if driver station does not display alliance color
        },
        SwerveSubsystem.getInstance()
    );
  }

  /**
   * Sets up the controls to map to commands or actions
   */
  private void configureBindings() {
    driverController = new CommandXboxController(Constants.INPUT.DRIVER_CONTROLLER_PORT);
    operatorController = new CommandXboxController(Constants.INPUT.OPERATOR_CONTROLLER_PORT);

    // Add which controls set we want to use
    Controls.singleDriverControls();

    // Apply controls here
    Controls.SNAIL_MODE.whileTrue(
      SwerveSubsystem.getInstance().getCommands().setSnail(true)
    ).whileFalse(
      SwerveSubsystem.getInstance().getCommands().setSnail(false)
    );

    Controls.STOW.onTrue(new StowCommand());

    Controls.OUTAKE.whileTrue(
      new OutakeCommand()
    ).onFalse(
      FeederSubsystem.getInstance().getCommands().setFeederState(FeederState.kOff)
      .alongWith(IntakeSubsystem.getInstance().getCommands().setIntakeVelocity(() -> 0d))
    );

    Controls.INTAKE.whileTrue(
      new IntakeCommand()
    ).onFalse(
      FeederSubsystem.getInstance().getCommands().setFeederState(FeederState.kOff)
      .alongWith(IntakeSubsystem.getInstance().getCommands().setIntakeVelocity(() -> 0d))
    );

    Controls.SHOOT.whileTrue(
      new SendToShooterCommand()
    ).onFalse(
      new StowCommand()
    );

    Controls.PRIME_SUBWOOFER.onTrue(
      new PrimeCommand(Robot.SHOT_TABLE.getSubwooferShot())
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    Controls.PRIME_RANGE.onTrue(
      new PrimeCommand(Robot.SHOT_TABLE.getShotFromDistance(
        VisionSubsystem.getInstance().getDistanceToSpeaker()
      )).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
  }

  /**
   * Registers all named commands to be used as events in PathPlanner
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Intake", new IntakeCommand());
    
    NamedCommands.registerCommand("ShootSubwoofer", 
      new SimulatedCommand(new ShootCommand(Robot.SHOT_TABLE.getSubwooferShot()), 1.0)
        .andThen(new StowCommand())
    );

    NamedCommands.registerCommand("PrimeSubwoofer", new PrimeCommand(Robot.SHOT_TABLE.getSubwooferShot()));
    
    NamedCommands.registerCommand("ShootRange", 
      new SimulatedCommand(new ShootCommand(Robot.SHOT_TABLE.getShotFromDistance(
        VisionSubsystem.getInstance().getDistanceToSpeaker()
      )), 1.0)
      // new WaitCommand(1.0)
    );

    NamedCommands.registerCommand("PrimeRange", 
      new PrimeCommand(Robot.SHOT_TABLE.getShotFromDistance(
        VisionSubsystem.getInstance().getDistanceToSpeaker()
      ))
    );
  }

  /**
   * Sets the default commands for all subsystems
   */
  private void setDefaults() {
    SwerveSubsystem.getInstance().setDefaultCommand( // Teleop Driving
      SwerveSubsystem.getInstance().getCommands().teleopDriveCommand(
        () -> xScaler.scale(translateX.getAsDouble()),
        () -> yScaler.scale(translateY.getAsDouble()),
        () -> rotateScaler.scale(rotate.getAsDouble())
      ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );

    LEDSubsystem.getInstance().setDefaultCommand( // Default LED mode
      LEDSubsystem.getInstance().getCommands().setLEDMode(LEDMode.kDisabled)
      .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
    );
  }

  /**
   * Grabs the created autonomous command
   */
  public Command getAutonomousCommand() {
    return m_autoSelector.getSelectedAutonomous().createAuto();
  }

  /**
   * Code for each subsystem ran to start of each mode
   */
  public void onInit() {
    m_runningSubsystems.onInitAll();
  }

  /**
   * The starting position of the robot on the field
   */
  public Pose2d getStartPose() {
    return m_autoSelector.getSelectedAutonomous().getStartPose();
  }
}