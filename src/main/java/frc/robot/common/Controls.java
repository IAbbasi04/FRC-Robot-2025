package frc.robot.common;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class Controls {
    public static Trigger SNAIL_MODE = new Trigger(() -> false);
    public static Trigger INTAKE = new Trigger(() -> false);
    public static Trigger OUTAKE = new Trigger(() -> false);
    public static Trigger PRIME_SUBWOOFER = new Trigger(() -> false);
    public static Trigger PRIME_PODIUM = new Trigger(() -> false);
    public static Trigger PRIME_RANGE = new Trigger(() -> false);
    public static Trigger STOW = new Trigger(() -> false);
    public static Trigger PRIME_AMP = new Trigger(() -> false);
    public static Trigger SHOOT = new Trigger(() -> false);
    public static Trigger SPEAKER_LOCK = new Trigger(() -> false);

    private static SmartLogger logger;

    private static boolean logToShuffleboard = false;
    private static boolean loggingEnabled = false;

    public static void singleDriverControls() {
        OUTAKE = RobotContainer.getDriverController().leftBumper();
        INTAKE = RobotContainer.getDriverController().leftTrigger();

        PRIME_SUBWOOFER = RobotContainer.getDriverController().x();
        PRIME_AMP = RobotContainer.getDriverController().y();
        STOW = RobotContainer.getDriverController().a();
        PRIME_PODIUM = RobotContainer.getDriverController().b();

        PRIME_RANGE = RobotContainer.getDriverController().rightBumper(); // On press
        SPEAKER_LOCK = RobotContainer.getDriverController().rightBumper(); // On hold

        SHOOT = RobotContainer.getDriverController().rightTrigger();

        SNAIL_MODE = RobotContainer.getDriverController().leftStick();
    }

    public static void dualDriverControls() {
        SHOOT = RobotContainer.getDriverController().rightTrigger();

        OUTAKE = RobotContainer.getOperatorController().leftBumper();
        INTAKE = RobotContainer.getOperatorController().leftTrigger();

        PRIME_AMP = RobotContainer.getOperatorController().x();
        
        PRIME_RANGE = RobotContainer.getOperatorController().rightBumper();
        PRIME_SUBWOOFER = RobotContainer.getOperatorController().rightTrigger();
    }

    public static void initializeShuffleboardLogs(boolean logToShuffleboard) {
        Controls.logToShuffleboard = logToShuffleboard;
        if (!logToShuffleboard) return;
        
        Controls.loggingEnabled = true;
        Controls.logger = new SmartLogger("Controls");
        Controls.logger.enable();

    }

    public static void logControlsToShuffleboard() {
        if (!Controls.logToShuffleboard) return; // Don't log if we don't want to log
        
        if (!Controls.loggingEnabled) { // If not already enabled, enable it
            initializeShuffleboardLogs(true);
        }

        for (Field field : Controls.class.getDeclaredFields()) {
            try {
                logger.logBoolean(field.getName(), ((Trigger)field.get(null)).getAsBoolean());
            } catch (Exception e) {}
        }
    }
}