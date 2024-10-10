package frc.robot.common;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;

public class Controls {
    public static Trigger INTAKE = new Trigger(() -> false);
    public static Trigger OUTAKE = new Trigger(() -> false);
    public static Trigger PRIME_SUBWOOFER = new Trigger(() -> false);
    public static Trigger PRIME_PODIUM = new Trigger(() -> false);
    public static Trigger PRIME_RANGE = new Trigger(() -> false);
    public static Trigger STOW = new Trigger(() -> false);
    public static Trigger PRIME_AMP = new Trigger(() -> false);
    public static Trigger SHOOT = new Trigger(() -> false);
    public static Trigger SPEAKER_LOCK = new Trigger(() -> false);

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

    }

    public static void dualDriverControls() {
        SHOOT = RobotContainer.getDriverController().rightTrigger();

        OUTAKE = RobotContainer.getOperatorController().leftBumper();
        INTAKE = RobotContainer.getOperatorController().leftTrigger();

        PRIME_AMP = RobotContainer.getOperatorController().x();
        
        PRIME_RANGE = RobotContainer.getOperatorController().rightBumper();
        PRIME_SUBWOOFER = RobotContainer.getOperatorController().rightTrigger();
    }

    public static void programmingTestControls() {
        
    }
}