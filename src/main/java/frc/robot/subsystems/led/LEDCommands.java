package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.led.LEDSubsystem.LEDMode;

public class LEDCommands {
    private LEDSubsystem m_led;
    public LEDCommands(LEDSubsystem led) {
        this.m_led = led;
    }

    public Command setLEDMode(LEDMode mode) {
        return m_led.runOnce(() -> {
            m_led.setLEDMode(mode);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
}
