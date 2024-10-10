package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LEDSubsystem.LEDMode;

public class LEDCommands {
    private LEDSubsystem m_led;
    public LEDCommands(LEDSubsystem led) {
        this.m_led = led;
    }

    public Command setLEDMode(LEDMode mode) {
        return m_led.run(() -> {
            m_led.setLEDMode(mode);
        });
    }
}
