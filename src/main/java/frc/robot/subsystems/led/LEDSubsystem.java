package frc.robot.subsystems.led;

import frc.robot.common.Constants;
import frc.robot.common.MatchMode;
import frc.robot.subsystems.NewtonSubsystem;
import frc.robot.hardware.NeoPixelLED;
import frc.robot.hardware.NeoPixelLED.PresetColor;
import edu.wpi.first.wpilibj.Timer;

public class LEDSubsystem extends NewtonSubsystem {
    private static LEDSubsystem INSTANCE = null;
    public static LEDSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new LEDSubsystem();
        return INSTANCE;
    }

    private NeoPixelLED ledStrip;
    private LEDMode ledMode;

    private Timer ampTimer;
    private Timer stagedNoteTimer;

    private LEDCommands m_commands;

    public enum LEDMode {
        kOff,
        kDisabled,
        kAmplify,
        kHasNote,
        kStagedNote,
        kTargetLockSearching,
        kTargetLockLocked
    }

    private LEDSubsystem() {
        ledStrip = new NeoPixelLED(0, Constants.LED.);
        ledMode = LEDMode.kOff;

        ampTimer = new Timer();
        stagedNoteTimer = new Timer();

        m_commands = new LEDCommands(this);
    }

    public LEDCommands getCommands() {
        return this.m_commands;
    }
    
    public void setLEDMode(LEDMode ledMode) {
        this.ledMode = ledMode;
    }

    @Override
    public void onInit(MatchMode mode) {
        if (mode == MatchMode.DISABLED) {
            ledMode = LEDMode.kDisabled;
        } else {
            ledMode = LEDMode.kOff;
        }

        ampTimer.reset();
        ampTimer.start();

        stagedNoteTimer.reset();
        stagedNoteTimer.start();
    }

    @Override
    public void periodicLogs() {
        m_logger.logEnum("LED Mode", ledMode);
    }

    @Override
    public void periodicOutputs() {
        switch (ledMode) {
            case kAmplify:
                if (ampTimer.get() <= 1.0) {
                    ledStrip.wave(PresetColor.YELLOW, PresetColor.WHITE, 0.05);
                } 
                else {
                    ledStrip.setColor(PresetColor.OFF);
                }
                break;
            case kHasNote:
                ledStrip.pulse(PresetColor.LIME_GREEN, PresetColor.OFF, 0.75);
                break;
            case kStagedNote:
                if (stagedNoteTimer.get() <= 1.0) {
                    ledStrip.scroll(PresetColor.LIME_GREEN, PresetColor.WHITE, 0.05);
                } 
                else {
                    ledStrip.setColor(PresetColor.OFF);
                }
                break;
            case kDisabled:
                ledStrip.scroll(PresetColor.TEAL, PresetColor.ORANGE, 0.05);
                break;
            case kOff:
            default:
                ledStrip.setColor(PresetColor.OFF);
                break;
        }

        if (!ledMode.equals(LEDMode.kAmplify)) ampTimer.reset();
        if (!ledMode.equals(LEDMode.kStagedNote)) stagedNoteTimer.reset();
    }
}