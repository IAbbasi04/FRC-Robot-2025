package frc.robot.subsystems.led;

import frc.robot.Robot;
import frc.robot.common.Constants;
import frc.robot.common.Controls;
import frc.robot.common.MatchMode;
import frc.robot.common.Ports;
import frc.robot.subsystems.NewtonSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
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

    private Timer stagedNoteTimer;

    private LEDCommands m_commands;

    public enum LEDMode {
        kOff,
        kDisabled,
        kHasNote,
        kStagedNote,
        kTargetLockSearching,
        kTargetLockLocking,
        kTargetLockLocked,
    }

    private LEDSubsystem() {
        ledStrip = new NeoPixelLED(Ports.LED_PWM_ID, Constants.LED.LED_LENGTH);
        ledMode = LEDMode.kOff;

        stagedNoteTimer = new Timer();
        m_commands = new LEDCommands(this);
    }

    public LEDCommands getCommands() {
        return this.m_commands;
    }
    
    public void setLEDMode(LEDMode ledMode) {
        this.ledMode = ledMode;
    }

    private void updateLEDMode() {
        LEDMode desiredLEDMode = LEDMode.kOff;
        switch(FeederSubsystem.getInstance().getNoteState()) {
            case kAligning:
            case kHasNote:
                desiredLEDMode = LEDMode.kHasNote;
                break;
            case kStaged:
                desiredLEDMode = LEDMode.kStagedNote;
                break;
            case kNone:
            default:
                desiredLEDMode = LEDMode.kOff;
                break;
        }

        if (Controls.SPEAKER_LOCK.getAsBoolean()) { // Locking to target
            if (VisionSubsystem.getInstance().isSpeakerTargetLocked()) {
                desiredLEDMode = LEDMode.kTargetLockLocked;
            } else if (VisionSubsystem.getInstance().isSpeakerTargetVisible()) {
                desiredLEDMode = LEDMode.kTargetLockLocking;
            } else {
                desiredLEDMode = LEDMode.kTargetLockSearching;
            }
        }

        if (Robot.MODE.is(MatchMode.DISABLED)) {
            desiredLEDMode = LEDMode.kDisabled;
        }

        
        setLEDMode(desiredLEDMode);
    }

    @Override
    public void onInit(MatchMode mode) {
        if (mode == MatchMode.DISABLED) {
            ledMode = LEDMode.kDisabled;
        } else {
            ledMode = LEDMode.kOff;
        }

        stagedNoteTimer.reset();
        stagedNoteTimer.start();
    }

    @Override
    public void periodicLogs() {
        m_logger.logEnum("LED Mode", ledMode);
    }

    @Override
    public void periodicOutputs() {
        this.updateLEDMode();

        switch (ledMode) {
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
            case kTargetLockLocked:
                ledStrip.setColor(PresetColor.LIME_GREEN);
                break;
            case kTargetLockLocking:
                ledStrip.setColor(PresetColor.YELLOW);
                break;
            case kTargetLockSearching:
                ledStrip.pulse(PresetColor.YELLOW, PresetColor.OFF, 0.5);
                break;
            case kOff:
            default:
                ledStrip.setColor(PresetColor.OFF);
                break;
        }

        if (!ledMode.equals(LEDMode.kStagedNote)) stagedNoteTimer.reset();
    }
}