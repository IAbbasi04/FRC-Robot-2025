package frc.robot.hardware;

import edu.wpi.first.wpilibj.DigitalInput;

public class BeamSensor {
    private DigitalInput sensor; // Plugs into digital IO port

    public BeamSensor(int channel) {
        sensor = new DigitalInput(channel);
    }

    /**
     * If the beam is broken, indicating an object has obstructed its view
     */
    public boolean isBroken() {
        return !sensor.get();
    }
}