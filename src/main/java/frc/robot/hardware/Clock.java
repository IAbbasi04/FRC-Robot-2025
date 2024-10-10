package frc.robot.hardware;

import edu.wpi.first.wpilibj.Timer;

public class Clock extends Timer {
    private double lastTime = 0.0;

    /**
     * Updates the current time of the clock
     */
    public void update() {
        lastTime = super.get();
    }

    /**
     * Returns the change in time between cycles
     */
    public double dt() {
        return super.get() - lastTime;
    }
}