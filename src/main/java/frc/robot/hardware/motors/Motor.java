package frc.robot.hardware.motors;

import frc.robot.hardware.ProfileGains;

public abstract class Motor {
    /**
     * The CAN ID of the motor
     */
    public abstract int getMotorID();

    /**
     * Sets the motor following another either directly or reversed
     */
    public abstract void follow(Motor other, boolean reversed);

    /**
     * Sets the PID gains for a particular motor slot
     */
    public abstract void withGains(ProfileGains gains, int index);
    
    /**
     * Returns the velocity of the motor in RPM
     */
    public abstract double getVelocity();

    /**
     * Returns the position of the motor in Rotations
     */
    public abstract double getPosition();

    /**
     * Sets max allowed current in amps
     */
    public abstract void setCurrentLimit(int amps);

    /**
     * Sets the motor to the desired velocity in RPM
     */
    public void setVelocity(double velocityRPM) {
        setVelocity(velocityRPM, 0);
    }

    /**
     * Sets the motor to the desired velocity in RPM
     */
    public void setProfiledVelocity(double velocityRPM) {
        setProfiledVelocity(velocityRPM, 0);
    }

    /**
     * Sets the motor to the desired position in rotations
     */
    public void setPosition(double positionRotations) {
        setPosition(positionRotations, 0);
    }

    /**
     * Sets the motor to the desired velocity in RPM
     */
    public abstract void setVelocity(double velocityRPM, int pidSlot);

    /**
     * Sets the motor to the desired velocity in RPM using a trapezoidal profile
     */
    public abstract void setProfiledVelocity(double velocityRPM, int pidSlot);

    /**
     * Sets the motor to the desired position in rotations
     */
    public abstract void setPosition(double positionRotations, int pidSlot);
}