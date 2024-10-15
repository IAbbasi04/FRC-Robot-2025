package frc.robot.subsystems;

import frc.robot.common.MatchMode;
import frc.robot.hardware.motors.Motor;

public abstract class SimpleRollerSubsystem<M extends Motor> extends NewtonSubsystem {
    protected M m_motor = null; // Make sure to initialize this in the subsystem itself
    protected double m_desiredVelocityRPM = 0.0;

    public void setDesiredVelocity(double desiredRPM) {
        this.m_desiredVelocityRPM = desiredRPM;
    }

    public double getDesiredVelocity() {
        return m_desiredVelocityRPM;
    }

    @Override
    public void onInit(MatchMode mode) {
        this.setDesiredVelocity(0.0);
    }

    @Override
    public abstract void periodicLogs();

    @Override
    public void periodicOutputs() {
        m_motor.setVelocity(m_desiredVelocityRPM);
    }
}