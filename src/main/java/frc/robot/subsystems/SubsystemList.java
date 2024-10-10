package frc.robot.subsystems;

import java.util.ArrayList;

import frc.robot.Robot;

public class SubsystemList {
    private ArrayList<NewtonSubsystem> m_subsystems;

    public SubsystemList(NewtonSubsystem... subsystems) {
        m_subsystems = new ArrayList<>();
        for (NewtonSubsystem subsystem : subsystems) {
            
            m_subsystems.add(subsystem);
            subsystem.m_running = true;
        }
    }

    /**
     * Adds a subsystem to the list of subsytems
     */
    public void addSubsystem(NewtonSubsystem subsystem) {
        m_subsystems.add(subsystem);
    }

    public void removeSubsystem(NewtonSubsystem subsystem) {
        subsystem.m_running = false;
        m_subsystems.remove(subsystem);
    }

    public void enableShuffleboardLogging(boolean enable) {
        m_subsystems.forEach(subsystem -> subsystem.enableLogger(enable));
    }

    public void logDataAll() {
        // m_subsystems.forEach(subsystem -> subsystem.periodicLogs());
    }

    public void onInitAll() {
        m_subsystems.forEach(subsystem -> subsystem.onInit(Robot.MODE));
    }
}