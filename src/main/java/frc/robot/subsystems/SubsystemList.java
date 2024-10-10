package frc.robot.subsystems;

import java.util.ArrayList;

import frc.robot.Robot;

public class SubsystemList {
    private ArrayList<NewtonSubsystem> m_subsystems;

    public SubsystemList(NewtonSubsystem... subsystems) {
        m_subsystems = new ArrayList<>();
        for (NewtonSubsystem subsystem : subsystems) {
            subsystem.m_running = true;
            m_subsystems.add(subsystem);
        }
    }

    public void initializeLoggers(boolean logToShuffleboard) {
        m_subsystems.forEach(subsystem -> subsystem.initializeLogger(logToShuffleboard));
    }

    /**
     * Adds a subsystem to the list of subsytems
     */
    public void addSubsystem(NewtonSubsystem subsystem) {
        subsystem.m_running = true;
        m_subsystems.add(subsystem);
    }

    public void removeSubsystem(NewtonSubsystem subsystem) {
        subsystem.m_running = false;
        m_subsystems.remove(subsystem);
    }

    public void enableShuffleboardLogging(boolean enable) {
        m_subsystems.forEach(subsystem -> subsystem.enableLogger(enable));
    }

    public void onInitAll() {
        m_subsystems.forEach(subsystem -> subsystem.onInit(Robot.MODE));
    }
}