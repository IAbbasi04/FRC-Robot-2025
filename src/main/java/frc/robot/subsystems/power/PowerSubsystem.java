package frc.robot.subsystems.power;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.common.MatchMode;
import frc.robot.common.Ports;
import frc.robot.subsystems.NewtonSubsystem;

public class PowerSubsystem extends NewtonSubsystem {
    private static PowerSubsystem INSTANCE = null;
    public static PowerSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new PowerSubsystem();
        return INSTANCE;
    }

    private PowerDistribution m_PDH;

    private PowerSubsystem() {
        m_PDH = new PowerDistribution(Ports.PDH_ID, ModuleType.kRev); // Enables power distribution logging
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void periodicLogs() {
        m_logger.logDouble("Robot Voltage", m_PDH.getVoltage());
        m_logger.logDouble("Robot Total Current", m_PDH.getTotalCurrent());
        m_logger.logDouble("Robot Total Energy", m_PDH.getTotalEnergy());
        m_logger.logDouble("Robot Total Power", m_PDH.getTotalPower());
        m_logger.logDouble("Robot Temperature", m_PDH.getTemperature());
    }

    @Override
    public void periodicOutputs() {}
}