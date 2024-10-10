package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.MatchMode;
import frc.robot.common.SmartLogger;

public abstract class NewtonSubsystem extends SubsystemBase {
    protected SmartLogger m_logger;
    protected boolean m_running = false;
    
    public abstract void onInit(MatchMode mode);

    public abstract void periodicLogs();

    public abstract void periodicOutputs();

    @Override
    public void periodic() {
        periodicLogs();
        periodicOutputs();
    }

    public boolean isRunning() {
        return m_running;
    }

    /**
   * Either turns on or off the logging for the particular subsystem
   */
  public void enableLogger(boolean log) {
    if (log) {
        m_logger.enable();
    } else {
        m_logger.disable();
    }
  }
}