package frc.robot.subsystems.intake;

import frc.robot.common.*;
import frc.robot.hardware.motors.VortexMotor;
import frc.robot.subsystems.NewtonSubsystem;

public class IntakeSubsystem extends NewtonSubsystem {
    private static IntakeSubsystem INSTANCE = null;
    public static IntakeSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new IntakeSubsystem();
        return INSTANCE;
    }

    private VortexMotor m_rollerMotor;
    private double m_desiredRollerRPM = 0.0;

    public IntakeCommands m_commands;

    private IntakeSubsystem() {
        this.m_rollerMotor = new VortexMotor(Ports.INTAKE_ROLLER_CAN_ID);
        this.m_rollerMotor.withGains(Constants.INTAKE.ROLLER_GAINS, 0);
        this.m_rollerMotor.setCurrentLimit(Constants.INTAKE.INTAKE_MOTOR_CURRENT_LIMIT);

        this.m_commands = new IntakeCommands(this);
    }

    /**
     * Sets the desired speed of the roller motor in RPM
     */
    public void setDesiredVelocity(double desiredRPM) {
        this.m_desiredRollerRPM = desiredRPM;
    }

    public double getDesiredRPM() {
        return this.m_desiredRollerRPM;
    }

    /**
     * The swerve commands set for the swerve subsystem
     */
    public IntakeCommands getCommands() {
        return this.m_commands;
    }

    @Override
    public void periodicLogs() {
        this.m_logger.logDouble("Desired Roller Velocity (RPM)", m_desiredRollerRPM);
        this.m_logger.logDouble("Current Roller Velocity (RPM)", m_rollerMotor.getVelocity());
    }

    @Override
    public void onInit(MatchMode mode) {
        this.setDesiredVelocity(0.0); // Always start each match mode with the intake turned off
    }

    @Override
    public void periodicOutputs() {
        this.m_rollerMotor.setVelocity(m_desiredRollerRPM);
    }
}