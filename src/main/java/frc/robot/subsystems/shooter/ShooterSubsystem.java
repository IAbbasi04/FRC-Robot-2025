package frc.robot.subsystems.shooter;

import frc.robot.Robot;
import frc.robot.common.MatchMode;
import frc.robot.common.Ports;
import frc.robot.common.SmartLogger;
import frc.robot.hardware.motors.Motor;
import frc.robot.hardware.motors.VortexMotor;
import frc.robot.subsystems.NewtonSubsystem;

public class ShooterSubsystem extends NewtonSubsystem {
    private static ShooterSubsystem INSTANCE = null;
    public static ShooterSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new ShooterSubsystem();
        return INSTANCE;
    }

    private Motor leftMotor, rightMotor;

    private double m_desiredLeftVelocityRPM = 0.0;
    private double m_desiredRightVelocityRPM = 0.0;

    private ShooterCommands m_commands;

    public ShooterSubsystem() {
        leftMotor = new VortexMotor(Ports.SHOOTER_LEFT_CAN_ID);
        rightMotor = new VortexMotor(Ports.SHOOTER_RIGHT_CAN_ID);
        m_commands = new ShooterCommands(this);

        super.m_logger = new SmartLogger("ShooterSubsystem");
    }

    public ShooterCommands getCommands() {
        return m_commands;
    }

    /**
     * Sets the desired speed of the left shooter motor in RPM
     */
    public void setLeftShooterVelocity(double desiredRPM) {
        m_desiredLeftVelocityRPM = desiredRPM;
    }

    /**
     * Sets the desired speed of the right shooter motor in RPM
     */
    public void setRightShooterVelocity(double desiredRPM) {
        m_desiredRightVelocityRPM = desiredRPM;
    }

    /**
     * Sets the desired speed of both shooter motors in RPM
     */
    public void setDesiredVelocity(double desiredLeftRPM, double desiredRightRPM) {
        setLeftShooterVelocity(desiredLeftRPM);
        setRightShooterVelocity(desiredRightRPM);
    }

    /**
     * Current speed of the left shooter motor in RPM
     */
    public double getLeftVelocity() {
        return leftMotor.getVelocity();
    }

    /**
     * Current speed of the left shooter motor in RPM
     */
    public double getRightVelocity() {
        return rightMotor.getVelocity();
    }

    /**
     * The desired speed in RPM of the left shooter motor
     */
    public double getDesiredLeftVelocity() {
        return m_desiredLeftVelocityRPM;
    }

    /**
     * The desired speed in RPM of the left shooter motor
     */
    public double getDesiredRightVelocity() {
        return m_desiredRightVelocityRPM;
    }

    /**
     * Whether the shooter motors are at the desired speed
     */
    public boolean isAtTargetSpeed() {
        boolean left = Math.abs(leftMotor.getVelocity() - m_desiredLeftVelocityRPM) <= 50;
        boolean right = Math.abs(rightMotor.getVelocity() - m_desiredRightVelocityRPM) <= 50;
        return (left && right) || Robot.isSimulation();
    }

    @Override
    public void periodicLogs() {
        this.m_logger.logDouble("Desired Left Velocity (RPM)", m_desiredLeftVelocityRPM);
        this.m_logger.logDouble("Desired Right Velocity (RPM)", m_desiredRightVelocityRPM);
        this.m_logger.logDouble("Current Left Velocity (RPM)", getLeftVelocity());
        this.m_logger.logDouble("Current Right Velocity (RPM)", getRightVelocity());
        this.m_logger.logBoolean("Is At Target Velocity?", isAtTargetSpeed());
    }

    @Override
    public void onInit(MatchMode mode) {
        this.setDesiredVelocity(0.0, 0.0);
    }

    @Override
    public void periodicOutputs() {
        leftMotor.setVelocity(m_desiredLeftVelocityRPM);
        rightMotor.setVelocity(m_desiredRightVelocityRPM);
    }
}