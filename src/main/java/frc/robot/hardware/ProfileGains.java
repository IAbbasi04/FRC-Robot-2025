package frc.robot.hardware;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ProfileGains {
    private int pidSlot = 0;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double kS = 0;
    private double kA = 0;
    private double kV = 0;
    private double kG = 0;


    private double maxAccel = Double.POSITIVE_INFINITY;
    private double maxVelo = Double.POSITIVE_INFINITY;

    private double tolerance = 0;

    private boolean continuousInput = false;
    private double continuousMin = 0;
    private double continuousMax = 0;

    private boolean softLimit = false;
    private double softLimitMin = 0;
    private double softLimitMax = 0;

    private double scale = 1.0;

    private int currentLimit = -1;

    private boolean useSmartMotion = false;

    public ProfileGains() {}

    public ProfileGains setP(double gain) {
        kP = gain;
        return this;
    }

    public double getP() {
        return kP;
    }

    public ProfileGains setI(double gain) {
        kI = gain;
        return this;
    }

    public double getI() {
        return kI;
    }

    public ProfileGains setD(double gain) {
        kD = gain;
        return this;
    }

    public double getD() {
        return kD;
    }

    public ProfileGains setFF(double gain) {
        kFF = gain;
        return this;
    }

    public double getFF() {
        return kFF;
    }

    public ProfileGains setV(double gain) {
        this.kV = gain;
        return this;
    }

    public double getV() {
        return kV;
    }

    public ProfileGains setA(double gain) {
        this.kA = gain;
        return this;
    }

    public double getA() {
        return kA;
    }

    public ProfileGains setS(double gain) {
        this.kS = gain;
        return this;
    }

    public double getS() {
        return kS;
    }

    public ProfileGains setG(double gain) {
        this.kG = gain;
        return this;
    }

    public double getG() {
        return kG;
    }

    public ProfileGains setMaxAccel(double gain) {
        maxAccel = gain;
        useSmartMotion = true;
        return this;
    }

    public double getMaxAccel() {
        return maxAccel;
    }

    public ProfileGains setMaxVelocity(double gain) {
        maxVelo = gain;
        useSmartMotion = true;
        return this;
    }

    public double getMaxVelocity() {
        return maxVelo;
    }

    public ProfileGains setSlot(int slot) {
        pidSlot = slot;
        return this;
    }

    public int getSlot() {
        return pidSlot;
    }

    public ProfileGains setTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public double getTolerance() {
        return tolerance;
    }

    /**
     * Useful for continually rotating mechanisms
     */
    public ProfileGains setContinuousInput(double min, double max) {
        this.continuousMin = min;
        this.continuousMax = max;
        this.continuousInput = true;
        return this;
    }

    public boolean isContinuousInput() {
        return continuousInput;
    }

    /**
     * Currently only works for neo motors
     */
    public ProfileGains setSoftLimits(double min, double max) {
        this.softLimitMin = min;
        this.softLimitMax = max;
        this.softLimit = true;
        return this;
    }

    public boolean hasSoftLimits() {
        return softLimit;
    }

    /**
     * Double array returns both min and max -> [minimum input, maximum input]
     */
    public float[] getSoftLimits() {
        return new float[] {(float)softLimitMin, (float)softLimitMax};
    }

    /**
     * Sets a scaling factor for the output
     */
    public ProfileGains setScalingFactor(double scale) {
        this.scale = scale;
        return this;
    }

    /**
     * Returns the scaling factor
     */
    public double getScalingFactor() {
        return scale;
    }

    /**
     * Sets a maximum current limit
     */
    public ProfileGains setCurrentLimit(int limit) {
        this.currentLimit = limit;
        return this;
    }

    /**
     * Returns the maximum current limit
     */
    public int getCurrentLimit() {
        return currentLimit;
    }

    /**
     * If smart motion and/or smart velocity should be applied
     */
    public boolean isSmartMotion() {
        return useSmartMotion;
    }

    /**
     * Creates a PID controller with the specified constants and configurations
     */
    public ProfiledPIDController toProfiledPIDController() {
        ProfiledPIDController pidCtrl = new ProfiledPIDController(kP, kI, kD, new Constraints(maxVelo * scale, maxAccel * scale));
        pidCtrl.setTolerance(tolerance);
        if (continuousInput) {
            pidCtrl.enableContinuousInput(continuousMin, continuousMax);
        }
        return pidCtrl;
    }

    /**
     * Creates a PID controller with the specified constants and configurations
     */
    public PIDController toPIDController() {
        PIDController pidCtrl = new PIDController(kP, kI, kD);
        pidCtrl.setTolerance(tolerance);
        if (continuousInput) {
            pidCtrl.enableContinuousInput(continuousMin, continuousMax);
        }
        return pidCtrl;
    }
}