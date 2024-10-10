package frc.robot.subsystems.elevator;

import frc.robot.common.Constants.ELEVATOR;
import frc.robot.common.Conversions;
import frc.robot.common.MatchMode;
import frc.robot.common.Ports;
import frc.robot.hardware.ProfileGains;
import frc.robot.hardware.motors.VortexMotor;
import frc.robot.subsystems.NewtonSubsystem;

public class ElevatorSubsystem extends NewtonSubsystem {
    private static ElevatorSubsystem INSTANCE = null;
    public static ElevatorSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new ElevatorSubsystem();
        return INSTANCE;
    }

    public enum ElevatorState {
        kDefault(0, 0, 0, 0),
        kStow(0, 0, 1, 1),
        kAmp(0.27, 45, 0, 0),
        kClimb(0.279, 70, 2, 2);

        public double extension, pivot;
        public int pivotSlot, extensionSlot;
        private ElevatorState(double extension, double pivot, int pivotSlot, int extensionSlot) {
            this.extension = extension;
            this.pivot = pivot;
            this.pivotSlot = pivotSlot;
            this.extensionSlot = extensionSlot;
        }
    }

    private ProfileGains pivotDefaultGains = new ProfileGains()
        .setP(1E-6)
        .setFF(2.5E-4)
        .setMaxAccel(6500.0)
        .setMaxVelocity(5000.0)
    ;

    private ProfileGains pivotStowGains = new ProfileGains()
        .setP(1E-6)
        .setFF(2.5E-4)
        .setMaxAccel(6500.0)
        .setMaxVelocity(5000.0)
    ;

    private ProfileGains pivotClimbGains = new ProfileGains()
        .setP(1E-6)
        .setFF(2.5E-4)
        .setMaxAccel(6500.0)
        .setMaxVelocity(5000.0)
    ;

    private ProfileGains extensionUpGains = new ProfileGains()
        .setP(1E-6)
        .setFF(2.5E-4)
        .setMaxAccel(5000.0)
        .setMaxVelocity(10000.0)
    ;

    private ProfileGains extensionDownGains = new ProfileGains()
        .setP(1E-6)
        .setFF(2.5E-4)
        .setMaxAccel(5000.0)
        .setMaxVelocity(10000.0)
    ;

    private ProfileGains extensionClimbGains = new ProfileGains()
        .setP(1E-6)
        .setFF(2.5E-4)
        .setMaxAccel(5000.0)
        .setMaxVelocity(10000.0)
    ;

    private VortexMotor leftPivotMotor, rightPivotMotor, extensionMotor;
    private double desiredAngle, desiredExtension;
    private ElevatorState desiredState;
    private ElevatorCommands m_commands;
    
    private ElevatorSubsystem() {
        leftPivotMotor = new VortexMotor(Ports.PIVOT_MOTOR_CAN_ID);
        rightPivotMotor = new VortexMotor(Ports.PIVOT_FOLLOW_MOTOR_CAN_ID, true);
        extensionMotor = new VortexMotor(Ports.ELEVATOR_MOTOR_CAN_ID);

        leftPivotMotor.withGains(pivotDefaultGains, 0);
        leftPivotMotor.withGains(pivotStowGains, 1);
        leftPivotMotor.withGains(pivotClimbGains, 2);

        rightPivotMotor.withGains(pivotDefaultGains, 0);
        rightPivotMotor.withGains(pivotStowGains, 1);
        rightPivotMotor.withGains(pivotClimbGains, 2);

        extensionMotor.withGains(extensionUpGains, 0);
        extensionMotor.withGains(extensionDownGains, 1);
        extensionMotor.withGains(extensionClimbGains, 2);

        desiredAngle = 0.0;
        desiredExtension = 0.0;

        desiredState = ElevatorState.kDefault;

        m_commands = new ElevatorCommands(this);
    }

    /**
     * Current angle of the pivot in degrees
     */
    public double getPivotAngleDegrees() {
        return Conversions.rotationsToDegrees(leftPivotMotor.getPosition()) * ELEVATOR.PIVOT_RATIO;
    }

    /**
     * Current extension of the elevator in meters
     */
    public double getElevatorExtensionMeters() {
        return extensionMotor.getPosition() * ELEVATOR.EXTENSION_RATIO;
    }

    /**
     * Desired angle in degrees for the pivot
     */
    public double getDesiredPivotAngle() {
        return desiredAngle;
    }

    /**
     * Desired exntension in meters for the elevator
     */
    public double getDesiredExtension() {
        return desiredExtension;
    }

    /**
     * The desired state of the elevator
     */
    public ElevatorState getElevatorState() {
        return desiredState;
    }

    /**
     * Both extension and pivot are at desired positions
     */
    public boolean atTargetPosition() {
        boolean pivot = Math.abs(getPivotAngleDegrees() - desiredAngle) <= ELEVATOR.PIVOT_TOLERANCE;
        boolean extension = Math.abs(getElevatorExtensionMeters() - desiredExtension) <= ELEVATOR.EXTENSION_TOLERANCE;
        return pivot && extension;
    }

    /**
     * Sets the desired state of the elevator
     */
    public void setElevatorState(ElevatorState state) {
        this.desiredState = state;
    }

    /**
     * Sets the distance in meters for the elevator to extend
     */
    public void setExtension(double meters) {
        this.desiredExtension = meters;
        desiredState = ElevatorState.kDefault;
    }

    /**
     * Sets the distance in meters for the elevator to extend
     */
    public void setPivot(double degrees) {
        this.desiredAngle = degrees;
        desiredState = ElevatorState.kDefault;
    }

    /**
     * Raises or lowers desired extension position of the elevator
     */
    public void moveElevator(double deltaMeters) {
        setExtension(desiredExtension + deltaMeters);
    }

    /**
     * Raises or lowers desired angle of the pivot
     */
    public void movePivot(double deltaDegrees) {
        setPivot(desiredAngle + deltaDegrees);
    }

    public ElevatorCommands getCommands() {
        return this.m_commands;
    }

    @Override
    public void periodicLogs() {
        m_logger.logDouble("Current Angle Degrees", getPivotAngleDegrees());
        m_logger.logDouble("Current Extension Meters", getElevatorExtensionMeters());
        m_logger.logDouble("Desired Pivot", getDesiredPivotAngle());
        m_logger.logDouble("Desired Extension", getDesiredExtension());
        m_logger.logBoolean("At Target", atTargetPosition());
        m_logger.logEnum("Elevator State", getElevatorState());
    }

    @Override
    public void onInit(MatchMode mode) {
        // Always stall elevator and pivot at the start of each match mode
        extensionMotor.setVelocity(0.0);
        leftPivotMotor.setVelocity(0.0);
        rightPivotMotor.setVelocity(0.0);
    }

    @Override
    public void periodicOutputs() {
        double outputExtension = desiredExtension;
        double outputAngle = desiredAngle;

        int pivotSlot = 0;
        int extensionSlot = 0;

        switch (desiredState) {
            case kStow:
            case kAmp:
            case kClimb: // All above states use the enum constant's values
                desiredExtension = desiredState.extension;
                desiredAngle = desiredState.pivot;

                pivotSlot = desiredState.pivotSlot;
                extensionSlot = desiredState.extensionSlot;
                break;
            default:
                // Do nothing extra
                break;
        }

        // Clamp desired extension within range
        if (desiredExtension >= ELEVATOR.MAX_EXTENSION) {
            desiredExtension = ELEVATOR.MAX_EXTENSION;
        } else if (desiredExtension <= ELEVATOR.MIN_EXTENSION) {
            desiredExtension = ELEVATOR.MIN_EXTENSION;
        }

        // Clamp desired angle within range
        if (desiredAngle >= ELEVATOR.MAX_PIVOT) {
            desiredAngle = ELEVATOR.MAX_PIVOT;
        } else if (desiredAngle <= ELEVATOR.MIN_PIVOT) {
            desiredAngle = ELEVATOR.MIN_PIVOT;
        }

        // Extension
        if (desiredExtension >= getElevatorExtensionMeters()) { // Extending
            if (getPivotAngleDegrees() >= ELEVATOR.EXTENSION_PIVOT_THRESHOLD) { // Pivot above threshold
                outputExtension = desiredExtension;
            } else { // Pivot too low
                outputExtension = getElevatorExtensionMeters();
            }

            if (desiredState != ElevatorState.kClimb) {
                extensionSlot = extensionUpGains.getSlot();
            }
        } else if (desiredExtension < getElevatorExtensionMeters()) { // Retracting
            if (getPivotAngleDegrees() <= ELEVATOR.EXTENSION_PIVOT_THRESHOLD) { // Pivot above threshold
                outputExtension = getElevatorExtensionMeters();
                outputAngle = ELEVATOR.EXTENSION_PIVOT_THRESHOLD + 5.0; // Adding a few degrees because of error margin
            } else { // Pivot too low
                outputExtension = desiredExtension;
            }

            if (desiredState != ElevatorState.kClimb) {
                extensionSlot = extensionDownGains.getSlot();
            }
        }

        // Pivot
        if (desiredAngle >= getPivotAngleDegrees()) { // Raising
            outputAngle = desiredAngle;
        } else if (desiredAngle < getPivotAngleDegrees()) { // Lowering
            if (getElevatorExtensionMeters() >= ELEVATOR.PIVOT_EXTENSION_THRESHOLD) { // Elevator currently extended
                outputAngle = getPivotAngleDegrees();
            } else { // Extension below threshold
                outputAngle = desiredAngle;
            }
        }

        double outputPivotRotations = Conversions.degreesToRotations(outputAngle / ELEVATOR.PIVOT_RATIO);
        double outputExtensionRotations = outputExtension / ELEVATOR.EXTENSION_RATIO;

        leftPivotMotor.setPosition(outputPivotRotations, pivotSlot);
        rightPivotMotor.setPosition(outputPivotRotations, pivotSlot);
        extensionMotor.setPosition(outputExtensionRotations, extensionSlot);
    }
}