package com.NewtonSwerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Robot;
import frc.robot.common.swerve.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.NewtonSwerve.Gyro.Gyro;

public class NewtonSwerve extends Swerve {
    
    public static final double METERS_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

    // PASSED IN BY USER
    public final Gyro gyro;
    private final ModuleConfig config;

    /**
     * Swerve module controllers, intialized in the constructor
     */
    private final NewtonModule m_frontLeftModule;
    private final NewtonModule m_frontRightModule;
    private final NewtonModule m_backLeftModule;
    private final NewtonModule m_backRightModule;

    // Odometry object for swerve drive
    protected SwerveDriveOdometry odometry;
    public static final double METERS_PER_SECOND_TO_TICKS = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

    // The maximum angular velocity of the robot in radians per second.
    private double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    // Set up the kinematics module based on physical drivetrain characteristics
    private SwerveDriveKinematics m_kinematics;

    // set up the pose estimator based on vision and odometry
    private SwerveDrivePoseEstimator m_poseEstimator;

    public NewtonSwerve(ModuleConfig config, Gyro gyro, SwerveModule frontLeft, SwerveModule frontRight,
            SwerveModule backLeft, SwerveModule backRight) {

        this.gyro = gyro;
        this.config = config;

        double MAX_VELOCITY_METERS_PER_SECOND = config.getMaxVelocityMetersPerSecond();

        // grab drivetrain dimensions
        double DRIVETRAIN_LENGTH_METERS = config.getDriveTrainLengthMeters();
        double DRIVETRAIN_WIDTH_METERS = config.getDriveTrainWidthMeters();
        double WHEEL_CIRCUMFERENCE = config.getWheelCircumference();

        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_WIDTH_METERS / 2.0, DRIVETRAIN_LENGTH_METERS / 2.0);

        m_kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(DRIVETRAIN_WIDTH_METERS / 2.0, DRIVETRAIN_LENGTH_METERS / 2.0),
                // Front right
                new Translation2d(DRIVETRAIN_WIDTH_METERS / 2.0, -DRIVETRAIN_LENGTH_METERS / 2.0),
                // Back left
                new Translation2d(-DRIVETRAIN_WIDTH_METERS / 2.0, DRIVETRAIN_LENGTH_METERS / 2.0),
                // Back right
                new Translation2d(-DRIVETRAIN_WIDTH_METERS / 2.0, -DRIVETRAIN_LENGTH_METERS / 2.0)); 

        // setup modules
        this.m_frontLeftModule = new NewtonModule(frontLeft, WHEEL_CIRCUMFERENCE);
        this.m_frontRightModule = new NewtonModule(frontRight, WHEEL_CIRCUMFERENCE);
        this.m_backLeftModule = new NewtonModule(backLeft, WHEEL_CIRCUMFERENCE);
        this.m_backRightModule = new NewtonModule(backRight, WHEEL_CIRCUMFERENCE);

        // intialize odometry
        this.odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(),
                new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition() });
        
        // initialize pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(), 
                new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition() }, 
                new Pose2d());
    }

    // public NewtonSwerve(ModuleConfig config, Gyro gyro, NewtonModule frontLeft,
    // NewtonModule frontRight, NewtonModule backLeft, NewtonModule backRight){

    // this.gyro = gyro;
    // this.config = config;

    // this.MAX_VELOCITY_METERS_PER_SECOND = config.getMaxVelocityMetersPerSecond();
    // this.MAX_VOLTAGE = config.getNominalVoltage();

    // // set current limits
    // this.MAX_SWERVE_DRIVE_AUTO_CURRENT =
    // !Double.isNaN(config.getAutoCurrentLimit()) ? config.getAutoCurrentLimit() :
    // this.MAX_SWERVE_DRIVE_AUTO_CURRENT;
    // this.MAX_SWERVE_DRIVE_TELEOP_CURRENT =
    // !Double.isNaN(config.getTeleopCurrentLimit()) ?
    // config.getTeleopCurrentLimit() : this.MAX_SWERVE_DRIVE_TELEOP_CURRENT;

    // // grab drivetrain dimensions
    // this.DRIVETRAIN_LENGTH_METERS = config.getDriveTrainLengthMeters();
    // this.DRIVETRAIN_WIDTH_METERS = config.getDriveTrainWidthMeters();
    // this.WHEEL_CIRCUMFERENCE = config.getWheelCircumference();

    // // setup modules
    // this.m_frontLeftModule = frontLeft;
    // this.m_frontRightModule = frontRight;
    // this.m_backLeftModule = backLeft;
    // this.m_backRightModule = backRight;

    // // intialize odometry
    // this.odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(), new
    // SwerveModulePosition[] {new SwerveModulePosition(), new
    // SwerveModulePosition(), new SwerveModulePosition(), new
    // SwerveModulePosition()});
    // }

    public double getMaxTranslateVelocity() {
        return config.getMaxVelocityMetersPerSecond();
    }

    public double getMaxAngularVelocity() {
        return MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
    }

    // GYRO METHODS

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is
     * currently facing to the 'forwards' direction.
     */
    @Override
    public void resetGyro(double degrees) {
        gyro.setYaw(degrees);
    }

    @Override
    public Rotation2d getGyroscopeRotation() {
        if (Robot.isSimulation()) {
            return Robot.FIELD.getRobotPose().getRotation();
        }
        return Rotation2d.fromDegrees(this.getYaw());
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public double getYaw() {
        return gyro.getYaw();
    }

    // POSE METHODS

    @Override
    public Pose2d getCurrentPose() {
        if (Robot.isSimulation()) {
            return Robot.FIELD.getRobotPose();
        }
        return odometry.getPoseMeters();
    }

    @Override
    public void resetPose(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(0),
                new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition() },
                pose);

        m_poseEstimator.resetPosition(getGyroscopeRotation(), new SwerveModulePosition[] { new SwerveModulePosition(), 
            new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() }, pose);
    }

    // POSE ESTIMATOR METHODS
    // public void addVisionMeasurement(PoseVision m_poseVision) {
    //     // check if valid. for now, just if it's visible and close enough
    //     double dist = m_poseVision.getCurrTagZ();
    //     if (m_poseVision.getVisionActive() && 
    //         m_poseVision.getTagInView() && 
    //         dist < APRILTAG_VISION.FUSE_DISTANCE) {
    //         Pose2d visionPose = m_poseVision.getPose2d();
    //         double timestamp = Timer.getFPGATimestamp() - 0.1; // NT runs at 10 FPS, so subtract 0.1
    //         m_poseEstimator.addVisionMeasurement(visionPose.transformBy(APRILTAG_VISION.CAMERA_TO_ROBOT), 
    //                                              timestamp, 
    //                                              VecBuilder.fill(0.3 * dist, 0.3 * dist, 9999999)); // ignore theta
    //     }
    // }

    public Pose2d getCurrentPosVision() {
        return m_poseEstimator.getEstimatedPosition();
    }

    @Override
    public void driveFieldCentric(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, config.getMaxVelocityMetersPerSecond());

        double frontLeftVelo = states[0].speedMetersPerSecond;
        double frontRightVelo = states[1].speedMetersPerSecond;
        double backLeftVelo = states[2].speedMetersPerSecond;
        double backRightVelo = states[3].speedMetersPerSecond;

        m_frontLeftModule.setModule(states[0].angle.getRadians(), metersPerSecondToTicks(frontLeftVelo));
        m_frontRightModule.setModule(states[1].angle.getRadians(), metersPerSecondToTicks(frontRightVelo));
        m_backLeftModule.setModule(states[2].angle.getRadians(), metersPerSecondToTicks(backLeftVelo));
        m_backRightModule.setModule(states[3].angle.getRadians(), metersPerSecondToTicks(backRightVelo));

        SwerveModulePosition[] readPositions = new SwerveModulePosition[] {m_frontLeftModule.getModulePosition(),
            m_frontRightModule.getModulePosition(), m_backLeftModule.getModulePosition(),
            m_backRightModule.getModulePosition()};

        this.odometry.update(this.getGyroscopeRotation(), readPositions);
        this.m_poseEstimator.update(getGyroscopeRotation(), readPositions);
    }

    @Override
    public void driveRobotCentric(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds appliedSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, getGyroscopeRotation());
        this.driveFieldCentric(appliedSpeeds);
    }

    // zero the absolute encoder for all modules
    public void resetEncoders() {
        m_frontLeftModule.resetThrottleEncoder();
        m_frontRightModule.resetThrottleEncoder();
        m_backLeftModule.resetThrottleEncoder();
        m_backRightModule.resetThrottleEncoder();
    }

    public double metersPerSecondToTicks(double input) {
        return input * METERS_SECOND_TO_TICKS;
    }

    public double ticksToMetersPerSecond(double input) {
        return input / METERS_SECOND_TO_TICKS;
    }
    public double ticks100MSToRPM(double input) {
        // return input / ((2048 * 6.75) * 10) / (2 * Math.PI * 0.0508);
        return ((((input / 2048) * 10) * 60));
    }

    public void resetSteerAngles() {
        m_frontLeftModule.resetAbsoluteAngle();
        m_frontRightModule.resetAbsoluteAngle();
        m_backLeftModule.resetAbsoluteAngle();
        m_backRightModule.resetAbsoluteAngle();
    }

    // SET CURRENT LIMIT
    @Override
    public void setThrottleCurrentLimit(double currentLimit) {
        m_frontLeftModule.setThrottleCurrentLimit(currentLimit);
        m_frontRightModule.setThrottleCurrentLimit(currentLimit);
        m_backLeftModule.setThrottleCurrentLimit(currentLimit);
        m_backRightModule.setThrottleCurrentLimit(currentLimit);
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    @Override
    public ChassisSpeeds getCurrentSpeeds(){
        SwerveModuleState[] readStates = new SwerveModuleState[]{
            m_frontLeftModule.getModuleState(),
            m_frontRightModule.getModuleState(),
            m_backLeftModule.getModuleState(),
            m_backRightModule.getModuleState()
        };
        return m_kinematics.toChassisSpeeds(readStates);
    }

    public double[] getThrottleAppliedCurrent() {
        double frontLeftCurrent = m_frontLeftModule.getAppliedCurrent();
        double frontRightCurrent = m_frontRightModule.getAppliedCurrent();
        double backLeftCurrent = m_backLeftModule.getAppliedCurrent();
        double backRightCurrent = m_backRightModule.getAppliedCurrent();

        return new double[] { frontLeftCurrent, frontRightCurrent, backLeftCurrent, backRightCurrent };
    }

    public double[] getThrottleAppliedVelocity() {
        double frontLeftVelo = ticksToMetersPerSecond(m_frontLeftModule.getThrottleVelocity(null));
        double frontRightVelo = ticksToMetersPerSecond(m_frontRightModule.getThrottleVelocity(null));
        double backLeftVelo = ticksToMetersPerSecond(m_backLeftModule.getThrottleVelocity(null));
        double backRightVelo = ticksToMetersPerSecond(m_backRightModule.getThrottleVelocity(null));

        return new double[] { Math.abs(frontLeftVelo), Math.abs(frontRightVelo), Math.abs(backLeftVelo),
                Math.abs(backRightVelo) };
    }
}
