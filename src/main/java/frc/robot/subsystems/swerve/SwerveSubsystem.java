package frc.robot.subsystems.swerve;

import com.NewtonSwerve.Mk4.Mk4ModuleConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.common.Constants.SWERVE;
import frc.robot.common.Constants;
import frc.robot.common.MatchMode;
import frc.robot.Robot;
import frc.robot.common.Ports;
import frc.robot.common.SmartLogger;
import frc.robot.common.swerve.Swerve;
import frc.robot.common.swerve.ctre.*;
import frc.robot.common.swerve.sds.SDSModuleConfigurations;
import frc.robot.subsystems.NewtonSubsystem;

public class SwerveSubsystem extends NewtonSubsystem {
    private static SwerveSubsystem INSTANCE = null;
    public static SwerveSubsystem getInstance() {
        if (INSTANCE == null) INSTANCE = new SwerveSubsystem();
        return INSTANCE;
    }

    private Swerve m_swerve;
    private SwerveCommands m_commands;
    private ChassisSpeeds m_desiredSpeeds = new ChassisSpeeds();
    private boolean m_isSnailMode = false;
    private Pigeon2 m_pigeon;

    private SwerveSubsystem() {
        SwerveConstants drivetrain =
            new SwerveConstants()
                .withPigeon2Id(Ports.PIGEON_CAN_ID)
                .withTurnKp(SWERVE.TURN_KP)
                .withTurnKd(SWERVE.TURN_KD)
        ;

        Slot0Configs steerGains = new Slot0Configs();
        Slot0Configs driveGains = new Slot0Configs();

        {
            steerGains.kP = 0.2;
            steerGains.kD = 0.1;

            driveGains.kP = 0.02;
            driveGains.kD = 0.01;
        }

        // Stores physical information about the swerve and info about user config
        Mk4ModuleConfiguration config = new Mk4ModuleConfiguration();

        // Drivetrain dimensions
        config.setDriveTrainWidthMeters(SWERVE.DRIVE_TRAIN_WIDTH);
        config.setDriveTrainLengthMeters(SWERVE.DRIVE_TRAIN_LENGTH);
        config.setWheelCircumference(SWERVE.WHEEL_CIRCUMFERENCE);

        // Max Values
        config.setNominalVoltage(SWERVE.MAX_VOLTAGE);
        config.setMaxVelocityMetersPerSecond(SWERVE.MAX_VELOCITY_METERS_PER_SECOND);

        // Set PID constants
        config.setThrottlePID(SWERVE.THROTTLE_kP, SWERVE.THROTTLE_kI, SWERVE.THROTTLE_kD);
        config.setSteerPID(SWERVE.STEER_kP, SWERVE.STEER_kI, SWERVE.STEER_kD);

        m_pigeon = new Pigeon2(Ports.PIGEON_CAN_ID);

        SwerveConstantsCreator m_constantsCreator =
                new SwerveConstantsCreator(
                        SDSModuleConfigurations.MK4I_L2.getDriveReduction(),
                        12.8, // 12.8:1 ratio for the steer motor
                        3, // 3 inch radius for the wheels
                        17, // Only apply 17 stator amps to prevent slip
                        steerGains, // Use the specified steer gains
                        driveGains, // Use the specified drive gains
                        false // CANcoder not reversed from the steer motor. For WCP Swerve X this should be true.
                );

        SwerveModuleConstants frontLeft = m_constantsCreator.createModuleConstants(
                Ports.FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID,
                Ports.FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
                Ports.FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID,
                SWERVE.FRONT_LEFT_MODULE_STEER_OFFSET,
                SWERVE.DRIVE_TRAIN_LENGTH / 2.0,
                SWERVE.DRIVE_TRAIN_WIDTH  / 2.0
        );

        SwerveModuleConstants frontRight = m_constantsCreator.createModuleConstants(
                Ports.FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
                Ports.FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
                Ports.FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
                SWERVE.FRONT_RIGHT_MODULE_STEER_OFFSET,
                SWERVE.DRIVE_TRAIN_LENGTH / 2.0,
                -SWERVE.DRIVE_TRAIN_WIDTH  / 2.0
        );

        SwerveModuleConstants backLeft = m_constantsCreator.createModuleConstants(
                Ports.BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID,
                Ports.BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
                Ports.BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID,
                SWERVE.BACK_LEFT_MODULE_STEER_OFFSET,
                -SWERVE.DRIVE_TRAIN_LENGTH / 2.0,
                SWERVE.DRIVE_TRAIN_WIDTH  / 2.0
        );

        SwerveModuleConstants backRight = m_constantsCreator.createModuleConstants(
                Ports.BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
                Ports.BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
                Ports.BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
                SWERVE.BACK_RIGHT_MODULE_STEER_OFFSET,
                -SWERVE.DRIVE_TRAIN_LENGTH / 2.0,
                -SWERVE.DRIVE_TRAIN_WIDTH  / 2.0
        );

        // SwerveModule m_frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //     Mk4iSwerveModuleHelper.GearRatio.L2,
        //     Ports.FRONT_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
        //     Ports.FRONT_LEFT_MODULE_STEER_MOTOR_CAN_ID,
        //     Ports.FRONT_LEFT_MODULE_STEER_ENCODER_CAN_ID,
        //     SWERVE.FRONT_LEFT_MODULE_STEER_OFFSET
        // );

        // SwerveModule m_frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //     Mk4iSwerveModuleHelper.GearRatio.L2,
        //     Ports.FRONT_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
        //     Ports.FRONT_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
        //     Ports.FRONT_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
        //     SWERVE.FRONT_RIGHT_MODULE_STEER_OFFSET
        // );

        // SwerveModule m_backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //     Mk4iSwerveModuleHelper.GearRatio.L2,
        //     Ports.BACK_LEFT_MODULE_DRIVE_MOTOR_CAN_ID,
        //     Ports.BACK_LEFT_MODULE_STEER_MOTOR_CAN_ID,
        //     Ports.BACK_LEFT_MODULE_STEER_ENCODER_CAN_ID,
        //     SWERVE.BACK_LEFT_MODULE_STEER_OFFSET
        // );

        // SwerveModule m_backRightModule = Mk4iSwerveModuleHelper.createFalcon500(config,
        //     Mk4iSwerveModuleHelper.GearRatio.L2,
        //     Ports.BACK_RIGHT_MODULE_DRIVE_MOTOR_CAN_ID,
        //     Ports.BACK_RIGHT_MODULE_STEER_MOTOR_CAN_ID,
        //     Ports.BACK_RIGHT_MODULE_STEER_ENCODER_CAN_ID,
        //     SWERVE.BACK_RIGHT_MODULE_STEER_OFFSET
        // );

        // this.m_swerve = new NewtonSwerve(
        //     config,
        //     new NewtonPigeon2(m_pigeon),
        //     m_frontLeftModule, 
        //     m_frontRightModule, 
        //     m_backLeftModule, 
        //     m_backRightModule
        // );

        this.m_swerve = new CTRESwerve(
            drivetrain, 
            m_pigeon, 
            frontLeft, 
            frontRight, 
            backLeft, 
            backRight
        );

        m_commands = new SwerveCommands(this);
        super.m_logger = new SmartLogger("SwerveSubsystem");
    }

    /**
     * Drives the robot based on inputted field-centric translational and rotational speeds 
     */
    public void driveFieldCentric(ChassisSpeeds desiredSpeeds) {
        this.m_desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, m_swerve.getGyroscopeRotation());
    }

    /**
     * Drives the robot based on inputted robot-centric translational and rotational speeds
     */
    public void driveRobotCentric(ChassisSpeeds desiredSpeeds) {
        this.m_desiredSpeeds = desiredSpeeds;
    }

    /**
     * Runs through code for normalizing and processing driver input
     */
    public ChassisSpeeds processInputs(double translateX, double translateY, double rotate) {
        // double driveTranslateY = (
        //     translateY >= 0
        //     ? (Math.pow(translateY, SWERVE.TRANSLATE_EXPONENT))
        //     : -(Math.pow(translateY, SWERVE.TRANSLATE_EXPONENT))
        // );

        // double driveTranslateX = (
        //     translateX >= 0 
        //     ? (Math.pow(translateX, SWERVE.TRANSLATE_EXPONENT))
        //     : -(Math.pow(translateX, SWERVE.TRANSLATE_EXPONENT))
        // );

        // double driveRotate = (
        //     rotate >= 0
        //     ? (Math.pow(rotate, SWERVE.TRANSLATE_EXPONENT))
        //     : -(Math.pow(rotate, SWERVE.TRANSLATE_EXPONENT))
        // );

        // double driveTranslateX = Math.pow(translateX, SWERVE.TRANSLATE_EXPONENT) * Math.signum(translateX);
        // double driveTranslateY = Math.pow(translateY, SWERVE.TRANSLATE_EXPONENT) * Math.signum(translateY);
        // double driveRotate = Math.pow(rotate, SWERVE.ROTATE_EXPONENT) * Math.signum(rotate);

        double driveTranslateX = translateX;
        double driveTranslateY = translateY;
        double driveRotate = rotate;

        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;

        if (m_isSnailMode) { //Slow Mode slows down the robot for better precision & control
            currentSpeeds = new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveRotate * SWERVE.ROTATE_POWER_SLOW * SWERVE.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            );
        }
        else {
            currentSpeeds = new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveRotate * SWERVE.ROTATE_POWER_FAST * SWERVE.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            );
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            currentSpeeds, m_swerve.getGyroscopeRotation()
        );
    }

    /**
     * Whether the drivetrain is slowed down
     */
    public void setSnailMode(boolean snailMode) {
        this.m_isSnailMode = snailMode;
    }

    /**
     * The current 2d position of the robot on the field
     */
    public Pose2d getCurrentPose() {
        if (Robot.isSimulation()) {
            return Robot.FIELD.getRobotPose();
        }
        return m_swerve.getCurrentPose();
    }

    /**
     * Sets the starting 2d position of the robot on the field
     */
    public void setStartPose(Pose2d startPose) {
        m_swerve.resetGyro(startPose.getRotation().getDegrees());
        m_swerve.resetPose(startPose);
        if (Robot.isSimulation()) {
            Robot.FIELD.setRobotPose(startPose);
        }
    }

    /**
     * The current translational and rotational speeds of the robot
     */
    public ChassisSpeeds getCurrentSpeeds() {
        if (Robot.isSimulation()) {
            return this.m_desiredSpeeds;
        }
        return this.m_swerve.getCurrentSpeeds();
    }

    /**
     * The swerve commands set for the swerve subsystem
     */
    public SwerveCommands getCommands() {
        return this.m_commands;
    }

    @Override
    public void onInit(MatchMode mode) {
        // Do not reset swerve start pose on auto init since that is taken care of already
        switch (mode) {
            case AUTONOMOUS:
                this.m_swerve.setThrottleCurrentLimit(Constants.SWERVE.THROTTLE_CURRENT_LIMIT_AUTO);
                break;
            default:
                this.m_swerve.setThrottleCurrentLimit(Constants.SWERVE.THROTTLE_CURRENT_LIMIT_TELEOP);
                break;
        }
    }

    @Override
    public void periodicLogs() {
        this.m_logger.logChassisSpeeds("Desired Speeds", m_desiredSpeeds);
    }

    @Override
    public void periodicOutputs() {
        this.m_swerve.driveRobotCentric(m_desiredSpeeds);
        if (Robot.isSimulation() && Robot.MODE.isAny(MatchMode.TELEOP, MatchMode.TEST)) {
            Robot.FIELD.setRobotPose( // Translate the simulated robot pose to simulate teleop movement
                getCurrentPose().transformBy(
                    new Transform2d(
                        new Translation2d(
                            0.02 * m_desiredSpeeds.vxMetersPerSecond,
                            0.02 * m_desiredSpeeds.vyMetersPerSecond
                        ),
                        Rotation2d.fromRadians(0.02 * m_desiredSpeeds.omegaRadiansPerSecond)
                    )
                )
            );
        }
    }
}