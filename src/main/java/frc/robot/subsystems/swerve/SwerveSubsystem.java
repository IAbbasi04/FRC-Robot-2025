package frc.robot.subsystems.swerve;

import com.NewtonSwerve.Mk4.Mk4ModuleConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.common.Constants.INPUT;
import frc.robot.common.Constants.SWERVE;
import frc.robot.common.Constants;
import frc.robot.common.MatchMode;
import frc.robot.Robot;
import frc.robot.common.Ports;
import frc.robot.common.swerve.Swerve;
import frc.robot.common.swerve.ctre.*;
import frc.robot.common.swerve.sds.SDSModuleConfigurations;
import frc.robot.hardware.HardwareUtils;
import frc.robot.hardware.ProfileGains;
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

    private ProfileGains turnToTargetGains = new ProfileGains()
        .setP(0.01)
        .setD(0.005)
        .setMaxVelocity(2.0) // meters per second
        .setMaxAccel(3.0) // meters per second per second
        ;

    private SwerveSubsystem() {
        SwerveConstants drivetrain =
            new SwerveConstants()
                .withPigeon2Id(Ports.PIGEON_CAN_ID)
                .withTurnKp(SWERVE.TURN_KP)
                .withTurnKd(SWERVE.TURN_KD)
        ;

        Slot0Configs driveGains = HardwareUtils.applyPIDGains(SWERVE.STEER_GAINS);
        Slot0Configs steerGains = HardwareUtils.applyPIDGains(SWERVE.STEER_GAINS);

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

        this.m_swerve = new CTRESwerve(
            drivetrain, 
            m_pigeon, 
            frontLeft, 
            frontRight, 
            backLeft, 
            backRight
        );

        m_commands = new SwerveCommands(this);
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
        if (Robot.MODE.isAny(MatchMode.AUTONOMOUS, MatchMode.DISABLED)) return new ChassisSpeeds();
        double driveTranslateX = translateX;// >= Constants.INPUT.JOYSTICK_DEADBAND_TRANSLATE ? translateX : 0;
        double driveTranslateY = translateY;// >= Constants.INPUT.JOYSTICK_DEADBAND_TRANSLATE ? translateY : 0;
        double driveRotate = rotate;// >= Constants.INPUT.JOYSTICK_DEADBAND_ROTATE ? rotate : 0;;

        //Create a new ChassisSpeeds object with X, Y, and angular velocity from controller input
        ChassisSpeeds currentSpeeds;

        if (m_isSnailMode) { //Slow Mode slows down the robot for better precision & control
            currentSpeeds = new ChassisSpeeds(
                driveTranslateY * INPUT.TRANSLATE_POWER_SLOW * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * INPUT.TRANSLATE_POWER_SLOW * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveRotate * INPUT.ROTATE_POWER_SLOW * SWERVE.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
            );
        }
        else {
            currentSpeeds = new ChassisSpeeds(
                driveTranslateY * INPUT.TRANSLATE_POWER_FAST * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * INPUT.TRANSLATE_POWER_FAST * SWERVE.MAX_VELOCITY_METERS_PER_SECOND,
                driveRotate * INPUT.ROTATE_POWER_FAST * SWERVE.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
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

    public double getRotationSpeedToTarget(Pose2d targetPose) {
        return turnToTargetGains.toProfiledPIDController().calculate(
            m_swerve.getGyroscopeRotation().getDegrees(),
            targetPose.getRotation().getDegrees()    
        );
    }

    public double getRotationSpeedToRotation(Rotation2d rotation) {
        return turnToTargetGains.toProfiledPIDController().calculate(
            m_swerve.getGyroscopeRotation().getDegrees(),
            rotation.getDegrees()
        );
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
        this.m_logger.logChassisSpeeds("Current Speeds", m_swerve.getCurrentSpeeds());
        this.m_logger.logPose2d("Current Pose", m_swerve.getCurrentPose());
    }

    @Override
    public void periodicOutputs() {
        this.m_swerve.driveRobotCentric(m_desiredSpeeds);
        if (Robot.isSimulation()) {
            switch (Robot.MODE) {
                case AUTONOMOUS:
                    double colorMultiplier = 1;
                    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                        colorMultiplier = -1;
                    }

                    Robot.FIELD.setRobotPose( // Translate the simulated robot pose to simulate teleop movement
                        getCurrentPose().transformBy(
                            new Transform2d(
                                new Translation2d(
                                    0.02 * m_desiredSpeeds.vxMetersPerSecond * colorMultiplier,
                                    0.02 * m_desiredSpeeds.vyMetersPerSecond * colorMultiplier
                                ),
                                Rotation2d.fromRadians(0.02 * m_desiredSpeeds.omegaRadiansPerSecond)
                            )
                        )
                    );
                break;
                case TEST: // Test drives the same as teleop
                case TELEOP:
                    Robot.FIELD.setRobotPose( // Translate the simulated robot pose to simulate teleop movement
                        getCurrentPose().transformBy(
                            new Transform2d(
                                new Translation2d(
                                    -0.02 * m_desiredSpeeds.vyMetersPerSecond,
                                    0.02 * m_desiredSpeeds.vxMetersPerSecond
                                ),
                                Rotation2d.fromRadians(0.02 * m_desiredSpeeds.omegaRadiansPerSecond)
                            )
                        )
                    );
                default:
                // Do nothing if in disabled
                break;
            }
            
        }
    }
}