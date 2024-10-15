package frc.robot.common.swerve.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.common.swerve.Swerve;

public class CTRESwerve extends Swerve {
    private final int ModuleCount;

    private CTRESwerveModule[] m_modules;
    private Pigeon2 m_pigeon2;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDriveOdometry m_odometry;
    private SwerveModulePosition[] m_modulePositions;
    private Translation2d[] m_moduleLocations;
    private OdometryThread m_odometryThread;
    private PIDController m_turnPid;
    private Notifier m_telemetry;

    /* Put smartdashboard calls in separate thread to reduce performance impact */
    private void telemeterize() {}

    /* Perform swerve module updates in a separate thread to minimize latency */
    private class OdometryThread extends Thread {
        private BaseStatusSignal[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;

        public OdometryThread() {
            super();
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
            for (int i = 0; i < ModuleCount; ++i) {
                var signals = m_modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw();
            m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZDevice();
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (var sig : m_allSignals) {
                sig.setUpdateFrequency(250);
            }
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                var status = BaseStatusSignal.waitForAll(0.1, m_allSignals);

                SmartDashboard.putNumber("TEST FEIN FEIN FEIN", Timer.getFPGATimestamp());

                /* Get status of the waitForAll */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                for (int i = 0; i < ModuleCount; ++i) {
                    /* No need to refresh since it's automatically refreshed from the waitForAll() */
                    m_modulePositions[i] = m_modules[i].getPosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees =
                        BaseStatusSignal.getLatencyCompensatedValue(
                                m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZDevice());

                m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);
            }
        }
    }

    public CTRESwerve(
            SwerveConstants driveTrainConstants, Pigeon2 gyro, SwerveModuleConstants... modules) {
        ModuleCount = modules.length;

        m_pigeon2 = gyro;

        m_modules = new CTRESwerveModule[ModuleCount];
        m_modulePositions = new SwerveModulePosition[ModuleCount];
        m_moduleLocations = new Translation2d[ModuleCount];

        int iteration = 0;
        for (SwerveModuleConstants module : modules) {
            m_modules[iteration] = new CTRESwerveModule(module, driveTrainConstants.CANbusName);
            m_moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
            m_modulePositions[iteration] = m_modules[iteration].getPosition(true);

            iteration++;
        }
        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry =
                new SwerveDriveOdometry(m_kinematics, m_pigeon2.getRotation2d(), getSwervePositions());

        m_turnPid = new PIDController(driveTrainConstants.TurnKp, 0, driveTrainConstants.TurnKd);
        m_turnPid.enableContinuousInput(-Math.PI, Math.PI);

        m_odometryThread = new OdometryThread();
        m_odometryThread.start();

        m_telemetry = new Notifier(this::telemeterize);
        m_telemetry.startPeriodic(0.1); // Telemeterize every 100ms
    }

    private SwerveModulePosition[] getSwervePositions() {
        return m_modulePositions;
    }

    public void driveRobotCentric(ChassisSpeeds speeds) {
        var swerveStates = m_kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveFieldCentric(ChassisSpeeds speeds) {
        var roboCentric = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void driveFullyFieldCentric(double xSpeeds, double ySpeeds, Rotation2d targetAngle) {
        var currentAngle = m_pigeon2.getRotation2d();
        double rotationalSpeed =
                m_turnPid.calculate(currentAngle.getRadians(), targetAngle.getRadians());

        var roboCentric =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeeds, ySpeeds, rotationalSpeed, m_pigeon2.getRotation2d());
        var swerveStates = m_kinematics.toSwerveModuleStates(roboCentric);
        for (int i = 0; i < ModuleCount; ++i) {
            m_modules[i].apply(swerveStates[i]);
        }
    }

    public void resetPose(Pose2d pose) {
        m_odometry.resetPosition(new Rotation2d(0),
                new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(),
                        new SwerveModulePosition(), new SwerveModulePosition() },
                pose);
    }

    public void lockWheels() {
        /* Point every module toward (0,0) to make it close to a X configuration */
        for (int i = 0; i < ModuleCount; ++i) {
            var angle = m_moduleLocations[i].getAngle();
            m_modules[i].apply(new SwerveModuleState(0, angle));
        }
    }

    @Override
    public ChassisSpeeds getCurrentSpeeds() {
        return m_kinematics.toChassisSpeeds(
            m_modules[0].getModuleState(),
            m_modules[1].getModuleState(),
            m_modules[2].getModuleState(),
            m_modules[3].getModuleState()
        );
    }

    public void driveStopMotion() {
        /* Point every module toward (0,0) to make it close to a X configuration */
        for (int i = 0; i < ModuleCount; ++i) {
            var angle = m_moduleLocations[i].getAngle();
            m_modules[i].apply(new SwerveModuleState(0, angle));
        }
    }

    @Override
    public void resetGyro(double degrees) {
        m_pigeon2.setYaw(degrees);
    }

    @Override
    public Rotation2d getGyroscopeRotation() {
        if (Robot.isSimulation()) {
            return Robot.FIELD.getRobotPose().getRotation();
        }
        return m_pigeon2.getRotation2d();
    }

    @Override
    public Pose2d getCurrentPose() {
        return m_odometry.getPoseMeters();
    }

    public double getSuccessfulDaqs() {
        return m_odometryThread.SuccessfulDaqs;
    }

    public double getFailedDaqs() {
        return m_odometryThread.FailedDaqs;
    }

    @Override
    public void setThrottleCurrentLimit(double currentLimit) {
        m_modules[0].setThrottleCurrentLimit(currentLimit);
        m_modules[1].setThrottleCurrentLimit(currentLimit);
        m_modules[2].setThrottleCurrentLimit(currentLimit);
        m_modules[3].setThrottleCurrentLimit(currentLimit);
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }
}