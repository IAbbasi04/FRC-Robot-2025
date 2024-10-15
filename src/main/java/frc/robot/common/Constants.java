package frc.robot.common;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.common.math.Conversions;
import frc.robot.common.swerve.sds.SDSModuleConfigurations;
import frc.robot.hardware.ProfileGains;

public final class Constants {
    public static class FIELD {
        public static final double RED_WALL_X = 16.542;
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    }
    
    public static class SWERVE {
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -0.389648;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -0.441162;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -0.198730;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -0.498291;

        public static final double THROTTLE_kP = 3.0;
        public static final double THROTTLE_kI = 0.0;
        public static final double THROTTLE_kD = 0.0;

        public static final double STEER_kP = 100.0;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.2;

        public static final double DRIVE_MOTOR_GEAR_RATIO = SDSModuleConfigurations.MK4I_L2.getDriveReduction();
        public static final double STEER_MOTOR_GEAR_RATIO = SDSModuleConfigurations.MK4I_L2.getSteerReduction();

        public static final ProfileGains THROTTLE_GAINS = new ProfileGains()
            .setP(THROTTLE_kP)
            .setI(THROTTLE_kI)
            .setD(THROTTLE_kD)
        ;

        public static final ProfileGains STEER_GAINS = new ProfileGains()
            .setP(STEER_kP)
            .setI(STEER_kI)
            .setD(STEER_kD)
        ;
        
        public static final double DRIVE_TRAIN_WIDTH = 0.527; // meters
        public static final double DRIVE_TRAIN_LENGTH = 0.478; // meters 
        public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI; // inches

        public static final double DRIVE_TRAIN_RADIUS = Math.sqrt(Math.pow(DRIVE_TRAIN_WIDTH, 2) + Math.pow(DRIVE_TRAIN_LENGTH, 2)); // meters

        public static final double MAX_VOLTAGE = 12.0;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.5;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 4.5;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_PER_SECOND = 
            MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(
                SWERVE.DRIVE_TRAIN_WIDTH / 2.0,
                DRIVE_TRAIN_LENGTH / 2.0
            );

        public static final double SLEW_LIMIT = 2.0; // 200% rate of change per second
        public static final double TURN_KP = 0.05;
        public static final double TURN_KD = 0.02;
        public static final double TRANSLATE_EXPONENT = 1; // Exponential drive
        public static final double ROTATE_EXPONENT = 1; // Exponential drive
        public static final double THROTTLE_CURRENT_LIMIT_AUTO = 60; // amps
        public static final double THROTTLE_CURRENT_LIMIT_TELEOP = 60; // amps
    }

    public static class INTAKE {
        public static final double ROLLER_INTAKE_RPM = 4500.0;
        public static final double ROLLER_OUTAKE_RPM = -2500.0;

        public static final int INTAKE_MOTOR_CURRENT_LIMIT = 60;

        public static final ProfileGains ROLLER_GAINS = (
            new ProfileGains()
            .setP(3.0E-3)
            .setD(1.5E-2)
            .setFF(1.5E-4)
        );
    }

    public static class FEEDER {
        public static final int PID_FEED_SLOT = 0;
        public static final int PID_SHOOT_SLOT = 1;

        public static final int FEEDER_SHOOT_RPM = 2500;
        public static final int FEEDER_INTAKE_RPM = 2000;
        public static final int FEEDER_OUTAKE_RPM = -2000;
        public static final int FEEDER_ALIGN_RPM = -1000;
        public static final double FEEDER_AMP_RPM = -5000;
    }

    public static class SHOOTER {
        public static final double TARGET_TOLERANCE = 50.0;
    }

    public static class ELEVATOR {
        private static final double PIVOT_PULLEY_RATIO = 12.0 / 5.0;
        private static final double PIVOT_GEARBOX_RATIO = 75.0 / 1.0;
        public static final double PIVOT_RATIO = PIVOT_PULLEY_RATIO * PIVOT_GEARBOX_RATIO;

        private static final double DIAMETER_OF_ELEVATOR_SPROCKET = 1.885; // inches
        private static final double EXTENSION_GEARBOX_RATIO = 1.0 / 48.0;
        public static final double EXTENSION_RATIO = 
            Conversions.inchesToMeters(
                EXTENSION_GEARBOX_RATIO *                                
                DIAMETER_OF_ELEVATOR_SPROCKET * Math.PI
            );

        public static final double PIVOT_TOLERANCE = 0.1; // degrees
        public static final double EXTENSION_TOLERANCE = 0.005; // meters

        public static final double MIN_EXTENSION = 0; // meters
        public static final double MAX_EXTENSION = 0.279; // meters

        public static final double MIN_PIVOT = 0; // meters
        public static final double MAX_PIVOT = 75; // meters

        public static final double EXTENSION_PIVOT_THRESHOLD = 30; // degrees
        public static final double PIVOT_EXTENSION_THRESHOLD = 0.05; // meters
    }

    public static class LED {
        public static final int LED_LENGTH = 30;
    }

    public static class VISION {
        public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = 
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
        
        public static final Transform3d CAMERA_POSE_TO_ROBOT_POSE = new Transform3d();

        public static final String FRONT_LIMELIGHT_NAME = "limelight-targetting";
        public static final String REAR_ORANGE_PY_NAME = "jetson";

        public static final double SPEAKER_LOCK_THRESHOLD = 2.0; // degrees
    }

    public static class INPUT {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;

        public static final double TRANSLATE_POWER_FAST = 1.0; // Scaling for teleop driving.  1.0 is maximum
        public static final double ROTATE_POWER_FAST = 0.75; // Scaling for teleop driving.  1.0 is maximum
        public static final double TRANSLATE_POWER_SLOW = 0.30; // Scaling for teleop driving.  1.0 is maximum
        public static final double ROTATE_POWER_SLOW = 0.25; // Scaling for teleop driving.  1.0 is maximum

        public static final double JOYSTICK_DEADBAND_TRANSLATE = 0.05;
        public static final double JOYSTICK_DEADBAND_ROTATE = 0.05;
        public static final double SIMULATION_DEADBAND = 0.1;

        public static final double SLEW_RATE_TRANSLATE = 3.0; // % change in input per second
        public static final double SLEW_RATE_ROTATE = 3.0; // % change in input per second
    }
}