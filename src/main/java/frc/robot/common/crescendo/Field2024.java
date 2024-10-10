package frc.robot.common.crescendo;

import edu.wpi.first.math.geometry.*;
import static frc.robot.common.Constants.FIELD.*;

public enum Field2024 {
    SPEAKER_OPENING(0.25, 5.50),
    SUBWOOFER_AMP(0.675, 6.75, Rotation2d.fromDegrees(60)),
    SUBWOOFER_MIDDLE(1.385, 5.585),
    SUBWOOFER_SOURCE(0.675, 4.345, Rotation2d.fromDegrees(-60)),

    AMP(1.82, 7.65, Rotation2d.fromDegrees(90)),

    STAGE(5.0, 4.0),

    WING_NOTE_1(2.90, 7.00),
    WING_NOTE_2(2.90, 5.585),
    WING_NOTE_3(2.90, 4.345),
    
    MID_NOTE_1(8.30, 7.475),
    MID_NOTE_2(8.30, 5.775),
    MID_NOTE_3(8.30, 4.125),
    MID_NOTE_4(8.30, 2.45),
    MID_NOTE_5(8.30, 0.75),

    WING_SCORE(6.0, 7.125 - 0.3);

    private final double m_x, m_y;
    private final Rotation2d m_rot;
    private final Pose2d m_pose;

    private Field2024(double p_x, double p_y) {
        this(p_x, p_y, new Rotation2d());
    }

    private Field2024(double p_x, double p_y, Rotation2d p_rot) {
        this.m_x = p_x;
        this.m_y = p_y;
        this.m_rot = p_rot;

        m_pose = new Pose2d(new Translation2d(m_x, m_y), m_rot);
    }

    public Pose2d getPose(boolean flipForRed) {
        if (flipForRed) {
            return new Pose2d(
                new Translation2d(RED_WALL_X - m_x, m_y),
                Rotation2d.fromDegrees(180).minus(m_rot)
            );
        }
        return m_pose;
    }

    public Pose2d getPose() {
        return this.getPose(false);
    }
}