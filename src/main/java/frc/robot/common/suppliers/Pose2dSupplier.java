package frc.robot.common.suppliers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Pose2dSupplier {
    private DoubleSupplier m_x, m_y;
    private Rotation2dSupplier m_rot;
    public Pose2dSupplier(DoubleSupplier p_x, DoubleSupplier p_y, Rotation2dSupplier p_rot) {
        m_x = p_x;
        m_y = p_y;
        m_rot = p_rot;
    }

    public DoubleSupplier getX() {
        return m_x;
    }

    public DoubleSupplier getY() {
        return m_y;
    }

    public Rotation2dSupplier getRotation() {
        return m_rot;
    }

    public Pose2d getAsPose2d() {
        return new Pose2d(
            new Translation2d(
                m_x.getAsDouble(), 
                m_y.getAsDouble()), 
                m_rot.getAsRotation2d()
        );
    }
}
