package frc.robot.common.swerve.ctre;

public class SwerveConstants {
    /** CAN ID of the Pigeon2 on the drivetrain */
    public int Pigeon2Id = 0;
    /** Name of CANivore the swerve drive is on */
    public String CANbusName = "rio";

    public double TurnKp = 0;
    public double TurnKd = 0;

    public SwerveConstants withPigeon2Id(int id) {
        this.Pigeon2Id = id;
        return this;
    }

    public SwerveConstants withCANbusName(String name) {
        this.CANbusName = name;
        return this;
    }

    public SwerveConstants withTurnKp(double TurnKp) {
        this.TurnKp = TurnKp;
        return this;
    }

    public SwerveConstants withTurnKd(double TurnKd) {
        this.TurnKd = TurnKd;
        return this;
    }
}