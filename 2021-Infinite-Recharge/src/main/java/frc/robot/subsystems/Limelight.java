package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    public static Limelight instance;

    private NetworkTable m_visionTable;
    private NetworkTableEntry m_visX;
    private NetworkTableEntry m_visY;
    private NetworkTableEntry m_visA;
    private NetworkTableEntry m_ledState;

    public Limelight() {
        Limelight.instance = this;
        m_visionTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_visX = m_visionTable.getEntry("tx");
        m_visY = m_visionTable.getEntry("ty");
        m_visA = m_visionTable.getEntry("ta");
        m_ledState = m_visionTable.getEntry("ledMode");

        m_ledState.forceSetDouble(3.0);
    }

    public double getX() {
        return m_visX.getDouble(0.0);
    }

    public double getY() {
        return m_visY.getDouble(0.0);
    }

    public double getA() {
        return m_visA.getDouble(0.0);
    }

    public static Limelight getInstance() {
        return Limelight.instance;
    }

}
