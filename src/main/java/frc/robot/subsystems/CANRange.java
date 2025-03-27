package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CAN_RANGE;

public class CANRange extends SubsystemBase {
    private CANrange m_sensor;

    public CANRange() {
        m_sensor = new CANrange(CAN_RANGE.CAN_ID, CAN_RANGE.CAN_BUS);

        m_sensor.getConfigurator().apply(CAN_RANGE.CONFIG);
    }

    public double getDistance() {
        return m_sensor.getDistance().refresh().getValueAsDouble();
    }

    public boolean isAtTarget() {
        return getDistance() == CAN_RANGE.TARGET_DISTANCE;
    }
}
