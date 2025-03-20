package frc.robot.library;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Generic subsystem to interact with generic motor
 */
public class GenericMotorSubsystem extends SubsystemBase {
    public GenericMotor m_motor;

    /**
     * Setter for subsystem motor component
     * 
     * @param motor GenericMotor object
     */
    public void setMotor(GenericMotor motor) {
        m_motor = motor;
    }
}
