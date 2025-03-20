package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ACTUATOR;

public class Actuator extends SubsystemBase {
    private PWM m_servo;

    private Servo servo;
    
    public Actuator() {
        // servo = new Servo(ACTUATOR.PWM_PORT);

        // m_servo = new PWM(ACTUATOR.PWM_PORT);

        // m_servo.setPosition(0);
    }

    public void setSpeed(double speed) {
        m_servo.setSpeed(speed);
    }

    public double getSpeed() {
        return m_servo.getSpeed();
    }

    public double getPosition() {
        return m_servo.getPosition();
    }

    public void setPosition(double pos) {
        m_servo.setPosition(pos);
    }

    public void stop() {
        m_servo.setSpeed(0);
    }
}
