package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ACTUATOR;

import edu.wpi.first.wpilibj2.command.Command;

public class Actuator extends SubsystemBase {
    private PWM m_servo;

    public Actuator() {
        m_servo = new PWM(ACTUATOR.PWM_PORT);
        
        System.out.println("Servo init, moving to positon 0");
        m_servo.setPosition(0);  
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
