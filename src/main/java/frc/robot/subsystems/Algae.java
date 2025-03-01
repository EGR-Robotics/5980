package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.ALGAE;
import frc.robot.Helpers;

import com.revrobotics.spark.SparkBase.ResetMode;

public class Algae implements Subsystem {
    private SparkFlex m_vortex;

    private SparkMax m_arm_motor;
    private RelativeEncoder m_encoder;

    private double currentVelocity = 0;

    public Algae() {
        m_vortex = new SparkFlex(ALGAE.VORTEX_CAN_ID, MotorType.kBrushless);

        m_arm_motor = new SparkMax(ALGAE.ARM_CAN_ID, MotorType.kBrushless);
        m_encoder = m_arm_motor.getEncoder();
        m_encoder.setPosition(0);

        m_arm_motor.configure(
                ALGAE.MOTOR_CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setIntakeSpeed(double speed) {
        m_vortex.set(speed);
    }

    private void setVelocity(double targetVelocity, double rampRate, SparkMax motor, Boolean up) {
        new Thread(() -> {
            while (Math.abs(targetVelocity - currentVelocity) > 0.1) { // Small threshold to stop ramping
                if (up) {
                    if (targetVelocity > currentVelocity) {
                        currentVelocity += rampRate;// Change in speed per cycle
                    } else {
                        currentVelocity -= rampRate;
                    }
                } else {
                    if (targetVelocity < currentVelocity) {
                        currentVelocity -= rampRate;// Change in speed per cycle
                    } else {
                        currentVelocity += rampRate;
                    }

                }

                motor.set(currentVelocity); // currentVelocity/ Max RPM

                try {
                    Thread.sleep(50); // Small delay for smooth ramping
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }

            motor.set(targetVelocity); // Final adjustment
        }).start();
    }

    public void setArmSpeed(double velocity) {
        // if (ALGAE.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(m_encoder.getPosition(),
        //         ALGAE.ENCODER_UPPER_LIMIT) && velocity > 0)
        //     return;
        // else if (ALGAE.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(m_encoder.getPosition(),
        //         ALGAE.ENCODER_LOWER_LIMIT) && velocity < 0)
        //     return;

        setVelocity(velocity, 0.05, m_arm_motor, velocity >= 0);
        System.out.println("Algae Bar Position: " + m_encoder.getPosition());
    }
}
