package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants.CLIMBER;

import com.revrobotics.spark.SparkBase.ResetMode;

public class Climber implements Subsystem {
    private SparkMax motor;
    private double currentVelocity = 0;

    public Climber() {
        motor = new SparkMax(13, MotorType.kBrushless);

        motor.configure(CLIMBER.MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setVelocity(double targetVelocity, double rampRate, SparkMax motor, Boolean up) {
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

    public void moveWench(boolean up) {
        if (up) {
            setVelocity(.8, .05, motor, true);
        } else {
            setVelocity(-.8, .05, motor, false);
        }
    }

    public Command stopWenchCommand() {
        return run(() -> motor.set(0));
    }

    public Command moveWenchUp() {
        return run(() -> moveWench(true));
    }

    public Command moveWenchDown() {
        return run(() -> moveWench(false));
    }
}
