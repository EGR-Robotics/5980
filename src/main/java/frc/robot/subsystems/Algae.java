package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ALGAE;
import frc.robot.Constants.ELEVATOR;

import com.revrobotics.spark.SparkBase.ResetMode;

public class Algae implements Subsystem {
    private SparkFlex intakeMotor;

    private SparkMax armMotor;
    private RelativeEncoder armEncoder;

    private SparkClosedLoopController armController;

    private double curArmPos = 0;
    private double currentVelocity = 0;

    public Algae() {
        // Initialize intake  motors
        intakeMotor = new SparkFlex(ALGAE.INTAKE_CAN_ID, MotorType.kBrushless);

        // Initialize arm motor
        armMotor = new SparkMax(ALGAE.ARM_CAN_ID, MotorType.kBrushless);

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);
        curArmPos = armEncoder.getPosition();

        armController = armMotor.getClosedLoopController();

        armMotor.configure(
            ALGAE.ARM_MOTOR_CONFIG,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void intake() {
        setIntakeSpeed(ALGAE.INTAKE_SPEED);
    }

    public void outake() {
        setIntakeSpeed(ALGAE.OUTAKE_SPEED);
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

    public void moveArm(boolean up) {
        if (up) {
            armMotor.set(ALGAE.ARM_RAISE_SPEED);
            // setVelocity(0.4, 0.05, elevatorMotor, true);
        } else {
            armMotor.set(ALGAE.ARM_LOWER_SPEED);
            
            // setVelocity(-0.15, 0.05, elevatorMotor, false);
        }

        curArmPos = armEncoder.getPosition();
    }

    public double getArmPosition() {
        return armEncoder.getPosition();
    }

    public void stopArm() {
        armMotor.set(0);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void zero() {
        // TODO: Inconsistent movement of algae bar making the encoder skip (0 != 0 all the time)
        // Also; does not run
        
        armController.setReference(
                0,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                ELEVATOR.MOTOR_ARB_F,
                ArbFFUnits.kVoltage);
    }
}