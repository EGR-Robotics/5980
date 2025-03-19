package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ALGAE;
import frc.robot.Constants.ELEVATOR;

import com.revrobotics.spark.SparkBase.ResetMode;

public class Algae implements Subsystem {
    private SparkFlex intakeMotor;

    private SparkMax armMotor;
    private RelativeEncoder armEncoder;

    private SparkClosedLoopController armController;

    public Algae() {
        // Initialize intake  motors
        intakeMotor = new SparkFlex(ALGAE.INTAKE_CAN_ID, MotorType.kBrushless);

        // Initialize arm motor
        armMotor = new SparkMax(ALGAE.ARM_CAN_ID, MotorType.kBrushless);

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

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

    public void moveArm(boolean up) {
        if (up) {
            armMotor.set(ALGAE.ARM_RAISE_SPEED);
        } else {
            armMotor.set(ALGAE.ARM_LOWER_SPEED);
        }
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