package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ELEVATOR;
import frc.robot.library.Helpers;

public class Elevator extends SubsystemBase {
    double targetEncoderPos;

    private SparkMax m_motor;

    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;

    public Elevator() {
        m_motor = new SparkMax(
                ELEVATOR.CAN_ID,
                MotorType.kBrushless);

        m_motor.configure(
                ELEVATOR.MOTOR_CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_PIDController = m_motor.getClosedLoopController();

        m_encoder = m_motor.getEncoder();
        m_encoder.setPosition(0);
    }

    public void setEncoderPosition(double pos) {
        targetEncoderPos = pos;

        m_PIDController.setReference(
                pos,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                ELEVATOR.MOTOR_ARB_F,
                ArbFFUnits.kVoltage);
    }

    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    public void setSpeed(double percentOutput) {
        // if (ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(m_encoder.getPosition(),
        //         ELEVATOR.ELEVATOR_UPPER_LIMIT) && percentOutput < 0)
        //     return;
        // else if (ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(m_encoder.getPosition(),
        //         ELEVATOR.ELEVATOR_LOWER_LIMIT) && percentOutput > 0)
        //     return;

        m_motor.set(percentOutput);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public boolean isAtTarget() {
        return ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(targetEncoderPos,
                m_encoder.getPosition());
    }
}
