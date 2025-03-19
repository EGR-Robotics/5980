package frc.robot.library;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ELEVATOR;
import frc.robot.Helpers;

public class GenericSparkMax {
    public SparkMax m_motor;

    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;

    private double m_motor_arb_f;
    private double m_target_encoder_pos;

    public GenericSparkMax(int canID, MotorType motorType, SparkMaxConfig motorConfig, double motorARBF) {
        m_motor = new SparkMax(canID, motorType);

        m_motor.configure(
                motorConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters);

        m_PIDController = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();

        m_encoder.setPosition(0);
        m_target_encoder_pos = m_encoder.getPosition();

        m_motor_arb_f = motorARBF;
    }

    public void setEncoderPosition(double pos) {
        m_PIDController.setReference(
                pos,
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                m_motor_arb_f,
                ArbFFUnits.kVoltage);
    }

    public double getEncoderPosition() {
        return m_encoder.getPosition();
    }

    public void setSpeed(double percentOutput) {
        m_motor.set(percentOutput);
    }

    public void stop() {
        m_motor.stopMotor();
    }

    public boolean isAtTarget() {
        return ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT > Helpers.percentError(m_target_encoder_pos,
                m_encoder.getPosition());
    }
}
