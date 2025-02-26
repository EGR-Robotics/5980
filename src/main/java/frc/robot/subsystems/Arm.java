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

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ARM;

public class Arm extends SubsystemBase {
    private SparkMax m_motor;

    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;

    private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
    private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
            Double.NaN);
    private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
            Double.NaN);

    public Arm() {
        m_motor = new SparkMax(
                ARM.CAN_ID,
                MotorType.kBrushless);

        m_motor.configure(
                ARM.MOTOR_CONFIG,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        m_PIDController = m_motor.getClosedLoopController();

        m_encoder = m_motor.getEncoder();
    }

    public void setSpeed(double percentOutput) {
        m_motor.set(percentOutput);
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    }

    public void setAxisSpeed(double speed) {
        speed *= ARM.AXIS_MAX_SPEED;
        m_motor.set(speed);
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    }

    public void stop() {
        m_motor.stopMotor();
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    }

    public void setTargetRotations(Angle targetRotations) {
        m_targetRotations.mut_replace(targetRotations);
        m_PIDController.setReference(
                m_targetRotations.in(Units.Rotations),
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                ARM.MOTOR_ARB_F,
                ArbFFUnits.kVoltage);
    }

    public AngularVelocity getVelocity() {
        m_currentAngularVelocityHolder.mut_replace(
                m_encoder.getVelocity(),
                Units.RPM);
        return m_currentAngularVelocityHolder;
    }

    private Angle getRotations() {
        m_currentRotationsHolder.mut_replace(
                m_encoder.getPosition(),
                Units.Rotations);
        return m_currentRotationsHolder;
    }

    public Distance getDistance() {
        return (ARM.OUTPUT_PULLEY_CIRCUMFERENCE.times(
                getRotations().div(ARM.GEAR_RATIO).in(Units.Rotations)));
    }

    private boolean isAtTargetRotations() {
        return m_targetRotations.isNear(
                getRotations(),
                ARM.MAX_MOTION_ALLOWED_ERROR_PERCENT);
    }

    public boolean isAtTarget() {
        return isAtTargetRotations();
    }

    public void setZero() {
        m_encoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("ARM RPM", m_encoder.getVelocity());
    }
}
