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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ELEVATOR;
import frc.robot.Helpers;

public class Elevator extends SubsystemBase {
    double curEncoderValue;
    Angle targetRots;
    Distance targetDistance;

    double targetEncoderPos;

    private SparkMax m_motor;

    private SparkClosedLoopController m_PIDController;
    private RelativeEncoder m_encoder;

    private MutAngle m_targetRotations = Units.Rotations.mutable(Double.NaN);
    private MutAngularVelocity m_currentAngularVelocityHolder = Units.RPM.mutable(
            Double.NaN);
    private MutAngle m_currentRotationsHolder = Units.Rotations.mutable(
            Double.NaN);

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

        // curEncoderValue = m_encoder.getPosition();

        targetRots = getRotations();
        targetDistance = getDistance();
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
        if (ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(m_encoder.getPosition(),
                ELEVATOR.ELEVATOR_UPPER_LIMIT) && percentOutput < 0)
            return;
        else if (ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT >= Helpers.percentError(m_encoder.getPosition(),
                ELEVATOR.ELEVATOR_LOWER_LIMIT) && percentOutput > 0)
            return;

        m_motor.set(percentOutput);
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
        // curEncoderValue = m_encoder.getPosition();

        System.out.println("Elevator position: " + m_encoder.getPosition());
    }

    public void setAxisSpeed(double speed) {
        speed *= ELEVATOR.AXIS_MAX_SPEED;
        m_motor.set(speed);
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    }

    public void stop() {
        m_motor.stopMotor();
        m_targetRotations.mut_replace(Double.NaN, Units.Rotations);
    }

    public void setTargetRotations(Angle targetRotations) {
        m_PIDController.setReference(
                m_targetRotations.in(Units.Rotations),
                ControlType.kMAXMotionPositionControl,
                ClosedLoopSlot.kSlot0,
                ELEVATOR.MOTOR_ARB_F,
                ArbFFUnits.kVoltage);
    }

    public void setTargetDistance(Distance targetDistance) {
        Angle rotations = Units.Rotations.of(
                targetDistance
                        .div(ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE)
                        .times(ELEVATOR.GEAR_RATIO)
                        .magnitude());

        setTargetRotations(rotations);
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
        return (ELEVATOR.OUTPUT_PULLEY_CIRCUMFERENCE.times(
                getRotations().div(ELEVATOR.GEAR_RATIO).in(Units.Rotations)));
    }

    public boolean isAtTarget() {
        return ELEVATOR.MAX_MOTION_ALLOWED_ERROR_PERCENT > Helpers.percentError(targetEncoderPos, m_encoder.getPosition());
    }

    public void postMove() {
        targetRots = getRotations();
        targetDistance = getDistance();
    }

    public void holdPosition() {
        // setSpeed(-.5);
    }

    public Command holdPositionCommand() {
        return run(() -> holdPosition());
    }

    @Override
    public void periodic() {

    }
}
