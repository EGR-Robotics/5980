package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeSubsystem implements Subsystem {
    private SparkFlex armMotor;

    private SparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorController;

    // Limit switches for elevator
    private DigitalInput upElevatorLimitSwitch;
    private DigitalInput lowerElevatorLimitSwitch;
    private double curElevatorPos;

    private double currentVelocity = 0;

    public AlgaeSubsystem() {
        // Initialize arm motors
        armMotor = new SparkFlex(16, MotorType.kBrushless);

        // Initialize elevator motor
        elevatorMotor = new SparkMax(17, MotorType.kBrushless);
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorController = elevatorMotor.getClosedLoopController();
        elevatorEncoder.setPosition(0);

        curElevatorPos = elevatorEncoder.getPosition();

        // Create configuration for sparks
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

        /*
         * Configure the closed loop controller. We want to make sure we set the
         * feedback sensor as the primary encoder.
         */
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                // Set PID values for position control
                .p(0.1)
                .outputRange(-1, 1).maxMotion
                // Set MAXMotion parameters for position control
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(0.25);

        elevatorMotor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void goToLevel1() {
        elevatorController.setReference(5, ControlType.kMAXMotionPositionControl);
        curElevatorPos = elevatorEncoder.getPosition();
    }
       
    public void moveArm() {
        armMotor.set(-0.3);
    }

    public void drop() {
        armMotor.set(0.3);
    }

    public void setVelocity(double targetVelocity, double rampRate, SparkMax motor, Boolean up) {
        new Thread(() -> {
            while (Math.abs(targetVelocity - currentVelocity) > 0.1) { // Small threshold to stop ramping
                if(up){
                    if (targetVelocity > currentVelocity) {
                        currentVelocity += rampRate;// Change in speed per cycle
                    } else {
                        currentVelocity -= rampRate;
                    }
                }
                else{
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

    public void moveElevator(boolean up) {
        System.out.println("cur elevator position " + elevatorEncoder.getVelocity());
        System.out.println("cur elevator voltage " + elevatorMotor.getBusVoltage());

        if (up) {
            setVelocity(0.1, 0.05, elevatorMotor, true);
        } else {
            setVelocity(-0.1, 0.05, elevatorMotor, false);
        }

        // elevatorController.setReference(1, ControlType.kMAXMotionPositionControl);

        curElevatorPos = elevatorEncoder.getPosition();
    }

    public Command goToLevel1Command() {
        return run(() -> goToLevel1());
    }

    public Command moveArmCommand() {
        return run(() -> moveArm());
    }

    public Command dropAlgaeCommand() {
        return run(() -> drop());
    }

    public Command moveElevatorUpCommand() {
        return run(() -> moveElevator(true));
    }

    public Command moveElevatorDownCommand() {
        return run(() -> moveElevator(false));
    }

    public Command holdElevatorPositionCommand() {
        return run(() -> elevatorController.setReference(curElevatorPos, ControlType.kMAXMotionPositionControl));
    }

    public Command stopArm() {
        currentVelocity = 0;

        return run(() -> armMotor.set(0));
    }
}
