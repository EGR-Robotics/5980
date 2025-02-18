package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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

import edu.wpi.first.wpilibj.Servo;

public class ClawSubsystem implements Subsystem {
    private SparkMax armMotor;
    private RelativeEncoder armEncoder;
    private SparkClosedLoopController armController;

    private SparkMax elevatorMotor;
    private RelativeEncoder elevatorEncoder;
    private SparkClosedLoopController elevatorController;

    // Limit switches for elevator
    private DigitalInput upElevatorLimitSwitch;
    private DigitalInput lowerElevatorLimitSwitch;

    // Limit switches for arm
    private DigitalInput upArmLimitSwitch;
    private DigitalInput lowerArmLimitSwitch;

    // Actuator to push out the coral
    private Servo push;

    private double curArmPos;
    private double curElevatorPos;

    private double currentVelocity = 0;

    public ClawSubsystem() {
        // Initialize actuator
        push = new Servo(1);
        push.setAlwaysHighMode();

        // Initialize arm motors
        armMotor = new SparkMax(14, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();

        armController = armMotor.getClosedLoopController();
        armEncoder.setPosition(0);

        curArmPos = armEncoder.getPosition();

        // Initialize elevator motor
        elevatorMotor = new SparkMax(1, MotorType.kBrushless);
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

        // Configure motors to use the config

        armMotor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevatorMotor.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
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

                motor.set(currentVelocity / 5676.0); // currentVelocity/ Max RPM
                
                try {
                    Thread.sleep(50); // Small delay for smooth ramping
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            motor.set(targetVelocity / 5676.0); // Final adjustment
        }).start();
    }

    public void goToLevel1() {
        armController.setReference(5, ControlType.kMAXMotionPositionControl);
        curArmPos = armEncoder.getPosition();
    }

    public void moveArm(boolean up) {
        if (up) {
            setVelocity(.3, .05, armMotor, true);
        } else {
            setVelocity(-.3, .05, armMotor, false);
        }

        curArmPos = armEncoder.getPosition();
    }

    public void moveElevator(boolean up) {
        System.out.println(curElevatorPos);
        if (up) {
            elevatorMotor.set(1);
        } else {
            elevatorMotor.set(-1);
        }

        curElevatorPos = elevatorEncoder.getPosition();
    }

    public void drop() {
        System.out.println(push.getAngle());
        push.setAngle(0);
    }

    public Command goToLevel1Command() {
        return run(() -> goToLevel1());
    }

    public Command moveArmUpCommand() {
        return run(() -> moveArm(true));
    }

    public Command moveArmDownCommand() {
        return run(() -> moveArm(false));
    }

    public Command moveElevatorUpCommand() {
        return run(() -> moveElevator(true));
    }

    public Command moveElevatorDownCommand() {
        return run(() -> moveElevator(false));
    }

    public Command holdArmPositionCommand() {
        return run(() -> armController.setReference(curArmPos, ControlType.kMAXMotionPositionControl));
    }

    public Command holdElevatorPositionCommand() {
        return run(() -> elevatorController.setReference(curElevatorPos, ControlType.kMAXMotionPositionControl));
    }

    public Command holdCommand() {
        return run(() -> {
            elevatorController.setReference(curElevatorPos, ControlType.kMAXMotionPositionControl);
            armController.setReference(curArmPos, ControlType.kMAXMotionPositionControl);
        });
    }

    public Command dropCoralCommand() {
        return run(() -> drop());
    }
}
