package frc.robot;

// Swerve
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

// Pathplanner 
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// Constants
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.ARM;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.ELEVATOR;

// Commands
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.StopElevator;
// import frc.robot.commands.elevator.SetElevatorDistance;

// import frc.robot.commands.actuator.PushOut;
// import frc.robot.commands.actuator.StopServo;
import frc.robot.commands.algae.HoldAlgaeArm;
import frc.robot.commands.algae.MoveAlgaeArm;
import frc.robot.commands.algae.MoveIntake;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.StopArm;
import frc.robot.commands.scoring.L1;
import frc.robot.commands.scoring.L2;
import frc.robot.commands.scoring.L3;
import frc.robot.commands.scoring.L4;
import frc.robot.commands.vision.AlignLR;
import frc.robot.commands.vision.AlignTA;
import frc.robot.commands.auto.AutoAlign;
import frc.robot.commands.auto.L4Auto;
import frc.robot.commands.auto.L4LowerCoral;
import frc.robot.commands.auto.Pickup;
import frc.robot.commands.auto.ResetForPickup;

// Subsystems
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Vision;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Actuator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Algae;

import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    // Set up swerve request bindings for necessary control of the swerve drive
    // platform
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DRIVE.MAX_SPEED * DRIVE.DRIVE_DEADBAND)
            .withRotationalDeadband(DRIVE.MAX_ANGULAR_RATE * DRIVE.DRIVE_DEADBAND) // Multiply by deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(DRIVE.MAX_SPEED);

    // Initialize controllers
    public final static CommandXboxController driverJoystick = new CommandXboxController(0);
    public final static CommandXboxController controllerJoystick = new CommandXboxController(1);

    // Initialize subsystems
    public static final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Vision vision = new Vision();

    public final Climber climber = new Climber();
    public static final Actuator actuator = new Actuator();

    public static final Elevator elevator = new Elevator();
    public static final Arm arm = new Arm();
    public static final Algae algae = new Algae();

    // Initialize auto chooser
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register named commands for auto
        NamedCommands.registerCommand("L4LowerCoral", new L4LowerCoral());
        NamedCommands.registerCommand("L4", new L4Auto());

        NamedCommands.registerCommand("FullAlign", new AutoAlign());

        NamedCommands.registerCommand("ResetForPickup", new ResetForPickup());
        NamedCommands.registerCommand("Pickup", new Pickup());

        NamedCommands.registerCommand("Align", new AlignLR());

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure bindings from controller to commands
        configureBindings();
    }

    private void configureBindings() {
        // Elevator Commands

        elevator.setDefaultCommand(
            new InstantCommand(
                () -> {
                    if (controllerJoystick.getLeftY() > ELEVATOR.DEADBAND)
                        (new MoveElevator(true)).execute();
                    else if (controllerJoystick.getLeftY() < -ELEVATOR.DEADBAND)
                        (new MoveElevator(false)).execute();
                    else
                        (new StopElevator()).execute();
                }, elevator
            )
        );

        controllerJoystick.x().whileTrue(new L1());
        controllerJoystick.y().whileTrue(new L2());
        controllerJoystick.b().whileTrue(new L3());
        controllerJoystick.a().whileTrue(new L4());

        // controllerJoystick.x().whileTrue(new PushOut());
        // controllerJoystick.x().onFalse(new StopServo());

        // controllerJoystick.x().whileTrue(new SetElevatorDistance(0));

        // Arm commands

        arm.setDefaultCommand(
            new InstantCommand(
                () -> {
                    if (controllerJoystick.getRightY() > ARM.DEADBAND)
                        (new MoveArm(true)).execute();
                    else if (controllerJoystick.getRightY() < -ARM.DEADBAND)
                        (new MoveArm(false)).execute();
                    else
                        (new StopArm()).execute();
                }, arm
            )
        );

        // Algae Bar Commands

        algae.setDefaultCommand(new HoldAlgaeArm());

        controllerJoystick.leftTrigger().whileTrue(new MoveAlgaeArm(false));
        controllerJoystick.rightTrigger().whileTrue(new MoveAlgaeArm(true));

        controllerJoystick.leftBumper().whileTrue(new MoveIntake(false));
        controllerJoystick.rightBumper().whileTrue(new MoveIntake(true));

        // Drive Commands

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    double targetDriveSpeed = DRIVE.MAX_SPEED;
                    double targetAngularRate = DRIVE.MAX_ANGULAR_RATE;

                    // If elevator is raised
                    if (elevator.getEncoderPosition() <= ELEVATOR.SLOW_DOWN_POSITION) {
                        targetDriveSpeed *= ELEVATOR.DRIVE_SLOW_DOWN_RATE;
                        targetAngularRate *= ELEVATOR.DRIVE_ANGULAR_SLOW_DOWN_RATE;
                    }

                    // If the right trigger is pressed
                    if (driverJoystick.getRightTriggerAxis() == 1) {
                        targetDriveSpeed *= DRIVE.SLOW_DOWN_RATE;
                        targetAngularRate *= DRIVE.SLOW_DOWN_RATE;
                    }

                    return drive
                            // Drive forward with negative Y forward
                            .withVelocityX(-driverJoystick.getLeftY() * targetDriveSpeed)
                            // Drive left with negative X (left)
                            .withVelocityY(-driverJoystick.getLeftX() * targetDriveSpeed)
                            // Drive counterclockwise with negative X (left)
                            .withRotationalRate(-driverJoystick.getRightX() * targetAngularRate);
                }));

        // driverJoystick.leftTrigger().onTrue(new AlignLR());
        driverJoystick.leftTrigger().onTrue(new AutoAlign());

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Climber Commands
        driverJoystick.x().whileTrue(climber.moveWenchUp());
        driverJoystick.x().onFalse(climber.stopWenchCommand());
        driverJoystick.y().whileTrue(climber.moveWenchDown());
        driverJoystick.y().onFalse(climber.stopWenchCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
