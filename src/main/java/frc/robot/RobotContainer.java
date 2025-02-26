package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ACTUATOR;
// Constants
import frc.robot.Constants.SCORING;
import frc.robot.generated.TunerConstants;

// Commands
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.SetElevatorDistance;
import frc.robot.commands.actuator.Drop;
import frc.robot.commands.arm.MoveArm;
import frc.robot.commands.arm.SetArmDistance;
import frc.robot.commands.arm.StopArm;
import frc.robot.commands.elevator.StopElevator;
import frc.robot.commands.scoring.L4;
// Subsystems
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;

import frc.robot.subsystems.Actuator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Initialize controllers
    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController controllerJoystick = new CommandXboxController(1);

    // Initialize subsystems
    
    public final AlgaeSubsystem algae = new AlgaeSubsystem();
    public final VisionSubsystem vision = new VisionSubsystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public static final Elevator elevator = new Elevator();
    public static final Arm arm = new Arm();
    public static final Actuator actuator = new Actuator();

    public RobotContainer() {
        // Register named commands for auto
        // NamedCommands.registerCommand("level1", claw.goToLevel1Command());

        configureBindings();
    }

    private void configureBindings() {
        // Elevator Commands

        controllerJoystick.leftTrigger().whileTrue(new MoveElevator(false));
        controllerJoystick.leftTrigger().onFalse(new StopElevator());

        controllerJoystick.rightTrigger().whileTrue(new MoveElevator(true));
        controllerJoystick.rightTrigger().onFalse(new StopElevator());

        controllerJoystick.a().onTrue(new L4()); 

        controllerJoystick.x().onTrue(new Drop());

        // Arm commands
        
        controllerJoystick.leftBumper().whileTrue(new MoveArm(false));
        controllerJoystick.leftBumper().onFalse(new StopElevator());

        controllerJoystick.rightBumper().whileTrue(new MoveArm(true));
        controllerJoystick.rightBumper().onFalse(new StopElevator());

        // Drive Commands

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive
                                                                                                         // forward with
                                                                                                         // negative Y
                                                                                                         // (forward)
                        .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                          // negative X (left)
                ));

        driverJoystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Zero out
        driverJoystick.b().onTrue(
                drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(0, 0))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);





        // Limelight Align Commands
        // controllerJoystick.a().onTrue(
        // vision.alignCommand(drivetrain)
        // );

        // Climber Commands
        driverJoystick.x().whileTrue(climber.moveWenchUp());
        driverJoystick.x().onFalse(climber.stopWenchCommand());
        driverJoystick.y().whileTrue(climber.moveWenchDown());
        driverJoystick.y().onFalse(climber.stopWenchCommand());

        // Algae Bar Commands
        // controllerJoystick.a().whileTrue(algae.moveElevatorUpCommand());
        // controllerJoystick.a().onFalse(algae.holdElevatorPositionCommand());

        // controllerJoystick.b().whileTrue(algae.moveElevatorDownCommand());
        // controllerJoystick.b().onFalse(algae.holdElevatorPositionCommand());

        // controllerJoystick.x().whileTrue(algae.moveArmCommand());
        // controllerJoystick.x().onFalse(algae.stopArm());

        // controllerJoystick.y().whileTrue(algae.dropAlgaeCommand());
        // controllerJoystick.y().onFalse(algae.stopArm());

        // controllerJoystick.x().whileTrue(claw.dropCoralCommand());

        // controllerJoystick.b().onTrue(claw.dropCoralCommand());
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("New Auto");
    }
}
