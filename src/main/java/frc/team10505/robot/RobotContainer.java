package frc.team10505.robot;

import com.ctre.phoenix6.Utils;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.simulation.Simulation;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.drive.generated.TunerConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1).withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /* controllers */
    private CommandJoystick joystick;
    private CommandJoystick joystick2;

    private CommandXboxController xbox;
    private CommandXboxController xbox2;

    /* Subsystems */
    private final DrivetrainSubsystem drivetrainSubsys = TunerConstants.createDrivetrain();
    private final ElevatorSubsystem elevatorSubsys = new ElevatorSubsystem();
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    private final CoralSubsystem coralSubsys = new CoralSubsystem();
    private final Superstructure superstructure = new Superstructure(algaeSubsys, coralSubsys, elevatorSubsys,
            drivetrainSubsys);
    private final Simulation simulation = new Simulation(algaeSubsys, elevatorSubsys, coralSubsys);

    /* Sendable choosers */
    private SendableChooser<Double> polarityChooser = new SendableChooser<>();
    public SendableChooser<Command> autonChooser;

    /* Constructor */
    public RobotContainer() {
        if (RobotBase.isSimulation()) {
            joystick = new CommandJoystick(0);
            joystick2 = new CommandJoystick(1);

            simConfigButtonBindings();
        } else {
            xbox = new CommandXboxController(1);
            xbox2 = new CommandXboxController(0);

            configButtonBindings();
            elevatorButtonBindings();
        }
        drivetrainSubsys.configAutoBuilder();
        configNamedCommands();
        configSendableChoosers();
    }

    // TODO create(& call) sim config default command method (just for drivetrain
    // subsys, reference season code if need be)

    // TODO create(& call) config default command method (just for drivetrain
    // subsys, reference season code if need be)

    private void simConfigButtonBindings() {
        // TODO add stuff
    }

    private void configButtonBindings() {
        if (Utils.isSimulation()) {// TODO get rid of if/else statement. Move joystick bindings to
                                   // simConfigButtonBindings method
            joystick.button(1).onTrue(algaeSubsys.setAngle(0));
            joystick.button(2).onTrue(algaeSubsys.setAngle(-30));
            joystick.button(3).onTrue(algaeSubsys.setAngle(-90));
        } else {
            xbox.x().onTrue(algaeSubsys.setAngle(0));
            xbox.a().onTrue(algaeSubsys.setAngle(-30));
            xbox.b().onTrue(algaeSubsys.setAngle(-90));
            xbox.y().onTrue(algaeSubsys.setAngle(90));
        }

        // TODO double check directions (pov up should be a negative velocity x, I
        // think)
        xbox.povUp()
                .whileTrue(drivetrainSubsys
                        .applyRequest(() -> robotDrive.withVelocityX(-0.4).withVelocityY(0.0).withRotationalRate(0.0)))
                .onFalse(drivetrainSubsys.stop());
        xbox.povDown()
                .whileTrue(drivetrainSubsys
                        .applyRequest(() -> robotDrive.withVelocityX(0.4).withVelocityY(0.0).withRotationalRate(0.0)))
                .onFalse(drivetrainSubsys.stop());
        xbox.povLeft()
                .whileTrue(drivetrainSubsys
                        .applyRequest(() -> robotDrive.withVelocityX(0.0).withVelocityY(0.6).withRotationalRate(0.0))
                        .until(() -> !drivetrainSubsys.seesLeftSensor()));
        xbox.povRight()
                .whileTrue(drivetrainSubsys
                        .applyRequest(() -> robotDrive.withVelocityX(0.0).withVelocityY(-0.6).withRotationalRate(0.0))
                        .until(() -> !drivetrainSubsys.seesRightSensor()));

    }

    private void elevatorButtonBindings() {
        xbox2.x().onTrue(elevatorSubsys.setHeight(0));
        xbox2.a().onTrue(elevatorSubsys.setHeight(8));
        xbox2.b().onTrue(elevatorSubsys.setHeight(24));
        xbox2.y().onTrue(elevatorSubsys.setHeight(48.5));
    }

    private void coralSubsysButtonBindings() {
        xbox2.povDown().onTrue(coralSubsys.Intake());
        xbox2.povUp().onTrue(coralSubsys.outakeTop());
        xbox2.povLeft().whileTrue(coralSubsys.Outtake());
        xbox2.povRight().whileTrue(coralSubsys.trough());
    }

    private void configSendableChoosers() {
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("Negative", -1.0);
        SmartDashboard.putData("Polarity Chooser", polarityChooser);

        autonChooser = AutoBuilder.buildAutoChooser();
        autonChooser.setDefaultOption("null", Commands.print("stupid monky, be gooderer"));
        SmartDashboard.putData("Auton Chooser", autonChooser);

    }

    private void configNamedCommands() {
        // TODO add stuff here
    }

    // Pivot Controls
    private void algaePivotControls() {

    }
}
