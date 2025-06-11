package frc.team10505.robot.subsystems;
import com.ctre.phoenix6.Utils;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team10505.robot.Constants.HardwareConstants.*;
import static frc.team10505.robot.Constants.AlgaeConstants.*;


public class AlgaeSubsystem extends SubsystemBase {
 // Variables
    private double startingAngle = -90;
    private SparkMaxConfig pivotMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();
    private PIDController pivotController;
    private ArmFeedforward pivotFeedForward;
    private double simSpeed = 0;

    
    //Motor Definitions
    private SparkMax pivotMotor = new SparkMax(ALGAE_PIVOT_MOTOR_ID, MotorType.kBrushless);
    private SparkMax intakeMotor = new SparkMax(ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);


    //Constants
    private final int kPivotMotorCurrentLimit = 1;
    private final double pivotEncoderScale = 1;
    private double pivotSetpoint = 0;
    private double intakeSpeed = 0;
    public boolean coasting = false;
    private double absoluteOffset = 180;
    private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();

    // More Sim Variables
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), 80,
            SingleJointedArmSim.estimateMOI(0.305, 2),
            0.305, Units.degreesToRadians(-110), Units.degreesToRadians(110), true, startingAngle);

    // Motor Controllers
    private SparkMaxConfig IntakeMotorConfig = new SparkMaxConfig();

    public double getEffort() {
        return pivotFeedForward
                    .calculate(Units.degreesToRadians(getPivotEncoder()), 0)
                    + pivotController.calculate(getPivotEncoder(), pivotSetpoint);
    }

    public double getPivotEncoder() {
        return (-pivotEncoder.getPosition() + absoluteOffset);

    public Command setAngle(double Angle) {
        return run(() -> {
            pivotSetpoint = Angle;
        });
    }

    public Command setVoltage(double Voltage) {
        return run(() -> {
            pivotMotor.setVoltage(Voltage);
        });
    }

    public Command runIntake(double speed) {
        if (Utils.isSimulation()) {
            return runEnd(() -> {
                simSpeed = speed;
            }, () -> {
                simSpeed = 0;
            });
        } else {
            return runEnd(() -> {
                intakeMotor.set(speed);
            }, () -> {
                intakeMotor.set(0);
            });
        }
    }

    // Constructor
    public AlgaeSubsystem() {        
        /* Pivot Configurator */
        pivotMotorConfig.idleMode(IdleMode.kBrake);
        pivotMotorConfig.smartCurrentLimit(kPivotMotorCurrentLimit, kPivotMotorCurrentLimit);
        pivotMotorConfig.absoluteEncoder.positionConversionFactor(pivotEncoderScale);

        if (Utils.isSimulation()) {
            pivotController = new PIDController(0, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0, 0, 0);
        } else {
            pivotController = new PIDController(0.1, 0, 0);
            pivotFeedForward = new ArmFeedforward(0, 0.08, 0, 0);
        }
    }

    // Periodic Stuff
    @Override
    public void periodic() {
        // dashboard stuff
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Pivot Encoder", getPivotEncoder());
        SmartDashboard.putNumber("Pivot Calculated Effort", getEffort());
        SmartDashboard.putNumber("Algae Intake Speed", intakeSpeed);

        // Sim Updating
        pivotMotor.setVoltage(getEffort());
            SmartDashboard.putNumber("Algae Intake Motor Output", intakeMotor.getAppliedOutput());
            SmartDashboard.putNumber("Pivot Motor Output", pivotMotor.getAppliedOutput());
            SmartDashboard.putNumber("pivotEncoder", getPivotEncoder());
            pivotMotor.getMotorTemperature();
            pivotMotor.getAppliedOutput();
    }
}

