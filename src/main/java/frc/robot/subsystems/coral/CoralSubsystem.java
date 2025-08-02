package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public class CoralSubsystem extends StateMachine implements AutoCloseable {

    public static record Hardware (
        SparkMax coralMotor,
        SparkMax armMotor
    ) {}

    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100);
    static final Dimensionless SCORE_MOTOR_SPEED = Percent.of(-100);
    static final Dimensionless ARM_RETRACT_SPEED = Percent.of(5); // todo check this

    public enum CoralSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        REST {
            @Override
            public void initialize() {
                getInstance().stopMotor();
                // getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.STOW);
                getInstance().setArmToGoBack();
            }

            @Override
            public void execute() {
                if (Units.Amps.of(getInstance().m_armMotor.getOutputCurrent()).gte(Constants.CoralArmHardware.ARM_STALL_CURRENT)) {
                    getInstance().zeroRelativeEncoders();
                    getInstance().m_armMotor.stopMotor();
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_intakeCoralButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_scoreCoralButton.getAsBoolean()) return SCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.INTAKE);
            }

            @Override
            public void execute() {
                if (getInstance().isArmAtSetpoint(Constants.CoralArmSetpoints.INTAKE)) {
                    getInstance().setMotorToSpeed(INTAKE_MOTOR_SPEED);
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_scoreCoralButton.getAsBoolean()) return SCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                if (getInstance().intakeIsStalled()) return REST;

                return this;
            }
        },
        SCORE {
            @Override
            public void initialize() {
                getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.SCORE);
            }

            @Override
            public void execute() {
                if (getInstance().isArmAtSetpoint(Constants.CoralArmSetpoints.SCORE)) {
                    getInstance().setMotorToSpeed(SCORE_MOTOR_SPEED);
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeCoralButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        REGURGITATE {
            @Override
            public void initialize() {
                getInstance().setMotorToSpeed(SCORE_MOTOR_SPEED);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeCoralButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_scoreCoralButton.getAsBoolean()) return SCORE;

                return this;
            }
        },
    }

    private static CoralSubsystem s_coralSubsystemInstance;
    private final SparkMax m_coralMotor;
    private final SparkMaxConfig m_coralMotorConfig;
    private final SparkMax m_armMotor;
    private final SparkClosedLoopController m_armController;
    private final RelativeEncoder m_armEncoder;
    private final SparkMaxConfig m_armMotorConfig;
    private BooleanSupplier m_cancelButton;
    private BooleanSupplier m_intakeCoralButton;
    private BooleanSupplier m_scoreCoralButton;
    private BooleanSupplier m_regurgitateButton;

    public static CoralSubsystem getInstance() {
        if (s_coralSubsystemInstance == null) {
            s_coralSubsystemInstance = new CoralSubsystem(CoralSubsystem.initializeHardware());
        }
        return s_coralSubsystemInstance;
    }

    public CoralSubsystem(Hardware hardware) {
        super(CoralSubsystemStates.REST);
        m_coralMotor = hardware.coralMotor;
        m_armMotor = hardware.armMotor;

        m_armMotorConfig = new SparkMaxConfig();
        m_armMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.CoralArmPID.P,
                 Constants.CoralArmPID.I,
                 Constants.CoralArmPID.D)
            .maxMotion
                .allowedClosedLoopError(
                    Constants.CoralArmHardware.ALLOWED_CLOSED_LOOP_ERROR
                );
        m_armController = m_armMotor.getClosedLoopController();
        m_armEncoder = m_armMotor.getEncoder();
        m_armMotorConfig.smartCurrentLimit((int)Constants.CoralArmHardware.ARM_MOTOR_CURRENT_LIMIT.in(Units.Amps));
        m_armMotor.configure(m_armMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_coralMotorConfig = new SparkMaxConfig();
        m_coralMotorConfig.closedLoop.maxMotion
            .maxAcceleration(750)
            .maxVelocity(750);
        m_coralMotorConfig.smartCurrentLimit((int)Constants.CoralArmHardware.ROLLER_MOTOR_CURRENT_LIMIT.in(Units.Amps));
        m_coralMotor.configure(m_coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Hardware initializeHardware() {
        Hardware coralSubsystemHardware = new Hardware(
            new SparkMax(Constants.CoralArmHardware.EFFECTOR_MOTOR_ID, MotorType.kBrushless),
            new SparkMax(Constants.CoralArmHardware.ARM_MOTOR_ID, MotorType.kBrushless)
        );
        return coralSubsystemHardware;
    }

    public void configureBindings(
        BooleanSupplier cancelButton,
        BooleanSupplier intakeCoralButton,
        BooleanSupplier scoreCoralButton,
        BooleanSupplier regurgitateButton
    ) {
        m_cancelButton = cancelButton;
        m_intakeCoralButton = intakeCoralButton;
        m_scoreCoralButton = scoreCoralButton;
        m_regurgitateButton = regurgitateButton;
    }

    public void zeroRelativeEncoders() {
        m_armEncoder.setPosition(0.0);
    }

    public void stopMotor() {
        m_coralMotor.stopMotor();
    }

    public void setMotorToSpeed(Dimensionless speed) {
        m_coralMotor.getClosedLoopController().setReference(speed.in(Value), ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
    }

    public boolean intakeIsStalled() {
        return Units.Amps.of(getInstance().m_coralMotor.getOutputCurrent()).gte(Constants.CoralArmHardware.ROLLER_STALL_CURRENT);
    }

    public void sendArmToSetpoint(double setpoint) {
        m_armController.setReference(
            setpoint,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public void setArmToGoBack() {
        // CCW is positive
        m_armMotor.set(ARM_RETRACT_SPEED.in(Value));
    }

    public boolean isArmAtSetpoint(double setpoint) {
        return Math.abs(setpoint - m_armEncoder.getPosition()) <= Constants.CoralArmHardware.ALLOWED_CLOSED_LOOP_ERROR;
    }

    @Override
    public void close() {
        m_coralMotor.close();
        m_armMotor.close();
    }
}