package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class CoralSubsystem extends StateMachine implements AutoCloseable {

    public static record Hardware (
        SparkMax coralMotor,
        SparkMax armMotor
    ) {}

    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100);
    static final Dimensionless SCORE_MOTOR_SPEED = Percent.of(-100);
    static final Dimensionless ARM_RETRACT_SPEED = Percent.of(30); // todo check this

    public enum CoralSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        AUTO {
            
            Timer m_autoTimer = new Timer();

            @Override
            public void initialize() {
                m_autoTimer.reset();
                m_autoTimer.start();
            }

            @Override
            public void execute() {
                // TODO make this code pretty
                if (m_autoTimer.hasElapsed(Constants.Swerve.AUTO_DRIVE_TIME + 1)) {
                    getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.STOW);
                    getInstance().stopMotor();
                } else if (m_autoTimer.hasElapsed(Constants.Swerve.AUTO_DRIVE_TIME + 0.2)) {
                    getInstance().setMotorToSpeed(SCORE_MOTOR_SPEED);
                } else if (m_autoTimer.hasElapsed(Constants.Swerve.AUTO_DRIVE_TIME)) {
                    getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.SCORE);
                }
            }

            @Override
            public SystemState nextState() {
                if (DriverStation.isAutonomous()) return this;

                return REST;
            }
        },
        REST {
            static Timer m_armZeroTimer = new Timer();

            @Override
            public void initialize() {
                getInstance().stopMotor();
                getInstance().setArmToGoBack();

                m_armZeroTimer.reset();
                m_armZeroTimer.start();
            }

            @Override
            public void execute() {
                if (m_armZeroTimer.hasElapsed(Constants.CoralArmConfig.ARM_MOTOR_DEADBAND_TIME)
                    && getInstance().armIsStalled()) {
                    getInstance().zeroRelativeEncoders();
                    getInstance().m_armMotor.stopMotor();
                    getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.STOW);
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_intakeCoralButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_intakeHighButton.getAsBoolean()) return INTAKE_HIGH;
                if (getInstance().m_scoreCoralButton.getAsBoolean()) return SCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;
                if (getInstance().m_emergencyRetractButton.getAsBoolean()) return EMERGENCY_RETRACT;

                return this;
            }
        },
        EMERGENCY_RETRACT {
            static Timer m_armZeroTimer = new Timer();

            @Override
            public void initialize() {
                getInstance().stopMotor();
                getInstance().setArmToGoBack();

                m_armZeroTimer.reset();
                m_armZeroTimer.start();
            }

            @Override
            public void execute() {
                if (m_armZeroTimer.hasElapsed(Constants.CoralArmConfig.ARM_MOTOR_DEADBAND_TIME * 2)
                    && getInstance().armIsStalled()) {
                    getInstance().zeroRelativeEncoders();
                    getInstance().m_armMotor.stopMotor();
                    getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.STOW);
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean() || (
                    m_armZeroTimer.hasElapsed(Constants.CoralArmConfig.ARM_MOTOR_DEADBAND_TIME * 2)
                    && getInstance().armIsStalled()
                )) return REST;
                if (getInstance().m_intakeCoralButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_intakeHighButton.getAsBoolean()) return INTAKE_HIGH;
                if (getInstance().m_scoreCoralButton.getAsBoolean()) return SCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        INTAKE {
            static Timer m_intaketimer = new Timer();

            @Override
            public void initialize() {
                getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.INTAKE);
                getInstance().setMotorToSpeed(INTAKE_MOTOR_SPEED);

                m_intaketimer.reset();
                m_intaketimer.start();
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeHighButton.getAsBoolean()) return INTAKE_HIGH;
                if (getInstance().m_scoreCoralButton.getAsBoolean()) return SCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                if (m_intaketimer.hasElapsed(Constants.CoralArmConfig.ROLLER_DEADBAND_TIME)
                    && getInstance().intakeIsStalled()) return REST;

                return this;
            }
        },
        INTAKE_HIGH {
            static Timer m_intaketimer = new Timer();

            @Override
            public void initialize() {
                getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.INTAKE_HIGH);
                getInstance().setMotorToSpeed(INTAKE_MOTOR_SPEED);

                m_intaketimer.reset();
                m_intaketimer.start();
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeCoralButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_scoreCoralButton.getAsBoolean()) return SCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                if (m_intaketimer.hasElapsed(Constants.CoralArmConfig.ROLLER_DEADBAND_TIME)
                    && getInstance().intakeIsStalled()) return REST;

                return this;
            }
        },
        SCORE {
            static Timer m_scoreTimer = new Timer();

            @Override
            public void initialize() {
                getInstance().sendArmToSetpoint(Constants.CoralArmSetpoints.SCORE);

                m_scoreTimer.reset();
                m_scoreTimer.start();
            }

            @Override
            public void execute() {
                if (m_scoreTimer.hasElapsed(Constants.CoralArmConfig.SCORE_EJECT_TIME)
                    || getInstance().isArmAtSetpoint(Constants.CoralArmSetpoints.SCORE)) {
                    getInstance().setMotorToSpeed(SCORE_MOTOR_SPEED);
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeCoralButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_intakeHighButton.getAsBoolean()) return INTAKE_HIGH;
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
                if (getInstance().m_intakeHighButton.getAsBoolean()) return INTAKE_HIGH;
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
    private BooleanSupplier m_intakeHighButton;
    private BooleanSupplier m_scoreCoralButton;
    private BooleanSupplier m_regurgitateButton;
    private BooleanSupplier m_emergencyRetractButton;

    public static CoralSubsystem getInstance() {
        if (s_coralSubsystemInstance == null) {
            s_coralSubsystemInstance = new CoralSubsystem(CoralSubsystem.initializeHardware());
        }
        return s_coralSubsystemInstance;
    }

    public CoralSubsystem(Hardware hardware) {
        super(CoralSubsystemStates.AUTO);
        m_coralMotor = hardware.coralMotor;
        m_armMotor = hardware.armMotor;

        m_armMotorConfig = new SparkMaxConfig();
        m_armMotorConfig
            .idleMode(IdleMode.kBrake)
            .closedLoop
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
        BooleanSupplier intakeHighButton,
        BooleanSupplier scoreCoralButton,
        BooleanSupplier regurgitateButton,
        BooleanSupplier retractButton
    ) {
        m_cancelButton = cancelButton;
        m_intakeCoralButton = intakeCoralButton;
        m_intakeHighButton = intakeHighButton;
        m_scoreCoralButton = scoreCoralButton;
        m_regurgitateButton = regurgitateButton;
        m_emergencyRetractButton = retractButton;
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

    public boolean armIsStalled() {
        return Units.Amps.of(getInstance().m_armMotor.getOutputCurrent()).gte(Constants.CoralArmHardware.ARM_STALL_CURRENT);
    }

    public boolean intakeIsStalled() {
        return Units.Amps.of(getInstance().m_coralMotor.getOutputCurrent()).gte(Constants.CoralArmHardware.ROLLER_STALL_CURRENT);
    }

    public void sendArmToSetpoint(double setpoint) {
        System.out.println("Send arm to setpoint " + setpoint);
        m_armController.setReference(
            setpoint,
            ControlType.kPosition,
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
    public void periodic() {
        Logger.recordOutput(getName() + "/state", getState().toString());
        Logger.recordOutput(getName() + "/inputs/armCurrent", m_armMotor.getAppliedOutput());
        Logger.recordOutput(getName() + "/inputs/armPosition", m_armEncoder.getPosition());
        Logger.recordOutput(getName() + "/inputs/rollerCurrent", m_coralMotor.getAppliedOutput());

    }

    @Override
    public void close() {
        m_coralMotor.close();
        m_armMotor.close();
    }
}