package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

public class AlgaeSubsystem extends StateMachine implements AutoCloseable {

    public static record Hardware (
        SparkMax armMotor,
        SparkMax intakeMotor,
        SparkMax shooterMotor,
        DigitalInput beamBreak
    ) {}

    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100); // TODO might need to change these
    static final Dimensionless REVERSE_INTAKE_MOTOR_SPEED = Percent.of(-100);
    static final Dimensionless DESCORE_MOTOR_SPEED = Percent.of(-100);
    static final Dimensionless SHOOTER_MOTOR_SPEED = Percent.of(100);
    static final Dimensionless REVERSE_SHOOTER_MOTOR_SPEED = Percent.of(-100);

    public enum AlgaeSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        REST {
            @Override
            public void initialize() {
                getInstance().stopIntake();
                getInstance().stopShooter();
                getInstance().sendArmToSetpoint(Constants.AlgaeArmSetpoints.STOW);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_intakeAlgaeButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_descoreAlgaeButton.getAsBoolean()) return DESCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        DESCORE {
            @Override
            public void initialize() {
                getInstance().stopShooter();
                getInstance().sendArmToSetpoint(Constants.AlgaeArmSetpoints.DESCORE);
                getInstance().setIntakeMotorSpeed(DESCORE_MOTOR_SPEED);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeAlgaeButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                getInstance().stopShooter();
                getInstance().sendArmToSetpoint(Constants.AlgaeArmSetpoints.INTAKE);
                getInstance().setIntakeMotorSpeed(INTAKE_MOTOR_SPEED);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().isBeamBrakeTripped()) {
                    return HAS_ALGAE;
                }

                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_descoreAlgaeButton.getAsBoolean()) return DESCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        HAS_ALGAE {
            @Override
            public void initialize() {
                getInstance().stopShooter();
                getInstance().sendArmToSetpoint(Constants.AlgaeArmSetpoints.STOW);
                getInstance().stopIntake();
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_shootAlgaeButton.getAsBoolean()) return SHOOT;
                if (getInstance().m_descoreAlgaeButton.getAsBoolean()) return DESCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        SHOOT {
            @Override
            public void initialize() {
                getInstance().stopIntake();
                getInstance().sendArmToSetpoint(Constants.AlgaeArmSetpoints.STOW);
                getInstance().setShooterMotorSpeed(SHOOTER_MOTOR_SPEED);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeAlgaeButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_descoreAlgaeButton.getAsBoolean()) return DESCORE;
                if (getInstance().m_regurgitateButton.getAsBoolean()) return REGURGITATE;

                return this;
            }
        },
        REGURGITATE {
            @Override
            public void initialize() {
                // spit out the front
                getInstance().setIntakeMotorSpeed(REVERSE_INTAKE_MOTOR_SPEED);
                getInstance().sendArmToSetpoint(Constants.AlgaeArmSetpoints.INTAKE);
                getInstance().setShooterMotorSpeed(REVERSE_SHOOTER_MOTOR_SPEED);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;
                if (getInstance().m_intakeAlgaeButton.getAsBoolean()) return INTAKE;
                if (getInstance().m_descoreAlgaeButton.getAsBoolean()) return DESCORE;

                return this;
            }
        }
    }

    private static AlgaeSubsystem s_algaeSubsystemInstance;
    private final SparkMax m_armMotor;
    private final SparkMax m_intakeMotor;
    private final SparkMax m_shooterMotor;
    private final DigitalInput m_beamBreak;
    private final SparkClosedLoopController m_armController;
    private final RelativeEncoder m_armEncoder;
    private final SparkMaxConfig m_armMotorConfig;
    private BooleanSupplier m_cancelButton;
    private BooleanSupplier m_intakeAlgaeButton;
    private BooleanSupplier m_shootAlgaeButton;
    private BooleanSupplier m_descoreAlgaeButton;
    private BooleanSupplier m_regurgitateButton;

    public static AlgaeSubsystem getInstance() {
        if (s_algaeSubsystemInstance == null) {
            s_algaeSubsystemInstance = new AlgaeSubsystem(AlgaeSubsystem.initializeHardware());
        }
        return s_algaeSubsystemInstance;
    }

    public AlgaeSubsystem(Hardware hardware) {
        super(AlgaeSubsystemStates.REST);
        this.m_armMotor = hardware.armMotor;
        this.m_intakeMotor = hardware.intakeMotor;
        this.m_shooterMotor = hardware.shooterMotor;
        this.m_beamBreak = hardware.beamBreak;

        m_armMotorConfig = new SparkMaxConfig();
        this.m_armMotorConfig.closedLoop.maxMotion
            .maxAcceleration(Constants.AlgaeHardware.MAX_ARM_ACCELERATION)
            .maxVelocity(Constants.AlgaeHardware.MAX_ARM_VELOCIY)
            .allowedClosedLoopError(Constants.AlgaeHardware.ALLOWED_CLOSED_LOOP_ERROR);
        this.m_armMotorConfig.closedLoop
            .p(Constants.AlgaeArmPID.P)
            .i(Constants.AlgaeArmPID.I)
            .d(Constants.AlgaeArmPID.D);
        this.m_armController = this.m_armMotor.getClosedLoopController();
        this.m_armEncoder = this.m_armMotor.getEncoder();
    }

    public static Hardware initializeHardware() {
        Hardware algaeSubsystemHardware = new Hardware(
            new SparkMax(Constants.AlgaeHardware.ARM_MOTOR_ID, MotorType.kBrushless),
            new SparkMax(Constants.AlgaeHardware.INTAKE_MOTOR_ID, MotorType.kBrushless),
            new SparkMax(Constants.AlgaeHardware.SHOOTER_MOTOR_ID, MotorType.kBrushless),
            new DigitalInput(Constants.AlgaeHardware.BEAM_BREAK_ID)
        );
        return algaeSubsystemHardware;
    }

    public void configureBindings(
        BooleanSupplier cancelButton,
        BooleanSupplier intakeAlgaeButton,
        BooleanSupplier shootAlgaeButton,
        BooleanSupplier descoreAlgaeButton,
        BooleanSupplier regurgitateButton
    ) {
        m_cancelButton = cancelButton;
        m_intakeAlgaeButton = intakeAlgaeButton;
        m_shootAlgaeButton = shootAlgaeButton;
        m_descoreAlgaeButton = descoreAlgaeButton;
        m_regurgitateButton = regurgitateButton;
    }

    public void zeroRelativeEncoders() {
        m_armEncoder.setPosition(0.0);
    }

    public void stopIntake() {
        m_intakeMotor.stopMotor();
    }

    public void stopShooter() {
        m_shooterMotor.stopMotor();
    }

    public void setShooterMotorSpeed(Dimensionless speed) {
        m_shooterMotor.set(speed.in(Value));
    }

    public void setIntakeMotorSpeed(Dimensionless speed) {
        m_intakeMotor.set(speed.in(Value));
    }

    public void sendArmToSetpoint(double setpoint) {
        m_armController.setReference(
            setpoint,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public boolean isArmAtSetpoint(double setpoint) {
        return Math.abs(setpoint - m_armEncoder.getPosition()) <= Constants.EPSILON;
    }

    public boolean isBeamBrakeTripped() {
        return m_beamBreak.get();
    }

    @Override
    public void close() {
        m_armMotor.close();
        m_intakeMotor.close();
        m_shooterMotor.close();
    }
}
