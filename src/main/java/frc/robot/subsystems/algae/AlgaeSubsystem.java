package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Dimensionless;

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
        SparkMax shooterMotor
    ) {}

    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100); // TODO might need to change these
    static final Dimensionless DESCORE_MOTOR_SPEED = Percent.of(-100);
    static final Dimensionless SHOOTER_MOTOR_SPEED = Percent.of(100);

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
                getInstance().stowArm();
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        },
        DESCORE {
            @Override
            public void initialize() {
                getInstance().stopShooter();
                getInstance().raiseArm();
                getInstance().descore();
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                getInstance().stopShooter();
                getInstance().lowerArm();
                getInstance().intake();
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        },
        SHOOT {
            @Override
            public void initialize() {
                getInstance().stopIntake();
                getInstance().stowArm();
                getInstance().shoot();
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        }
    }

    private static AlgaeSubsystem s_algaeSubsystemInstance;
    private final SparkMax m_armMotor;
    private final SparkMax m_intakeMotor;
    private final SparkMax m_shooterMotor;
    private final SparkClosedLoopController m_armController;
    private final RelativeEncoder m_armEncoder;
    private final SparkMaxConfig m_armMotorConfig;
    private AlgaeSubsystemStates nextState;

    public static AlgaeSubsystem getInstance() {
        if (s_algaeSubsystemInstance == null) {
            s_algaeSubsystemInstance = new AlgaeSubsystem(AlgaeSubsystem.initializeHardware());
        }
        return s_algaeSubsystemInstance;
    }

    public AlgaeSubsystem(Hardware hardware) {
        super(AlgaeSubsystemStates.REST);
        this.nextState = AlgaeSubsystemStates.REST;
        this.m_armMotor = hardware.armMotor;
        this.m_intakeMotor = hardware.intakeMotor;
        this.m_shooterMotor = hardware.shooterMotor;

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
            new SparkMax(Constants.AlgaeHardware.SHOOTER_MOTOR_ID, MotorType.kBrushless)
        );
        return algaeSubsystemHardware;
    }

    public void setState(AlgaeSubsystemStates state) {
        nextState = state;
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

    public void descore() {
        m_intakeMotor.set(DESCORE_MOTOR_SPEED.in(Value));
    }

    public void intake() {
        m_intakeMotor.set(INTAKE_MOTOR_SPEED.in(Value));
    }

    public void shoot() {
        m_shooterMotor.set(SHOOTER_MOTOR_SPEED.in(Value));
    }

    public void raiseArm() {
        m_armController.setReference(
            Constants.AlgaeArmSetpoints.DESCORE,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public void lowerArm() {
        m_armController.setReference(
            Constants.AlgaeArmSetpoints.INTAKE,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public void stowArm() {
        m_armController.setReference(
            Constants.AlgaeArmSetpoints.STOW,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public boolean isArmRaised() {
        return Math.abs(Constants.AlgaeArmSetpoints.DESCORE - m_armEncoder.getPosition()) <= Constants.EPSILON;
    }

    public boolean isArmLowered() {
        return Math.abs(Constants.AlgaeArmSetpoints.INTAKE - m_armEncoder.getPosition()) <= Constants.EPSILON;
    }

    public boolean isArmStowed() {
        return Math.abs(Constants.AlgaeArmSetpoints.STOW - m_armEncoder.getPosition()) <= Constants.EPSILON;
    }

    @Override
    public void close() {
        m_armMotor.close();
        m_intakeMotor.close();
        m_shooterMotor.close();
    }
}
