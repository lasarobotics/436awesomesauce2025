package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public class CoralSubsystem extends StateMachine implements AutoCloseable {

    public static record Hardware (
        SparkMax coralMotor,
        SparkMax armMotor
    ) {}

    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100); // TODO might need to change these
    static final Dimensionless SCORE_MOTOR_SPEED = Percent.of(-100);

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
                getInstance().stowArm();
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                getInstance().lowerArm();
            }

            @Override
            public void execute() {
                if (getInstance().armAtGroundPosition()) {
                    getInstance().intake();
                }
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        },
        SCORE {
            @Override
            public void initialize() {
                getInstance().raiseArm();
            }

            @Override
            public void execute() {
                if (getInstance().armAtScoringPosition()) {
                    getInstance().score();
                }
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        },
    }

    private static CoralSubsystem s_coralSubsystemInstance;
    private final SparkMax m_coralMotor;
    private final SparkMax m_armMotor;
    private final SparkClosedLoopController m_armController;
    private final RelativeEncoder m_armEncoder;
    private final SparkMaxConfig m_armMotorConfig;
    private CoralSubsystemStates nextState;

    public static CoralSubsystem getInstance() {
        if (s_coralSubsystemInstance == null) {
            s_coralSubsystemInstance = new CoralSubsystem(CoralSubsystem.initializeHardware());
        }
        return s_coralSubsystemInstance;
    }

    public CoralSubsystem(Hardware hardware) {
        super(CoralSubsystemStates.REST);
        this.nextState = CoralSubsystemStates.REST;
        this.m_coralMotor = hardware.coralMotor;
        this.m_armMotor = hardware.armMotor;

        m_armMotorConfig = new SparkMaxConfig();
        this.m_armMotorConfig.closedLoop.maxMotion
            .maxAcceleration(Constants.CoralArmHardware.MAX_ARM_ACCELERATION)
            .maxVelocity(Constants.CoralArmHardware.MAX_ARM_VELOCIY)
            .allowedClosedLoopError(Constants.CoralArmHardware.ALLOWED_CLOSED_LOOP_ERROR);
        this.m_armMotorConfig.closedLoop
            .p(Constants.CoralArmPID.P)
            .i(Constants.CoralArmPID.I)
            .d(Constants.CoralArmPID.D);
        this.m_armController = this.m_armMotor.getClosedLoopController();
        this.m_armEncoder = this.m_armMotor.getEncoder();
    }

    public static Hardware initializeHardware() {
        Hardware coralSubsystemHardware = new Hardware(
            new SparkMax(Constants.CoralArmHardware.EFFECTOR_MOTOR_ID, MotorType.kBrushless),
            new SparkMax(Constants.CoralArmHardware.ARM_MOTOR_ID, MotorType.kBrushless)
        );
        return coralSubsystemHardware;
    }

    public void setState(CoralSubsystemStates state) {
        nextState = state;
    }

    public void zeroRelativeEncoders() {
        m_armEncoder.setPosition(0.0);
    }

    public void stopMotor() {
        m_coralMotor.stopMotor();
    }

    public void intake() {
        m_coralMotor.set(INTAKE_MOTOR_SPEED.in(Value));
    }

    public void score() {
        m_coralMotor.set(SCORE_MOTOR_SPEED.in(Value));
    }
    
    public void stowArm() {
        m_armController.setReference(
            Constants.CoralArmSetpoints.STOW,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }
    
    public void lowerArm() {
        m_armController.setReference(
            Constants.CoralArmSetpoints.INTAKE,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }
    
    public void raiseArm() {
        m_armController.setReference(
            Constants.CoralArmSetpoints.SCORE,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public boolean armStowed() {
        return Math.abs(Constants.CoralArmSetpoints.STOW - m_armEncoder.getPosition()) <= Constants.EPSILON;
    }

    public boolean armAtGroundPosition() {
        return Math.abs(Constants.CoralArmSetpoints.INTAKE - m_armEncoder.getPosition()) <= Constants.EPSILON;
    }

    public boolean armAtScoringPosition() {
        return Math.abs(Constants.CoralArmSetpoints.SCORE - m_armEncoder.getPosition()) <= Constants.EPSILON;
    }

    @Override
    public void close() {
        m_coralMotor.close();
        m_armMotor.close();
    }
}