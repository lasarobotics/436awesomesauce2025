package frc.robot.subsystems.climb;

import java.util.function.BooleanSupplier;
import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {
    
    public static record Hardware (
        SparkMax climbMotor
     ) {}

    public enum ClimbSubsystemStates implements SystemState {
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
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_climberManagementButton.getAsBoolean()) return RETRACT;
                return this;
            }
        },
        EXTEND {
            @Override
            public void initialize() {
                getInstance().sendClimberToSetpoint(Constants.ClimbMotorSetpoints.EXTEND);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_climberManagementButton.getAsBoolean()) return RETRACT;
                if (getInstance().m_cancelButton.getAsBoolean() ||
                    getInstance().isClimberAtSetpoint(Constants.ClimbMotorSetpoints.EXTEND)) return REST;
                return this;
            }
        },
        RETRACT {
            @Override
            public void initialize() {
                getInstance().sendClimberToSetpoint(Constants.ClimbMotorSetpoints.RETRACT);
            }

            public void execute() {
                if (getInstance().isClimberAtSetpoint(Constants.ClimbMotorSetpoints.RETRACT)) {
                    getInstance().stopMotor();
                }
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_climberManagementButton.getAsBoolean()) return EXTEND;
                if (getInstance().m_cancelButton.getAsBoolean() ||
                    getInstance().isClimberAtSetpoint(Constants.ClimbMotorSetpoints.RETRACT)) return REST;
                return this;
            }
        }
    }

    private static ClimbSubsystem s_climbSubsystemInstance;
    private final SparkMax m_climbMotor;
    private final SparkClosedLoopController m_climbController;
    private final RelativeEncoder m_climbEncoder;
    private final SparkMaxConfig m_climbConfig;
    private BooleanSupplier m_cancelButton;
    private BooleanSupplier m_climberManagementButton;

    public static ClimbSubsystem getInstance() {
        if (s_climbSubsystemInstance == null) {
            s_climbSubsystemInstance = new ClimbSubsystem(ClimbSubsystem.initializeHardware());
        }
        return s_climbSubsystemInstance;
    }

    public ClimbSubsystem(Hardware hardware) {
        super(ClimbSubsystemStates.REST);
        m_climbMotor = hardware.climbMotor;

        m_climbConfig = new SparkMaxConfig();
        m_climbConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.ClimbPID.P,
                 Constants.ClimbPID.I,
                 Constants.ClimbPID.D)
            .maxMotion
                .allowedClosedLoopError(
                    Constants.ClimbHardware.ALLOWED_CLOSED_LOOP_ERROR
                );
        m_climbController = m_climbMotor.getClosedLoopController();
        m_climbEncoder = m_climbMotor.getEncoder();
        m_climbConfig.smartCurrentLimit((int)Constants.ClimbHardware.CLIMB_MOTOR_CURRENT_LIMIT.in(Units.Amps));
        m_climbMotor.configure(m_climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Hardware initializeHardware() {
        Hardware coralSubsystemHardware = new Hardware(
            new SparkMax(Constants.ClimbHardware.CLIMB_MOTOR_ID, MotorType.kBrushless)
        );
        return coralSubsystemHardware;
    }

    public void configureBindings(
        BooleanSupplier cancelButton,
        BooleanSupplier climberManagementButton
    ) {
        m_cancelButton = cancelButton;
        m_climberManagementButton = climberManagementButton;
    }

    public void zeroRelativeEncoders() {
        m_climbEncoder.setPosition(0.0);
    }

    public void sendClimberToSetpoint(double setpoint) {
        m_climbController.setReference(
            setpoint,
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0
        );
    }

    public boolean isClimberAtSetpoint(double setpoint) {
        return Math.abs(m_climbMotor.getAbsoluteEncoder().getPosition() - setpoint) < Constants.ClimbHardware.ALLOWED_CLOSED_LOOP_ERROR;
    }

    public void stopMotor() {
        m_climbMotor.stopMotor();
    }

    public void close() {
        m_climbMotor.close();
    }
}
