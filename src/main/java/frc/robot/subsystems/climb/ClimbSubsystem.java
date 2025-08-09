package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;
import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {
    
    public static record Hardware (
        SparkMax climbMotor
    ) {}
    
    // CCW (positive) unspools, ideally (IT DEPENDS)
    static final Dimensionless ARM_IN_SPEED = Percent.of(-100); // todo check this
    static final Dimensionless ARM_OUT_SPEED = Percent.of(100); // todo check this

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
                if (getInstance().m_climberManagementButton.getAsBoolean()) return EXTEND;
                return this;
            }
        },
        EXTEND {
            Timer m_deadbandTimer = new Timer();
            Timer m_delayTimer = new Timer();

            @Override
            public void initialize() {
                getInstance().setClimbMotorToSpeed(ARM_IN_SPEED);

                m_deadbandTimer.reset();
                m_delayTimer.reset();
                m_deadbandTimer.start();
                m_delayTimer.start();
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_climberManagementButton.getAsBoolean()
                    && m_deadbandTimer.hasElapsed(Constants.ClimbHardware.DEADBAND_TIME)) return RETRACT;
                if (getInstance().m_cancelButton.getAsBoolean()
                    || m_delayTimer.hasElapsed(Constants.ClimbHardware.END_TIMEOUT)) return REST;

                return this;
            }
        },
        RETRACT {
            Timer m_deadbandTimer = new Timer();
            Timer m_delayTimer = new Timer();

            @Override
            public void initialize() {
                getInstance().setClimbMotorToSpeed(ARM_OUT_SPEED);

                m_deadbandTimer.reset();
                m_delayTimer.reset();
                m_deadbandTimer.start();
                m_delayTimer.start();
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_climberManagementButton.getAsBoolean()
                    && m_deadbandTimer.hasElapsed(Constants.ClimbHardware.DEADBAND_TIME)) return EXTEND;
                if (getInstance().m_cancelButton.getAsBoolean()
                    || m_delayTimer.hasElapsed(Constants.ClimbHardware.END_TIMEOUT)) return REST;

                return this;
            }
        }
    }

    private static ClimbSubsystem s_climbSubsystemInstance;
    private final SparkMax m_climbMotor;
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
        m_climbConfig.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed);
        m_climbConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);
        m_climbConfig.idleMode(IdleMode.kBrake);
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

    public void setClimbMotorToSpeed(Dimensionless speed) {
        m_climbMotor.set(speed.in(Value));
        Logger.recordOutput(getName() + "/setSpeed", speed);
    }

    public void stopMotor() {
        m_climbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/state", getState().toString());
        Logger.recordOutput(getName() + "/voltage", getInstance().m_climbMotor.getBusVoltage());
        Logger.recordOutput(getName() + "/spark", getInstance().m_climbMotor.get());
        Logger.recordOutput(getName() + "/forwardLimit", m_climbMotor.getForwardLimitSwitch().isPressed());
        Logger.recordOutput(getName() + "/reverseLimit", m_climbMotor.getReverseLimitSwitch().isPressed());
    }

    public void close() {
        m_climbMotor.close();
    }
}
