package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;
import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {
    
    public static record Hardware (
        SparkMax climbMotor
    ) {}
    
    // CCW (positive) unspools, ideally
    static final Dimensionless ARM_IN_SPEED = Percent.of(-30); // todo check this
    static final Dimensionless ARM_OUT_SPEED = Percent.of(30); // todo check this

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
                getInstance().setClimbMotorToSpeed(ARM_IN_SPEED);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_climberManagementButton.getAsBoolean()) return RETRACT;
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        RETRACT {
            @Override
            public void initialize() {
                getInstance().setClimbMotorToSpeed(ARM_OUT_SPEED);
            }

            @Override
            public SystemState nextState() {
                if (getInstance().m_climberManagementButton.getAsBoolean()) return EXTEND;
                if (getInstance().m_cancelButton.getAsBoolean()) return REST;

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
        m_climbMotor.getClosedLoopController().setReference(speed.in(Value), ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
    }

    public void stopMotor() {
        m_climbMotor.stopMotor();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(getName() + "/state", getState().toString());
    }

    public void close() {
        m_climbMotor.close();
    }
}
