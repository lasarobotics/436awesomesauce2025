package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import java.util.function.BooleanSupplier;
import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public class ClimbSubsystem extends StateMachine implements AutoCloseable {
    
    public static record Hardware (
        SparkMax climbMotor
     ) {}

    static final Dimensionless EXTEND_MOTOR_SPEED = Percent.of(100); // TODO might need to change these
    static final Dimensionless RETRACT_MOTOR_SPEED = Percent.of(-100);

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
                getInstance().setMotorSpeed(EXTEND_MOTOR_SPEED);
            }

            public void execute() {
                if (getInstance().isClimberAtSetpoint(Constants.ClimbMotorSetpoints.EXTEND)) {
                    getInstance().stopMotor();
                }
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
                getInstance().setMotorSpeed(RETRACT_MOTOR_SPEED);
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
    private final RelativeEncoder m_climbEncoder;
    private final SparkMaxConfig m_climbMotorController;
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
        this.m_climbMotor = hardware.climbMotor;

        this.m_climbEncoder = this.m_climbMotor.getEncoder();
        m_climbMotorController = new SparkMaxConfig();
        m_climbMotorController.closedLoop.maxMotion
            .maxAcceleration(750)
            .maxVelocity(750);
        m_climbMotorController.smartCurrentLimit((int)Constants.ClimbHardware.CLIMB_MOTOR_CURRENT_LIMIT.in(Units.Amps));
        m_climbMotor.configure(m_climbMotorController, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
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

    public void stopMotor() {
        m_climbMotor.stopMotor();
    }
    
    public void setMotorSpeed(Dimensionless speed) {
        m_climbMotor.getClosedLoopController().setReference(speed.in(Value), ControlType.kDutyCycle, ClosedLoopSlot.kSlot0, 0.0, ArbFFUnits.kVoltage);
    }

    public boolean isClimberAtSetpoint(double setpoint) {
        return Math.abs(setpoint - m_climbEncoder.getPosition()) <= Constants.EPSILON;
    }

    public void close() {
        m_climbMotor.close();
    }
}
