package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Percent;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Dimensionless;

public class CoralSubsystem extends StateMachine implements AutoCloseable {
    public static record Hardware (
        SparkMax coralMotor,
        SparkMax armMotor
    ) {}

    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100);
    static final Dimensionless SCORE_MOTOR_SPEED = Percent.of(100);

    public enum CoralSubsystemStates implements SystemState {
        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        IDLE {
            @Override
            public void initialize() {
                s_coralSubsystemInstance.stopMotor();
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        }
    }

    private static CoralSubsystem s_coralSubsystemInstance;
    private final SparkMax m_coralMotor;
    private final SparkMax m_armMotor;
    private CoralSubsystemStates nextState;

    private CoralSubsystem(Hardware hardware) {
        super(CoralSubsystemStates.IDLE);
        this.nextState = CoralSubsystemStates.IDLE;
        this.m_coralMotor = hardware.coralMotor;
        this.m_armMotor = hardware.armMotor;
    }

    public void stopMotor() {
        m_coralMotor.set(0.0);
    }

    @Override
    public void close() {
        m_coralMotor.close();
    }
}