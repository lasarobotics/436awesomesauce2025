package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem;

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
                return getInstance().nextState;
            }
        },
        EXTEND {
            @Override
            public void initialize() {
                getInstance().extendMotor();
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        },
        RETRACT {
            @Override
            public void initialize() {
                getInstance().retractMotor();
            }

            @Override
            public SystemState nextState() {
                return getInstance().nextState;
            }
        }
    }

    private static ClimbSubsystem s_climbSubsystemInstance;
    private final SparkMax m_climbMotor;
    private ClimbSubsystemStates nextState;

    public static ClimbSubsystem getInstance() {
        if (s_climbSubsystemInstance == null) {
            s_climbSubsystemInstance = new ClimbSubsystem(ClimbSubsystem.initializeHardware());
        }
        return s_climbSubsystemInstance;
    }

    public ClimbSubsystem(Hardware hardware) {
        super(ClimbSubsystemStates.REST);
        this.nextState = ClimbSubsystemStates.REST;
        this.m_climbMotor = hardware.climbMotor;
    }

    public static Hardware initializeHardware() {
        Hardware coralSubsystemHardware = new Hardware(
            new SparkMax(Constants.ClimbHardware.ARM_MOTOR_ID, MotorType.kBrushless) // yeah it's probably brushless TODO check
        );
        return coralSubsystemHardware;
    }

    public void setState(ClimbSubsystemStates state) {
        nextState = state;
    }

    public void stopMotor() {
        m_climbMotor.stopMotor();
    }

    public void extendMotor() {
        // set arm to a position here? idk how to do that TODO
    }

    public void retractMotor() {
        // set arm to a position here? idk how to do that TODO
    }

    public boolean armExtended() {
        return true; // update later TODO
    }

    public boolean armRetracted() {
        return true; // update later TODO
    }

    public void close() {
        m_climbMotor.close();
    }
}
