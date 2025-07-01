package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Dimensionless;
import frc.robot.Constants;

public class CoralSubsystem extends StateMachine implements AutoCloseable {
    public static record Hardware (
        SparkMax coralMotor,
        SparkMax armMotor
    ) {}

    static final Dimensionless INTAKE_MOTOR_SPEED = Percent.of(100); // might need to swap these two
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
                s_coralSubsystemInstance.stopMotor();
                s_coralSubsystemInstance.homeArm();
            }

            @Override
            public SystemState nextState() {
                return s_coralSubsystemInstance.nextState;
            }
        },
        INTAKE {
            @Override
            public void initialize() {
                s_coralSubsystemInstance.lowerArm();
            }

            @Override
            public void execute() {
                if (s_coralSubsystemInstance.armAtGroundPosition()) {
                    s_coralSubsystemInstance.intake();
                }
            }

            @Override
            public SystemState nextState() {
                return s_coralSubsystemInstance.nextState;
            }
        },
        SCORE_POSITION {
            @Override
            public void initialize() {
                s_coralSubsystemInstance.raiseArmToScore();

            }

            @Override
            public SystemState nextState() {
                return s_coralSubsystemInstance.nextState;
            }
        },
        SCORE {
            @Override
            public void execute() {
                if (s_coralSubsystemInstance.armAtScoringPosition()) {
                    s_coralSubsystemInstance.score();
                }
            }

            @Override
            public SystemState nextState() {
                return s_coralSubsystemInstance.nextState;
            }
        },
        REGURGITATE {
            @Override
            public void initialize() {
                // hawk tuah!
                // TODO: make this move the arm up a little bit
                s_coralSubsystemInstance.score();
            }

            @Override
            public SystemState nextState() {
                return s_coralSubsystemInstance.nextState;
            }
        }
    }

    private static CoralSubsystem s_coralSubsystemInstance;
    private final SparkMax m_coralMotor;
    private final SparkMax m_armMotor;
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
    }

    public static Hardware initializeHardware() {
        Hardware coralSubsystemHardware = new Hardware(
            new SparkMax(Constants.CoralArmHardware.EFFECTOR_MOTOR_ID, MotorType.kBrushless), // yeah it's probably brushless TODO check
            new SparkMax(Constants.CoralArmHardware.ARM_MOTOR_ID, MotorType.kBrushless)
        );
        return coralSubsystemHardware;
    }

    public void setState(CoralSubsystemStates state) {
        nextState = state;
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
    
    public void homeArm() {
        // set arm to a position here? idk how to do that TODO
    }
    
    public void lowerArm() {
        // set arm to a position here? idk how to do that TODO
    }
    
    public void raiseArmToScore() {
        // set arm to a position here? idk how to do that TODO
    }

    public boolean armAtGroundPosition() {
        return true; // update later TODO
    }

    public boolean armAtScoringPosition() {
        return true; // update later TODO
    }

    @Override
    public void close() {
        m_coralMotor.close();
    }
}