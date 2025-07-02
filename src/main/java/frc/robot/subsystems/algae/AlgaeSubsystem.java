package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Value;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.measure.Dimensionless;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.subsystems.coral.CoralSubsystem.CoralSubsystemStates;

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
        NOTHING { // TODO: note that NOTHING should be provided by default, if possible
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
    }

    public static Hardware initializeHardware() {
        Hardware algaeSubsystemHardware = new Hardware(
            new SparkMax(Constants.AlgaeHardware.ARM_MOTOR_ID, MotorType.kBrushless), // yeah it's probably brushless TODO check
            new SparkMax(Constants.AlgaeHardware.INTAKE_MOTOR_ID, MotorType.kBrushless),
            new SparkMax(Constants.AlgaeHardware.SHOOTER_MOTOR_ID, MotorType.kBrushless)
        );
        return algaeSubsystemHardware;
    }

    public void setState(AlgaeSubsystemStates state) {
        nextState = state;
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
        // set arm to a position here? idk how to do that TODO
    }

    public void lowerArm() {
        // set arm to a position here? idk how to do that TODO
    }

    public void stowArm() {
        // set arm to a position here? idk how to do that TODO
    }

    public boolean isArmRaised() {
        return true; // update later TODO
    }

    public boolean isArmLowered() {
        return true; // update later TODO
    }

    public boolean isArmStowed() {
        return true; // update later TODO
    }

    @Override
    public void close() {
        m_armMotor.close();
        m_intakeMotor.close();
        m_shooterMotor.close();
    }
}
