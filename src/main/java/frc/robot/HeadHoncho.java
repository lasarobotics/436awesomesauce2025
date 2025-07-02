package frc.robot;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem.AlgaeSubsystemStates;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbSubsystemStates;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem.CoralSubsystemStates;

public class HeadHoncho extends StateMachine implements AutoCloseable {

    public enum State implements SystemState {

        NOTHING {
            @Override
            public SystemState nextState() {
                return this;
            }
        },
        AUTO {
            @Override
            public SystemState nextState() {
                if (DriverStation.isAutonomous()) return this;

                return REST;
            }
        },
        REST {
            @Override
            public void initialize() {
                CORAL_SUBSYSTEM.setState(CoralSubsystemStates.REST);
                ALGAE_SUBSYSTEM.setState(AlgaeSubsystemStates.REST);
            }

            @Override
            public SystemState nextState() {
                if (s_IntakeCoralButton.getAsBoolean()) return INTAKE_CORAL;
                if (s_ScoreCoralButton.getAsBoolean()) return SCORE_CORAL;

                if (s_IntakeAlgaeButton.getAsBoolean()) return INTAKE_ALGAE;
                if (s_DescoreAlgaeButton.getAsBoolean()) return DESCORE_ALGAE;
                if (s_ShootAlgaeButton.getAsBoolean()) return SHOOT_ALGAE;

                if (s_ClimbButton.getAsBoolean()) return RETRACT_CLIMBER;

                return this;
            }
        },
        INTAKE_CORAL {
            @Override
            public void initialize() {
                CORAL_SUBSYSTEM.setState(CoralSubsystemStates.INTAKE);
            }

            @Override
            public SystemState nextState() {
                if (s_ScoreCoralButton.getAsBoolean()) return SCORE_CORAL;

                if (s_IntakeAlgaeButton.getAsBoolean()) return INTAKE_ALGAE;
                if (s_DescoreAlgaeButton.getAsBoolean()) return DESCORE_ALGAE;
                if (s_ShootAlgaeButton.getAsBoolean()) return SHOOT_ALGAE;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        GO_TO_SCORE_CORAL {
            @Override
            public void initialize() {
                CORAL_SUBSYSTEM.setState(CoralSubsystemStates.SCORE_POSITION);
            }

            @Override
            public SystemState nextState() {
                if (s_IntakeCoralButton.getAsBoolean()) return INTAKE_CORAL;

                if (s_IntakeAlgaeButton.getAsBoolean()) return INTAKE_ALGAE;
                if (s_DescoreAlgaeButton.getAsBoolean()) return DESCORE_ALGAE;
                if (s_ShootAlgaeButton.getAsBoolean()) return SHOOT_ALGAE;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        SCORE_CORAL {
            @Override
            public void initialize() {
                CORAL_SUBSYSTEM.setState(CoralSubsystemStates.SCORE);
            }

            @Override
            public SystemState nextState() {
                if (!CoralSubsystem.getInstance().armAtScoringPosition()) return GO_TO_SCORE_CORAL;

                if (s_IntakeCoralButton.getAsBoolean()) return INTAKE_CORAL;

                if (s_IntakeAlgaeButton.getAsBoolean()) return INTAKE_ALGAE;
                if (s_DescoreAlgaeButton.getAsBoolean()) return DESCORE_ALGAE;
                if (s_ShootAlgaeButton.getAsBoolean()) return SHOOT_ALGAE;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        INTAKE_ALGAE {
            @Override
            public void initialize() {
                ALGAE_SUBSYSTEM.setState(AlgaeSubsystemStates.INTAKE);
            }

            @Override
            public SystemState nextState() {
                if (s_IntakeCoralButton.getAsBoolean()) return INTAKE_CORAL;
                if (s_ScoreCoralButton.getAsBoolean()) return SCORE_CORAL;

                if (s_DescoreAlgaeButton.getAsBoolean()) return DESCORE_ALGAE;
                if (s_ShootAlgaeButton.getAsBoolean()) return SHOOT_ALGAE;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        DESCORE_ALGAE {
            @Override
            public void initialize() {
                ALGAE_SUBSYSTEM.setState(AlgaeSubsystemStates.DESCORE);
            }

            @Override
            public SystemState nextState() {
                if (s_IntakeCoralButton.getAsBoolean()) return INTAKE_CORAL;
                if (s_ScoreCoralButton.getAsBoolean()) return SCORE_CORAL;

                if (s_IntakeAlgaeButton.getAsBoolean()) return INTAKE_ALGAE;
                if (s_ShootAlgaeButton.getAsBoolean()) return SHOOT_ALGAE;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        SHOOT_ALGAE {
            @Override
            public void initialize() {
                ALGAE_SUBSYSTEM.setState(AlgaeSubsystemStates.SHOOT);
            }

            @Override
            public SystemState nextState() {
                if (s_IntakeCoralButton.getAsBoolean()) return INTAKE_CORAL;
                if (s_ScoreCoralButton.getAsBoolean()) return SCORE_CORAL;

                if (s_IntakeAlgaeButton.getAsBoolean()) return INTAKE_ALGAE;
                if (s_DescoreAlgaeButton.getAsBoolean()) return DESCORE_ALGAE;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        EXTEND_CLIMBER {
            @Override
            public void initialize() {
                CLIMB_SUBSYSTEM.setState(ClimbSubsystemStates.EXTEND);
            }

            @Override
            public SystemState nextState() {
                if (s_ClimbButton.getAsBoolean()) return RETRACT_CLIMBER;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        },
        RETRACT_CLIMBER {
            @Override
            public void initialize() {
                CLIMB_SUBSYSTEM.setState(ClimbSubsystemStates.RETRACT);
            }

            @Override
            public SystemState nextState() {
                if (s_ClimbButton.getAsBoolean()) return EXTEND_CLIMBER;

                if (s_CancelButton.getAsBoolean()) return REST;

                return this;
            }
        }
    }

    private static CoralSubsystem CORAL_SUBSYSTEM;
    private static AlgaeSubsystem ALGAE_SUBSYSTEM;
    private static ClimbSubsystem CLIMB_SUBSYSTEM;

    private static BooleanSupplier s_CancelButton;

    private static BooleanSupplier s_IntakeCoralButton;
    private static BooleanSupplier s_ScoreCoralButton;

    private static BooleanSupplier s_IntakeAlgaeButton;
    private static BooleanSupplier s_ShootAlgaeButton;
    private static BooleanSupplier s_DescoreAlgaeButton;

    private static BooleanSupplier s_ClimbButton;

    public HeadHoncho (
        CoralSubsystem coralSubsystem,
        AlgaeSubsystem algaeSubsystem,
        ClimbSubsystem climbSubsystem
    ) {
        super(State.REST);
        CORAL_SUBSYSTEM = coralSubsystem;
        ALGAE_SUBSYSTEM = algaeSubsystem;
        CLIMB_SUBSYSTEM = climbSubsystem;
    }

    public void bindControls(
        // TODO figure out driving
        BooleanSupplier cancelButton,
        BooleanSupplier intakeCoralButton,
        BooleanSupplier scoreCoralButton,
        BooleanSupplier intakeAlgaeButton,
        BooleanSupplier shootAlgaeButton,
        BooleanSupplier descoreAlgaeButton,
        BooleanSupplier climbButton
    ) {
        s_CancelButton = cancelButton;
        s_IntakeCoralButton = intakeCoralButton;
        s_ScoreCoralButton = scoreCoralButton;
        s_IntakeAlgaeButton = intakeAlgaeButton;
        s_ShootAlgaeButton = shootAlgaeButton;
        s_DescoreAlgaeButton = descoreAlgaeButton;
        s_ClimbButton = climbButton;
    }

    public void close() {}
}
