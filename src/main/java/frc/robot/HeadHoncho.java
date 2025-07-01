package frc.robot;

import java.util.function.BooleanSupplier;

import org.lasarobotics.fsm.StateMachine;
import org.lasarobotics.fsm.SystemState;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.algae.AlgaeSubsystem;
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
                if (DriverStation.isAutonomous()) {
                    return this;
                } else {
                    return REST;
                }
            }
        },
        REST {
            @Override
            public void initialize() {
                CORAL_SUBSYSTEM.setState(CoralSubsystemStates.REST);
                // ALGAE_SUBSYSTEM.setState(); // TODO algae
            }

            @Override
            public SystemState nextState() {
                if (s_IntakeCoralButton.getAsBoolean()) {
                    return INTAKE_CORAL;
                } else if (s_ScoreCoralButton.getAsBoolean()) {
                    return SCORE_CORAL;
                } else if (s_RegurgitateCoralButton.getAsBoolean()) {
                    return REGURGITATE_CORAL;
                }

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
                // TODO: make it so that when you press an algae button it goes to do that
                if (s_ScoreCoralButton.getAsBoolean()) {
                    return SCORE_CORAL;
                } else if (s_RegurgitateCoralButton.getAsBoolean()) {
                    return REGURGITATE_CORAL;
                }

                if (s_CancelButton.getAsBoolean()) {
                    return REST;
                }

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
                if (s_IntakeCoralButton.getAsBoolean()) {
                    return INTAKE_CORAL;
                } else if (s_RegurgitateCoralButton.getAsBoolean()) {
                    return REGURGITATE_CORAL;
                }

                if (s_CancelButton.getAsBoolean()) {
                    return REST;
                }

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
                // TODO: make it so that when you press an algae button it goes to do that
                if (!CoralSubsystem.getInstance().armAtScoringPosition()) {
                    return GO_TO_SCORE_CORAL;
                }

                if (s_IntakeCoralButton.getAsBoolean()) {
                    return INTAKE_CORAL;
                } else if (s_RegurgitateCoralButton.getAsBoolean()) {
                    return REGURGITATE_CORAL;
                }
                
                if (s_CancelButton.getAsBoolean()) {
                    return REST;
                }

                return this;
            }
        },
        REGURGITATE_CORAL {
            @Override
            public void initialize() {
                CORAL_SUBSYSTEM.setState(CoralSubsystemStates.REGURGITATE);
            }

            @Override
            public SystemState nextState() {
                if (s_IntakeCoralButton.getAsBoolean()) {
                    return INTAKE_CORAL;
                } else if (s_ScoreCoralButton.getAsBoolean()) {
                    return SCORE_CORAL;
                } else if (s_RegurgitateCoralButton.getAsBoolean()) {
                    return REGURGITATE_CORAL;
                }
                
                if (s_CancelButton.getAsBoolean()) {
                    return REST;
                }

                return this;
            }
        }

    }

    private static CoralSubsystem CORAL_SUBSYSTEM;
    private static AlgaeSubsystem ALGAE_SUBSYSTEM;

    private static BooleanSupplier s_CancelButton;

    private static BooleanSupplier s_IntakeCoralButton;
    private static BooleanSupplier s_ScoreCoralButton;
    private static BooleanSupplier s_RegurgitateCoralButton;

    public HeadHoncho (
        CoralSubsystem coralSubsystem,
        AlgaeSubsystem algaeSubsystem
    ) {
        super(State.REST);

        CORAL_SUBSYSTEM = coralSubsystem;
        ALGAE_SUBSYSTEM = algaeSubsystem;
    }

    public void bindControls(
        // TODO figure out driving
        BooleanSupplier cancelButton,
        BooleanSupplier intakeCoralButton,
        BooleanSupplier scoreCoralButton,
        BooleanSupplier regurgitateCoralButton
        // TODO do algae
    ) {
        s_CancelButton = cancelButton;
        s_IntakeCoralButton = intakeCoralButton;
        s_ScoreCoralButton = scoreCoralButton;
        s_RegurgitateCoralButton = regurgitateCoralButton;
    }

    public void close() {}
}
