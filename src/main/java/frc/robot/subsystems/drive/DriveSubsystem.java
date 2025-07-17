package frc.robot.subsystems.drive;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.SystemState;
import org.lasarobotics.fsm.StateMachine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

    public enum DriveSubsystemStates implements SystemState {
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

                return TELEOP;
            }
        },
        TELEOP {
            @Override
            public void execute() {
                getInstance().drive();
            }

            @Override
            public SystemState nextState() {
                return this;
            }
        }
    }

    private static DriveSubsystem s_driveSubsystemInstance;
    private SwerveManager m_swerveManager;
    private DoubleSupplier m_leftX;
    private DoubleSupplier m_leftY;
    private DoubleSupplier m_rightX;

    public static DriveSubsystem getInstance() {
        if (s_driveSubsystemInstance == null) {
            s_driveSubsystemInstance = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        return s_driveSubsystemInstance;
    }

    public DriveSubsystem(
            File directory) {
        super(DriveSubsystemStates.AUTO);

        m_swerveManager = new SwerveManager(directory);
    }

    public void configureBindings(
            DoubleSupplier leftX,
            DoubleSupplier leftY,
            DoubleSupplier rightX) {
        m_leftX = leftX;
        m_leftY = leftY;
        m_rightX = rightX;
    }

    public void drive() {
        m_swerveManager.driveSecond(m_leftX, m_leftY, m_rightX);
    }

    public void zeroGyro() {
        m_swerveManager.zeroGyro();
    }

    public void close() {}
}
