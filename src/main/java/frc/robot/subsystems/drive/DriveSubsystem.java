package frc.robot.subsystems.drive;

import java.io.File;
import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.lasarobotics.fsm.SystemState;
import org.littletonrobotics.junction.Logger;

import com.studica.frc.AHRS;

import org.lasarobotics.fsm.StateMachine;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

    private final MAXSwerve swerveDrive;

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
    // private SwerveManager m_swerveManager;
    private DoubleSupplier m_leftX;
    private DoubleSupplier m_leftY;
    private DoubleSupplier m_rightX;
    private BooleanSupplier m_reset;

    public static DriveSubsystem getInstance() {
        if (s_driveSubsystemInstance == null) {
            s_driveSubsystemInstance = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        return s_driveSubsystemInstance;
    }

    public DriveSubsystem(
            File directory) {
        super(DriveSubsystemStates.TELEOP);
        swerveDrive = new MAXSwerve();
        // m_swerveManager = new SwerveManager(directory);
    }

    public void configureBindings(
            DoubleSupplier leftX,
            DoubleSupplier leftY,
            DoubleSupplier rightX,
            BooleanSupplier reset) {
        m_leftX = leftX;
        m_leftY = leftY;
        m_rightX = rightX;
        m_reset = reset;
    }

    public void drive() {
        Logger.recordOutput("DriveSubsystem/Inputs/LeftX", m_leftX.getAsDouble());
        Logger.recordOutput("DriveSubsystem/Inputs/LeftY", m_leftY.getAsDouble());
        Logger.recordOutput("DriveSubsystem/Inputs/RightX", m_rightX.getAsDouble());
        // m_swerveManager.driveSecond(m_leftX, m_leftY, m_rightX);
        
        swerveDrive.drive(
            m_leftX.getAsDouble() * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            m_leftY.getAsDouble() * Constants.DriveConstants.kMaxSpeedMetersPerSecond * Constants.Swerve.TRANSLATION_SCALE,
            m_rightX.getAsDouble() * Constants.DriveConstants.kMaxAngularSpeed,
            true
        );
    }

    public void periodic() {
        if (m_reset.getAsBoolean())
            zeroGyro();
        swerveDrive.periodic();
        Logger.recordOutput("DriveSubsystem/pose", swerveDrive.getPose());
        Logger.recordOutput("DriveSubsystem/heading", swerveDrive.getHeading());
    }

    public void zeroGyro() {
        // m_swerveManager.zeroGyro();

        swerveDrive.zeroHeading();
    }

    public void close() {}
}
