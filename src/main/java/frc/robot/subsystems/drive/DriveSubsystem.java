package frc.robot.subsystems.drive;

import java.io.File;
import java.io.IOException;
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
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

public class DriveSubsystem extends StateMachine implements AutoCloseable {

    private final SwerveDrive swerveDrive;

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

    public static DriveSubsystem getInstance() {
        if (s_driveSubsystemInstance == null) {
            s_driveSubsystemInstance = new DriveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        }
        return s_driveSubsystemInstance;
    }

    public DriveSubsystem(
            File directory) {
        super(DriveSubsystemStates.TELEOP);



        // 4.71 : 1
        // base kit high
        // https://www.revrobotics.com/rev-21-3005/
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(4.71, 1);

        // 3 inch wheel
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(3), angleConversionFactor);

        try {
            swerveDrive = new SwerveParser(directory)
                .createSwerveDrive(
                    Constants.Swerve.MAX_SPEED,
                    angleConversionFactor,
                    driveConversionFactor);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false);
        // m_swerveManager = new SwerveManager(directory);
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
        Logger.recordOutput("DriveSubsystem/Inputs/LeftX", m_leftX.getAsDouble());
        Logger.recordOutput("DriveSubsystem/Inputs/LeftY", m_leftY.getAsDouble());
        Logger.recordOutput("DriveSubsystem/Inputs/RightX", m_rightX.getAsDouble());
        // m_swerveManager.driveSecond(m_leftX, m_leftY, m_rightX);
        
        swerveDrive.driveFieldOriented(
            new ChassisSpeeds(
                m_leftX.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * Constants.Swerve.TRANSLATION_SCALE,
                m_leftY.getAsDouble() * swerveDrive.getMaximumChassisVelocity() * Constants.Swerve.TRANSLATION_SCALE,
                m_rightX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity()
            )
        );
    }

    public void zeroGyro() {
        // m_swerveManager.zeroGyro();

        AHRS navx = (AHRS)swerveDrive.getGyro().getIMU();
        navx.zeroYaw();
    }

    public void close() {}
}
