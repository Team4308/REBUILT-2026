package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;


// Compatability/testing swerve skeleton

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive = null; 
    private final Field2d field = new Field2d();
    private Pose2d fakePose = new Pose2d();

    public SwerveSubsystem() {
        SmartDashboard.putData("Field", field);
    }

    // required
    public SwerveDrive getSwerveDrive() { 
        return swerveDrive; 
    }

    // integrate with actual pose estimate
    public Pose2d getPose() { 
        return fakePose; 
    }

    // integrate with actual swerve swerveDrive.addVisionMeasurement
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        this.fakePose = pose;
        field.setRobotPose(pose);
    }

    @Override
    public void periodic() {
        field.setRobotPose(fakePose);
    }
}