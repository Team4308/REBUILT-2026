package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive = null; 
    private final Field2d field = new Field2d();
    private Pose2d fakePose = new Pose2d();

    public SwerveSubsystem() {
        SmartDashboard.putData("Field", field);
    }

    public SwerveDrive getSwerveDrive() { 
        return null; 
    }
    public Pose2d getPose() { 
        return fakePose; 
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        this.fakePose = pose;
        field.setRobotPose(pose);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {}

    @Override
    public void periodic() {
        field.setRobotPose(fakePose);
    }
}