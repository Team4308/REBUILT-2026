package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class Simulation {

    private LoggedMechanism2d mech;

    private LoggedMechanismRoot2d intakeRoot;
    private LoggedMechanismLigament2d intakeMech2d;

    private LoggedMechanismRoot2d turretRoot;
    private LoggedMechanismLigament2d turretMech2d;

    private LoggedMechanismRoot2d hoodRoot;
    private LoggedMechanismLigament2d hoodMech2d;

    private Pose3d intakePose = new Pose3d();
    private Pose3d turretPose = new Pose3d();
    private Pose3d hoodPose = new Pose3d();

    private double intakeAngle = 0.0;
    private double turretAngle = 0.0;
    private double hoodAngle = 0.0;

    public Simulation() {
        mech = new LoggedMechanism2d(10, 10);
        initIntake();
        initTurret();
        initHood();
    }

    public void simulationPeriodic() {

        double time = Timer.getFPGATimestamp();

        turretAngle = 0.5 * Math.sin(time);
        hoodAngle = 0.3 * Math.cos(time);
        intakeAngle = 0.6 * Math.sin(time);

        turretPose = new Pose3d(
            new Translation3d(0.142, 0, 0),
            new Rotation3d(0.0, 0.0, turretAngle)
        );

        hoodPose = getHoodPose(turretAngle, hoodAngle);

        updateIntake(intakeAngle);

        turretMech2d.setAngle(Math.toDegrees(turretAngle));
        hoodMech2d.setAngle(Math.toDegrees(hoodAngle));
        intakeMech2d.setAngle(Math.toDegrees(intakeAngle));

        Logger.recordOutput("RobotPose", new Pose3d()); //robot origin at 0,0,0
        Logger.recordOutput("TurretPose", turretPose);
        Logger.recordOutput("HoodPose", hoodPose);
        Logger.recordOutput("IntakePose", intakePose);

        Logger.recordOutput("Mechanism2d", mech);
    }

    public Pose3d getHoodPose(double turretAngle, double hoodAngle) {
        double xOffset = -0.028;
        double yOffset = 0.0;
        double zOffset = -0.425;

        double cos = Math.cos(turretAngle);
        double sin = Math.sin(turretAngle);
        double xGlobal = xOffset * cos - yOffset * sin;
        double yGlobal = xOffset * sin + yOffset * cos;
        double zGlobal = zOffset;

        return new Pose3d(
            xGlobal, yGlobal, zGlobal,
            new Rotation3d(0.0, hoodAngle, turretAngle)
        );
    }

    private void initIntake() {
        intakeRoot = mech.getRoot("Intake Root", 5, 0);
        intakeMech2d = intakeRoot.append(new LoggedMechanismLigament2d("Intake", 0.25, 90));
    }

    private void initTurret() {
        turretRoot = mech.getRoot("Turret Root", 5, 5);
        turretMech2d = turretRoot.append(new LoggedMechanismLigament2d("Turret", 1.0, 0));
    }

    private void initHood() {
        hoodRoot = mech.getRoot("Hood Root", 5, 6);
        hoodMech2d = hoodRoot.append(new LoggedMechanismLigament2d("Hood", 0.5, 0));
    }

    private void updateIntake(double angle) {
        intakeAngle = angle;
        intakePose = new Pose3d(
            new Translation3d(0.306, 0.0, -0.22),
            new Rotation3d(0.0, intakeAngle, 0.0)
        );
    }
}