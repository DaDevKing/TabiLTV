package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveSim {
    private Pose2d pose = new Pose2d();

    public Pose2d getPose() {
        return pose;
    }

    public void reset(Pose2d startPose) {
        pose = startPose;
    }

    public void update(ChassisSpeeds speeds, double dt) {
        double vx = speeds.vxMetersPerSecond;
        double omega = speeds.omegaRadiansPerSecond;
        double theta = pose.getRotation().getRadians();

        double dx = vx * Math.cos(theta) * dt;
        double dy = vx * Math.sin(theta) * dt;
        double dtheta = omega * dt;

        pose = new Pose2d(
            pose.getX() + dx,
            pose.getY() + dy,
            new Rotation2d(theta + dtheta)
        );
    }
}
