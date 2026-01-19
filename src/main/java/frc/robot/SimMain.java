package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SimMain {

    // Simple differential drive LTV simulation
    static class LTVController {
        // LTV gains for a simple forward + rotation controller
        double kx = 1.0;   // X position gain
        double ky = 1.0;   // Y position gain
        double ktheta = 1.0; // Heading gain

        Pose2d target;

        public LTVController(Pose2d target) {
            this.target = target;
        }

        public ChassisSpeeds calculate(Pose2d current) {
            double ex = target.getX() - current.getX();
            //double ey = target.getY() - current.getY(); // also not used in diff drive
            double etheta = target.getRotation().getRadians() - current.getRotation().getRadians();

            double vx = kx * ex;
            //double vy = ky * ey; //not used in diff drive but left for reference
            double omega = ktheta * etheta;

            return new ChassisSpeeds(vx, 0, omega);
        }

        public boolean atTarget(Pose2d current) {
            return current.getTranslation().getDistance(target.getTranslation()) < 0.05;
        }
    }

    public static void main(String[] args) {
        DriveSim sim = new DriveSim();
        sim.reset(new Pose2d());

        Pose2d target = new Pose2d(2.0, 1.0, new Rotation2d(0));
        LTVController controller = new LTVController(target);

        for (int i = 0; i < 250; i++) {
            ChassisSpeeds speeds = controller.calculate(sim.getPose());
            sim.update(speeds, 0.02);

            Pose2d pose = sim.getPose();
            System.out.printf(
                "Time: %.2fs | Pose: X=%.2f, Y=%.2f, Rot=%.2f deg%n",
                i * 0.02,
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees()
            );

            if (controller.atTarget(pose)) {
                System.out.println("Target reached!");
                break;
            }
        }
    }
}
