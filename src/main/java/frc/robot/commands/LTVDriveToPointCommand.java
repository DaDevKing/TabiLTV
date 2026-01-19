package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TabiSubsystem;
import frc.robot.subsystems.TabiSubsystem.LTVDrive;

public class LTVDriveToPointCommand extends Command {
    private final TabiSubsystem drive;
    private final LTVDrive ltv;

    public LTVDriveToPointCommand(Pose2d target, TabiSubsystem driveSubsystem) {
        drive = driveSubsystem;
        ltv = new LTVDrive(target, drive);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        ltv.execute();
    }

    @Override
    public boolean isFinished() {
        return ltv.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadedrive(0, 0); // stop the robot
    }
}
