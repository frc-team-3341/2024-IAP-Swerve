package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveAutonomousCMD extends Command{
    private final SwerveDrive swerve;

    public SwerveAutonomousCMD(SwerveDrive swerve, boolean setAlliance){
        this.swerve = swerve;
        this.addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        // this.swerve.drive(new Translation2d(correctedX, correctedY), 0, 0, true, false);
        Pose2d pos = this.swerve.getPoseFromEstimator();
        SmartDashboard.putNumber("getx", pos.getX());
        SmartDashboard.putNumber("gety", pos.getY());
        SmartDashboard.putNumber("pose.getRotation().degrees", pos.getRotation().getDegrees());
        SmartDashboard.putNumber("translation.getX", pos.getTranslation().getX());
        SmartDashboard.putNumber("translation.getY", pos.getTranslation().getY());
        SmartDashboard.putNumber(".getTranslation().getAngle().getDegrees()", pos.getTranslation().getAngle().getDegrees());
        
    }
    @Override
    public void end(boolean interrupted) {
        
        this.swerve.drive(new Translation2d(0, 0), 0, true, false);

        // PLEASE SET THIS FOR SAFETY!!!
        this.swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
