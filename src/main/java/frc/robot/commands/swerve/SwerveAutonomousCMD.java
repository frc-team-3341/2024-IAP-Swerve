package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveAutonomousCMD extends Command {

    private final SwerveDrive swerveDriveTrain;

    public SwerveAutonomousCMD(SwerveDrive drivetrain, boolean setAlliance){
        this.swerveDriveTrain = drivetrain;
        this.addRequirements(this.swerveDriveTrain);
    }

    @Override
    public void execute(){
        //this.swerveDriveTrain.drive(new Translation2d(0, 0), 0, true, false);
        Pose2d pos = this.swerveDriveTrain.getPoseFromEstimator();
        SmartDashboard.putNumber("getx", pos.getX());
        SmartDashboard.putNumber("gety", pos.getY());
        SmartDashboard.putNumber("pose.getRotation().degrees", pos.getRotation().getDegrees());
        SmartDashboard.putNumber("translation.getX", pos.getTranslation().getX());
        SmartDashboard.putNumber("translation.getY", pos.getTranslation().getY());
        SmartDashboard.putNumber(".getTraslation().getAngles().getDegrees()", pos.getTranslation().getAngle().getDegrees());
    }


    @Override
    public void end(boolean interrupted){
        this.swerveDriveTrain.drive(new Translation2d(0, 0), 0, true, false);
        this.swerveDriveTrain.stopMotors();
    }


    @Override
    public boolean isFinished(){
        return false;
    }

}