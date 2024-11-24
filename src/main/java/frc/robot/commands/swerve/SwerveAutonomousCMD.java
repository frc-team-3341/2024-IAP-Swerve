package frc.robot.commands.swerve;

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
        
        
    }

}
