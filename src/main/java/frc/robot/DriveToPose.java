package frc.robot;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class DriveToPose extends CommandBase{
	DriveSubsystem drive;
	Pose2d pose;
	public DriveToPose(DriveSubsystem drive, Pose2d pose){
		addRequirements(drive);
		this.drive=drive;
		this.pose=pose;
	}
	double maxSpeed=.5;
	double maxRot=2;
	public void execute(){
		double dx=drive.getPose().getX()-pose.getX();
		double dy=drive.getPose().getY()-pose.getY();
		double dt=drive.getPose().getRoatiation().getRadians()-pose.getRoatiation().getRadians();
		double speedX=MathUtil.clamp(dx,-maxSpeed,maxSpeed);
		double speedY=MathUtil.clamp(dy,-maxSpeed,maxSpeed);
		double speedT=Math.Util.clamp(dt,-maxRot,maxRot);
		drive.drive(speedX,speedY,0,true,true);
	}
	public boolean isFinished(){
		double dx=drive.getPose().getX()-pose.getX();
		double dy=drive.getPose().getY()-pose.getY();
		double dt=drive.getPose().getRoatiation().getRadians()-pose.getRoatiation().getRadians();
		return Math.sqrt(dx*dx+dy*dy)<.25&&dt<Math.PI/16;
	}
}