package frc.robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
public class CommandsFromPositions {
	private DriveSubsystem drive;
	private TrajectoryConfig config=new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
	private ProfiledPIDController thetaController=new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
	{thetaController.enableContinuousInput(-Math.PI, Math.PI);}
	public CommandsFromPositions(DriveSubsystem drive) {
    SwerveMovementCommandGenerator.drive = drive;
  }
  /**
   * Sets outputVolts to zero once the path is completed
   */
	private Command fromTrajectory(Trajectory trajectory){
		return new SwerveControllerCommand(
                trajectory,
                drive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                drive::setModuleStates,
                drive).andThen(() -> drive.drive(0, 0, 0, false, false));
	}
  /**
   * y is inversed
   * (0, -1): right
   * (1, 0): up
   * (0, 1): left
   * (-1, 0): down
   */
	private Command fromPoints(Pose2d start, Translation2d[] interiorWaypoints, Pose2d end){
		return fromTrajectory(TrajectoryGenerator.generateTrajectory(start, Arrays.asList(interiorWaypoints), end, config));
	}
  
  public class Position {
    boolean relative;
    double x;
    double y;
    /**
     * @param relative Whether the Position is relative to the previous Position, false means it is relative to the field.
       @param x The x value of the Position.
       @param y The y value of the Position.
     */
    public Position(boolean relative, double x, double y) {
      this.relative = relative;
      this.x = x;
      this.y = y;
    }
    public double getX() {return x;}
    public double getY() {return y;}
    public boolean isRelative() {return relative;}
    /**
     * This method should be used if it is not relative; other overloads of this method are intended for relative positions
     */
    public Translation2d getAsTranslation2d() {
      if (relative) { return new Translation2d(x + drive.getPose().getTranslation().getX(), y + drive.getPose().getTranslation().getY()); }
      return new Translation2d(x, y);
    }
    
    public Translation2d getAsTranslation2d(Position prevPosition) {
      if (!relative) { return new Translation2d(x, y); }
      return new Translation2d(prevPosition.getX() + x, prevPosition.getY() + y);
    }
    // probably won't be used
    public Translation2d getAsTranslation2d(Translation2d prevTranslation2d) {
      if (!relative) { return new Translation2d(x, y); }
      return new Translation2d(prevTranslation2d.getX() + x, prevTranslation2d.getY() + y);
    }
  }
  /**
   * @param positions The first and last Positions in this array should NOT be relative.
   */
  public Command fromPositions(Positions[] positions) {
    Translation2d[] waypoints = new int[positions.length - 2];
    for (int i = 0; i < waypoints.length; i++) {
      Position position = positions[i + 1];
      if (!position.isRelative()) {
        waypoints[i] = position.getAsTranslation2d();
      } else {
        waypoints[i] = position.getAsTranslation2d(positions[i]);
      }
    }
    return fromPoints(new Pose2d(positions[0].getAsTranslation2d(), new Rotation2d(0)), waypoints, new Pose2d(positions[positions.length - 1].getAsTranslation2d, new Rotation2d(0)))
  }

	// public static Command fromJSON(String path) throws IOException{
	// 	return fromTrajectory(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(path)));
	// }
	
}
