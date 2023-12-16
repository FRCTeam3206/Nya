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
public class SwerveMovementCommandGenerator{
	private static DriveSubsystem drive;
	private static TrajectoryConfig config=new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
	private static ProfiledPIDController thetaController=new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
	{thetaController.enableContinuousInput(-Math.PI, Math.PI);}
	public static void setDrive(DriveSubsystem drive){SwerveMovementCommandGenerator.drive=drive;}
        /**
         * 
         * @param trajectory The trajectory for the robot to follow.
         * @return Swerve drive command from trajectory.
         */
	public static Command fromTrajectory(Trajectory trajectory){
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
	public static Command fromPoints(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end){
		return fromTrajectory(TrajectoryGenerator.generateTrajectory(start,interiorWaypoints,end,config));
	}
	public static Command fromJSON(String path) throws IOException{
		return fromTrajectory(TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(path)));
	}
	
}
