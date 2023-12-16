package frc.robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

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
	public static Command fromJSONManual(String path) throws IOException{
		//System.out.println("Running what we want 6");
		Pose2d start=null;
		Pose2d finish=null;
		ArrayList<Translation2d> waypoints=new ArrayList<>();
		boolean first=true;
		String input=Files.readString(Paths.get(path));
                input=input.replaceAll(",", "");
                Scanner scan=new Scanner(input);
                for(int i=0;i<6;i++)scan.nextLine();
                boolean done=false;
                while(scan.hasNextLine()){
                        double rotation=Double.parseDouble(scan.nextLine().substring(11));
                        scan.nextLine();
                        scan.nextLine();
                        double x=Double.parseDouble(scan.nextLine().substring(5));
                        double y=Double.parseDouble(scan.nextLine().substring(5));
                        for(int i=0;i<10;i++){
                                if(!scan.hasNextLine()){
                                        done=true;
                                        break;
                                }
                                scan.nextLine();
                        }
                        if(first){
				start=new Pose2d(x,y,new Rotation2d(rotation));
				first=false;
                //System.out.println(start);
			}else if(!done){
				//we know there are more points
				waypoints.add(new Translation2d(x,y));
				//System.out.println(new Translation2d(x,y));
			}else{
				//there are no more numbers left, this is the last pose
				finish=new Pose2d(x,y,new Rotation2d(rotation));
				//System.out.println(finish);
			}
                }
		
                
                
		return fromPoints(start,waypoints,finish);
	}
	public static Command fromPathFile(String path) throws IOException{
		Pose2d start=null;
		Pose2d finish=null;
		ArrayList<Translation2d> waypoints=new ArrayList<>();
		boolean first=true;
		Scanner scan=new Scanner(new File(path));
		scan.nextLine();
		while(scan.hasNextLine()){
			Scanner lineScan=new Scanner(scan.nextLine()).useDelimiter(",");
			double x=lineScan.nextDouble();
			double y=lineScan.nextDouble();
			double tx=lineScan.nextDouble();
			double ty=lineScan.nextDouble();
			double rotation;
			if(Math.abs(tx)<Constants.EPSILON){
				rotation=Math.PI/2;
			}
			rotation=Math.atan(ty/tx);
			if(tx<-Constants.EPSILON){
				rotation+=Math.PI;
			}
			if(first){
				start=new Pose2d(x,y,new Rotation2d(rotation));
				//System.out.println(start);
				first=false;
			}else if(scan.hasNextLine()){
				//we know there are more points
				waypoints.add(new Translation2d(x,y));
				//System.out.println(new Translation2d(x,y));
			}else{
				//there are no more numbers left, this is the last pose
				finish=new Pose2d(x,y,new Rotation2d(rotation));
				//System.out.println(finish);
			}
		}
		return fromPoints(start,waypoints,finish);
	}
}
