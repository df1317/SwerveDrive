
package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import frc.robot.commands.AutoBalance;

public class HighCubeBalance extends SequentialCommandGroup {
  public HighCubeBalance(SwerveSubsystem m_SwerveSubsystem) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveConstants.swerveKinematics);

    TrajectoryConfig configReverse =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveConstants.swerveKinematics);
    configReverse.setReversed(true);


    // An example trajectory to follow.  All units in meters.
    Trajectory trajectoryOne =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these  interior waypoints
            List.of(new Translation2d(-1.5, 0)), 
            // End 1.5 meters straight ahead of where we started, facing forward
            new Pose2d(-6, 0, new Rotation2d(0)),
            configReverse);

    Trajectory trajectoryTwo =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(-6, 0, new Rotation2d(0)),
            // Pass through these  interior waypoints
            List.of(new Translation2d(-5, 0)), 
            // End 1.5 meters straight ahead of where we started, facing forward
            new Pose2d(-3, 0, new Rotation2d(0)),
            config);

    var concatTraj = trajectoryOne.concatenate(trajectoryTwo);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            concatTraj,
            m_SwerveSubsystem::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            m_SwerveSubsystem::setModuleStates,
            m_SwerveSubsystem);

    addCommands(
        new SequentialCommandGroup(
            new InstantCommand(() -> m_SwerveSubsystem.resetOdometry(concatTraj.getInitialPose())),
                swerveControllerCommand),
            new AutoBalance(m_SwerveSubsystem));
  }
}