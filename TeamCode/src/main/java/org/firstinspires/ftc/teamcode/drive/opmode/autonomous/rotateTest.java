package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.poleFinder;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.parkingZoneFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(group = "testing")
public class rotateTest extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(36, -63, Math.toRadians(90));
    private final Pose2d scorePose = new Pose2d(36, -12, Math.toRadians(90));
    private final Pose2d stackPose = new Pose2d(40, -10, Math.toRadians(5));

    private final double travelSpeed = 45.0, travelAccel = 30.0;

    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(60, -17, Math.toRadians(90))};

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(scorePose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        TrajectorySequence turnRight = drive.trajectorySequenceBuilder(stackPose)
                .turn(Math.toRadians(-90), Math.toRadians(120), Math.toRadians(90))
                .build();

        TrajectorySequence turnLeft = drive.trajectorySequenceBuilder(stackPose)
                .turn(Math.toRadians(90), Math.toRadians(120), Math.toRadians(90))
                .build();

        waitForStart();

        drive.followTrajectorySequence(goToStack);

        for (int i = 0; i < 3; i++) {
            sleep(500);
            drive.followTrajectorySequence(turnLeft);
            sleep(500);
            drive.followTrajectorySequence(turnRight);
        }
    }
}
