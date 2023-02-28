package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.parkingZoneFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Blocker Red Right Score", group = "competition")   //Telemtry allows us in the driver hub quickly chose which program to run
public class redRightDefense extends LinearOpMode {
//extends linearOpMode allows us to call functions from other helper classes
    private final Pose2d startPose = new Pose2d(36, -64.25, Math.toRadians(90)); // our Starting pose allows us to know our postions of the robot and know what way it os looking at
    // later be called in our first trajectories

    //score pose is the x and y that our IMU tries to go too and the engocders goathers position data. The heading should looking at the nearest high junction
    private final Pose2d scorePose = new Pose2d(40, -11, Math.toRadians(141));
    private final Pose2d preScorePose = new Pose2d(28, -8, Math.toRadians(120));

    // stack pose is what we point to when calling in our function so that we dont have to constantly put the same code in different trajectories. Stack pose
    // just slightly changes the postion of the robot while mainly just being a turn the change is y is for allowing the trjectory to build properly
    private final Pose2d stackPose = new Pose2d(40, -10, Math.toRadians(5));
// restrictions both in m/s
    private final double travelSpeed = 15.0, travelAccel = 15.0;
// the three different parking locations in poses
    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(64, -15, Math.toRadians(90))};
// camera images sizes 1280 pixles
    private final int width = 1280, height = 720;
// creates a dive object allows us to map funtions to our moters
    SampleMecanumDrive drive;

    // this is the microsoft life cam 3000
    OpenCvWebcam adjustCamera = null;

    // this is just our pipeline creating the filter of color on the signal sleeve
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone;

    @Override
    public void runOpMode() throws InterruptedException {   //when we start to run
        drive = new SampleMecanumDrive(hardwareMap);  // maps our moters to the robot

        // Initialize arm
        drive.initArm();

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts
// this is a trajectory we are telling the robot when goToStack is called to go from our stack pose to the score pose in a spline that looks like an s
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(35, -23), Math.toRadians(110),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel))
                .splineTo(new Vector2d(230, -12), Math.toRadians(120),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel))
                .build();

        // Set up the webcam
        WebcamName adjustCameraName = hardwareMap.get(WebcamName.class, "adjustCamera");
        adjustCamera = OpenCvCameraFactory.getInstance().createWebcam(adjustCameraName);

        // Set the camera's pipeline
        adjustCamera.setPipeline(parkingZonePipeline);

        // Open the camera
        adjustCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                adjustCamera.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        while (opModeInInit()) {
            zone = parkingZonePipeline.getParkingZone();
            telemetry.addData("Parking Zone", zone);
            telemetry.update();
        }

        adjustCamera.stopStreaming();
        adjustCamera.closeCameraDevice();

        drive.followTrajectorySequence(goToStack);

        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location

        // Update roadrunner's idea of where the robot is after we ran the trajectory
        drive.updatePoseEstimate();

        for (int i = 5; i > 4; i--) {
            toStack(drive, i);
            scoreCone(drive, i);
        }
// this is a qick if else statements that just calls the parkbot fucntion that just parks our bot using the zone from a list of the different parking postion
        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }
    }

    private void toStack(SampleMecanumDrive _drive, int stackHeight ) {
        Pose2d waypoint = new Pose2d(24, -12, Math.toRadians(90));
        sleep(500);
        _drive.updatePoseEstimate();
        if (stackHeight == 5) {
            waypoint = _drive.getPoseEstimate();
        }
        Trajectory goToStack = _drive.trajectoryBuilder(waypoint)
                .splineToLinearHeading(waypoint, Math.toRadians(160),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel))
                .build();

        _drive.followTrajectory(goToStack);
    }

    private void scoreCone(SampleMecanumDrive _drive, int stackHeight) {
        sleep(500);
        _drive.updatePoseEstimate();
        Trajectory goToScore = _drive.trajectoryBuilder(new Pose2d(55, -12, Math.toRadians(0)))
                .splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(0)), Math.toRadians(20),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel))
                .build();

        _drive.followTrajectory(goToScore);
    }

    // This code parks our robot using a list of locations and zones. If you were to use encoders
    // it would work a bit differently, but i hope this helps
    private void parkBot(SampleMecanumDrive _drive, int _zone, Pose2d[] locations) {
        _drive.updatePoseEstimate();
        Trajectory moveToPark = _drive.trajectoryBuilder(_drive.getPoseEstimate())
                .lineToLinearHeading(locations[_zone])
                .build();

        _drive.setGrip(false);
        _drive.setExtension(50);
        _drive.setHeight(4400);

        _drive.followTrajectory(moveToPark);

        _drive.setHeight(100);
    }
}
