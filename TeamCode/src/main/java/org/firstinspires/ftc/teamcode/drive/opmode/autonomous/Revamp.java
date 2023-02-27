package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.vision.parkingZoneFinder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous (name= "RedRightReVamp", group = "comepetition")
public class Revamp extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(35, -64.25, Math.toRadians(90)); // our Starting pose allows us to know our postions of the robot and know what way it os looking at
    // later be called in our first trajectories

    //score pose is the x and y that our IMU tries to go too and the engocders goathers position data. The heading should looking at the nearest high junction
    private final Pose2d scorePose = new Pose2d(37, -11, Math.toRadians(145));

    // stack pose is what we point to when calling in our function so that we dont have to constantly put the same code in different trajectories. Stack pose
    // just slightly changes the postion of the robot while mainly just being a turn the change is y is for allowing the trjectory to build properly
    private final Pose2d stackPose = new Pose2d(48, -13, Math.toRadians(-4));
    // restrictions both in m/s

    private final Pose2d smalljun = new Pose2d(37, -11, Math.toRadians(-4));
    private final double travelSpeed = 45.0, travelAccel = 30.0;
    // the three different parking locations in poses
    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(64, -15, Math.toRadians(90))};
    // camera images sizes 1280 pixles


    SampleMecanumDrive drive;

    private final int width = 1280, height = 720;

    OpenCvWebcam adjustCamera = null;

    // this is just our pipeline creating the filter of color on the signal sleeve
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone;

    @Override
    public void runOpMode() throws InterruptedException {   //when we start to run
        drive = new SampleMecanumDrive(hardwareMap);  // maps our moters to the robot

        // Initialize arm
        drive.initArm(telemetry);

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts
// this is a trajectory we are telling the robot when goToStack is called to go from our stack pose to the score pose in a spline that looks like an s
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(scorePose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
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

        while (!isStarted()) {
            zone = parkingZonePipeline.getParkingZone();
            telemetry.addData("Parking Zone", zone);
            telemetry.update();
        }

        adjustCamera.stopStreaming();
        adjustCamera.closeCameraDevice();

        drive.setSlideVelocity(4000, drive.slideRight, drive.slideLeft, drive.slideTop);

        // Close the grip and move the slide up a small amount
        drive.setGrip(true);
        sleep(250);
        drive.setHeight(200);
        drive.setExtension(50);

        // The sleep is necessary to wait for certain arm actions to finish
        sleep(250);

        // Increase the height of the slide and increase its velocity
        drive.setHeight(4200);
        drive.setExtension(750);

        drive.followTrajectorySequence(goToStack);

        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location

        // Update roadrunner's idea of where the robot is after we ran the trajectory
        drive.updatePoseEstimate();
        // Wait for arm to be in position
        sleep(250);

        // Open grip to drop cone
        drive.setGrip(false);

        // Wait for grip to fully open and cone to drop
        sleep(500);

        TrajectorySequence tostack = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(stackPose,
                SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(travelAccel)
        )
                .build();


        drive.followTrajectorySequence(tostack);
        drive.updatePoseEstimate();
        drive.setHeight(845);
        sleep(2000);
        drive.setGrip(true);
        sleep(2000);
        drive.setHeight(2000);
        sleep(500);

        /*

        TrajectorySequence toMed = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(smalljun,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        drive.followTrajectorySequence(toMed);
        drive.setGrip(false);



        // for liip to repeate 3 timss
        // calles totrack fuction that turns around grabs a cone and then stops
        // then calls score cone that goes from stack to target junction
        // commented out

       /* if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); } */

    }

    public void toStack(){

    }

    // mj - middle junction
    public void mj(){

    }

    //high junction
    public void hj (){

    }

    // low junction
    public void lj() {

    }

    private void parkBot(SampleMecanumDrive _drive, int _zone, Pose2d[] locations) {
        _drive.updatePoseEstimate();
        Trajectory moveToPark = _drive.trajectoryBuilder(_drive.getPoseEstimate())
                .lineToLinearHeading(locations[_zone])
                .build();

        _drive.setGrip(false);
        _drive.setExtension(50);
        _drive.setHeight(4400);
        _drive.setSlideVelocity(4000, _drive.slideLeft, _drive.slideRight, _drive.slideTop);

        _drive.followTrajectory(moveToPark);

        _drive.setHeight(100);
    }

}
