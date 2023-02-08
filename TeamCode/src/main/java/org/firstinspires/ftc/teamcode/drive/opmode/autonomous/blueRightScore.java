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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Blue Right Score")
public class blueRightScore extends LinearOpMode {

    private final Pose2d startPose = new Pose2d(36, -64.25, Math.toRadians(90));
    private final Pose2d scorePose = new Pose2d(41.5, -12, Math.toRadians(90));
    private final Pose2d stackPose = new Pose2d(44.5, -12, Math.toRadians(0));

    private final double maxVel = 50, maxAccel = 50;
    private final double maxAngVel = Math.toRadians(120), maxAngAccel = Math.toRadians(90);

    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(60, -17, Math.toRadians(90))};

    private final int width = 1280, height = 720;

    SampleMecanumDrive drive;
    OpenCvWebcam adjustCamera = null;
    parkingZoneFinder parkingZonePipeline = new parkingZoneFinder();
    parkingZoneFinder.parkingZone zone;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        // Initialize arm
        drive.initArm();

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        // Create the first trajectory to be run when the round starts

        TrajectorySequence goToScore = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(scorePose.vec(),
                        SampleMecanumDrive.getVelocityConstraint(maxVel, maxAngVel, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                )
                .turn(Math.toRadians(55), Math.toRadians(120), Math.toRadians(90))
                .build();

        TrajectorySequence scoreToStack = drive.trajectorySequenceBuilder(new Pose2d(scorePose.vec(), Math.toRadians(145)))
                .lineToSplineHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(maxVel, maxAngVel, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
                )
                .build();

        TrajectorySequence stackToScore = drive.trajectorySequenceBuilder(stackPose)
                .lineToSplineHeading(new Pose2d(scorePose.vec(), Math.toRadians(145)),
                        SampleMecanumDrive.getVelocityConstraint(maxVel, maxAngVel, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(maxAccel)
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
        drive.setExtension(670);

        drive.followTrajectorySequence(goToScore);

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

        for (int i = 5; i > 2; i--) {
            toStack(drive, scoreToStack,i);
            scoreCone(drive, stackToScore, i);
        }

        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }
    }

    private void toStack(SampleMecanumDrive _drive, TrajectorySequence trajSeq, int stackHeight) {
        // stackHeight is given as height of stack in cones
        //step one
        _drive.setExtension(500);
        sleep(200);

        _drive.followTrajectorySequence(trajSeq);
        _drive.setExtension(1800);
        sleep(750);
        _drive.setGrip(true);
        sleep(450);
        //end of step two
        //start of step three
        _drive.setHeight(4100);
        sleep(350);
        //Start of step four
        _drive.setExtension(700);
    }

    private void scoreCone(SampleMecanumDrive _drive, TrajectorySequence trajSeq, int stackHeight) {

        _drive.setHeight(4100);

        _drive.followTrajectorySequence(trajSeq);

        _drive.setExtension(650);

        // Wait for wiggles to stop just in case
        sleep(250);

        // Open grip to drop cone
        _drive.setGrip(false);
        sleep(250);
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
