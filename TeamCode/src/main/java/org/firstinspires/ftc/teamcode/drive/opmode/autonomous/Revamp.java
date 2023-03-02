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
@Autonomous (name= "RightMEDStack", group = "comepetition")
public class Revamp extends LinearOpMode {
    private final Pose2d startPose = new Pose2d(35, -64.25, Math.toRadians(90)); // our Starting pose allows us to know our postions of the robot and know what way it os looking at'/
    private final Pose2d stackPose = new Pose2d(48, -12, Math.toRadians(0));
    private final Pose2d lowJunction = new Pose2d(31, -12, Math.toRadians(-53));
    private final Pose2d medJunction = new Pose2d(38,-13, Math.toRadians(220));
    private final Pose2d highJunction = new Pose2d(37, -12, Math.toRadians(145));
    private final double travelSpeed = 25, travelAccel = 20;
    // the three different parking locations in poses
    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(64, -15, Math.toRadians(90))};
    // camera images sizes 1280 pixels

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
        drive.initArm();

        // Tell the robot where it is based on a pose created earlier
        drive.setPoseEstimate(startPose);

        TrajectorySequence goToPreload = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(highJunction,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        TrajectorySequence toStackFromHigh = drive.trajectorySequenceBuilder(highJunction)
                .setTangent(Math.toRadians(-25))
                .splineToLinearHeading(stackPose, Math.toRadians(15),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        TrajectorySequence toStackFromLow = drive.trajectorySequenceBuilder(lowJunction)
                .splineToLinearHeading(stackPose, Math.toRadians(-20),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        TrajectorySequence toLowFromStack = drive.trajectorySequenceBuilder(stackPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(lowJunction, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        // Create the first trajectory to be run when the round starts
// this is a trajectory we are telling the robot when goToPreload is called to go from our stack pose to the score pose in a spline that looks like an s

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

        drive.followTrajectorySequence(goToPreload);




        // Without waiting, run the trajectory we prepared earlier
        // This will take us to our cycle location

        // Update roadrunner's idea of where the robot is after we ran the trajectory

        // Wait for arm to be in position
        sleep(250);

        // Open grip to drop cone
        drive.setGrip(false);

        // Wait for grip to fully open and cone to drop
        sleep(500);



        //to stack is right
        drive.followTrajectorySequence(toStackFromHigh);
        drive.setHeight(725);
        sleep(2000);
        drive.setGrip(true);

        drive.updatePoseEstimate();

        //TODO THIS IS THE END OF THE FIRST CYCLE

        for(int i = 5; i > 1; i--){
            sleep(500);
            if (i>3) {
                scoreSmall(drive, i, toStackFromLow, toLowFromStack);
            } else {
                //scoreMed(drive, i, toMED2, toStack);
            }

        }
        if (zone == parkingZoneFinder.parkingZone.ZONE1) { parkBot(drive, 0, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE2) { parkBot(drive, 1, parkingSpots); }
        else if (zone == parkingZoneFinder.parkingZone.ZONE3) { parkBot(drive, 2, parkingSpots); }
        else { parkBot(drive, 1, parkingSpots); }

    }

    public void scoreSmall(SampleMecanumDrive _drive, int height, TrajectorySequence stackMove, TrajectorySequence scoreMove){

        _drive.setHeight(1800);

        _drive.followTrajectorySequence(scoreMove);

        _drive.setGrip(false);
        sleep(250);

        _drive.followTrajectorySequence(stackMove);
        _drive.setHeight(1350-(height*150));
        sleep(500);
        _drive.setGrip(true);
        sleep(250);

    }

    // mj - middle junction
    /*
    public void scoreMed(SampleMecanumDrive _drive, int h, TrajectorySequence med, TrajectorySequence stack){
        _drive.setHeight(2000);
        _drive.updatePoseEstimate();
        TrajectorySequence toMED2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(medJunction,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        _drive.followTrajectorySequence(toMED2);
        _drive.setGrip(false);
               _drive.updatePoseEstimate();
        TrajectorySequence tostack4 = _drive.trajectorySequenceBuilder(medJunction)
                .lineToLinearHeading(stackPose,
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        _drive.updatePoseEstimate();

        _drive.followTrajectorySequence(tostack4);
        _drive.setHeight(1050-(h*150));
        _drive.setGrip(false);
        sleep(250);

    }

    //high junction
    public void hj (){

    }

    // low junction
    public void lj() {

    }
     */

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
