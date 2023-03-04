package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous
public class testtrjectories extends LinearOpMode {


    private final Pose2d startPose = new Pose2d(35, -64.25, Math.toRadians(90)); // our Starting pose allows us to know our postions of the robot and know what way it os looking at'/
    private final Pose2d stackPose = new Pose2d(49.5, -12, Math.toRadians(0));
    private final Pose2d lowJunction = new Pose2d(34, -12, Math.toRadians(-53));
    private final Pose2d medJunction = new Pose2d(35,-14, Math.toRadians(-140));
    private final Pose2d highJunction = new Pose2d(37, -12, Math.toRadians(145));
    private final double travelSpeed = 50, travelAccel = 20;
    // the three different parking locations in poses
    private Pose2d[] parkingSpots = {new Pose2d(12, -17, Math.toRadians(90)), new Pose2d(36,
            -20, Math.toRadians(90)), new Pose2d(64, -15, Math.toRadians(90))};
    // camera images sizes 1280 pixels

    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
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
                .addTemporalMarker(0.5, () -> {
                    drive.setHeight(findHeight(5));
                })
                .setTangent(Math.toRadians(-25))
                .splineToSplineHeading(new Pose2d(43, -14, Math.toRadians(0)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .splineToConstantHeading(stackPose.vec(), stackPose.getHeading(),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();

        TrajectorySequence toStackFromLow = drive.trajectorySequenceBuilder(lowJunction)
                .addTemporalMarker(.5, () -> {
                    drive.setHeight(findHeight(3.5));
                })
                .splineToLinearHeading(stackPose, Math.toRadians(-20),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();
        TrajectorySequence toStackFromMed = drive.trajectorySequenceBuilder(medJunction)
                .addTemporalMarker(1, () -> {
                    drive.setHeight(findHeight(2));
                })
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)), Math.toRadians(0),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                //no bitches?
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .splineToConstantHeading((stackPose.minus(new Pose2d(1, 0, Math.toRadians(0)))).vec(), stackPose.getHeading(),
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

        TrajectorySequence toMedFromStack = drive.trajectorySequenceBuilder(stackPose)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(44, -12, Math.toRadians(0)), Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel))
                .splineToSplineHeading(medJunction, Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                )
                .build();


//TODO this is the Preloaded Cone Stuff
        drive.setSlideVelocity(3000, drive.slideRight, drive.slideLeft, drive.slideTop);

        // Close the grip and move the slide up a small amount
        drive.setGrip(true);
        sleep(250);
        drive.setHeight(200);
        //no bitches?
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

        drive.updatePoseEstimate();

        //TODO THIS IS THE END OF THE FIRST CYCLE


        drive.followTrajectorySequence(toStackFromHigh);
        sleep(2000);


        drive.followTrajectorySequence(toMedFromStack);
        sleep(2000);
        drive.followTrajectorySequence(toStackFromMed);


    }


    private int findHeight(double height) { return (int)(780 - ((5 - height) * 155)); }


}
