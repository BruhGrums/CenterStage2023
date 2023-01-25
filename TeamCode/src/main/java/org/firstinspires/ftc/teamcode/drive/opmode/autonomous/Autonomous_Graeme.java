package org.firstinspires.ftc.teamcode.drive.opmode.autonomous;/*
package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "drive1")
public class Autonomous_Graeme extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // this is our robot drive represents all the motors servos -caponets
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory firstTrajectory = new TrajectoryBuilder(new Pose2d())
        .strafeRight(10);

        // this is just looking for start
        waitForStart();
        // this is just an infinite look looking for stop
        while ( opModeIsActive() && !isStopRequested()){
            drive.followTrajectory(firstTrajectory);
        };



        // this is the first path i am creating a trajectory Ie pathway)
        // pose2d is just saying robot brain thinks
        // it is at 0,0 at heading on a cordinate plane
        // for future referance look at
        // https://learnroadrunner.com/trajectories.html#building-a-trajectory











    }
}
*/

// TODO: reenable this, it wouldn't built without commenting it out though