package org.firstinspires.ftc.teamcode.drive.centerstage2023.CenStageAUTO;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;


@Config
    @Autonomous(name = "PostSetUp",group = "Practice")


//ok now lets code

    public class postSetUp extends LinearOpMode{

//here i am setting up the start and endpose for my robot so that it does that math and makes sure it is accurate when moving

    //middle of the feild looking up
    private final Pose2d startPose = Pose2d(0,0,Math.toRadians(90));

    private final Pose2d endPose = Pose2d(15,15,Math.toRadians(0));


    //this is our travel constraints
        private final double travelSpeed = 45.0, travelAccel = 30.0;

        SampleMecanumDrive drive;

        @Override
        public void runOpMode() throws InterruptedException {

            //ok so this is a trajectory this is how we move the base of the robot
            //line 41 is making the trajectory similar to our pose2d
            TrajectorySequence Move = drive.trajectorySequenceBuilder (startPose)
                    //this .lineToSplineHeading is just a fancy word of move lol
                    .lineToSplineHeading(endPose,
                            //adding out travel constraints
                            SampleMecanumDrive.getVelocityConstraint(travelSpeed,
                                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(travelAccel)
                    )
                    .build();


            //what is important here is that Move rn is nothing it is just pre telling what the robot to do

            //what really makes the robot move is this line
            drive.followTrajectorySequence(Move);
            //anytime you want to move the robot you will need to make a trajecotry and follow what i just did
            // now as time goes on I will show more complicated and more confusing ways to write everything but rn
            //just try your best to understand and get used to his way of typing
            //next pratice I will make yall write this all solo and then we will hopefully run :)



            //now this dot stuff is getting confussing and well it is and im sorry
            //but a function works in a few ways
            // you can call a function by doing Function(object) or you can call a function by doing
            // object.function();
            //in the Move trajacotry the object type is TrajecotrySequence
            // the name is Move
            // then we do = drive.trjectorySequenceBuilder
            //trjectorySequenceBuilder is a FUNCTION and well we need to call this function with an object and the object we need is drive so we go
            //drive.trjectorySequenceBuilder
            //now what is in the () is our startpose so when this code runs this trajecotry is telling the robot what start pose says which is 0,0,heading 90
            // now what is confusing is this next .lineToSplineHeading this is hard to explain but this . connects to the trajecotry object not the drive object.
            // which means that we are saying in the Trajectory object of Move we are adding  LinetoSPineHeading. it has a lot of funcky stuff but dont need to explain that because yet
            // now we end and then we do .build() then an ; which means that we have a trjecotry object called MOVE
            // which has the action of doing a LinetoSplineHeading (this looks like irl the bot moves in an S path)
            // Now how do we run this well we apply this command to Drive aka our robot so going drive.followTrajectorySequence we are telling the code ROBOT you do this Trajecotry that we made


        }
    }

