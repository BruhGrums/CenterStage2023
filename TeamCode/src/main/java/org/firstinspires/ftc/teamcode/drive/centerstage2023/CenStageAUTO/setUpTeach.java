package org.firstinspires.ftc.teamcode.drive.centerstage2023.CenStageAUTO;
//This is how we comment is first with  "//" and we can type what ever in this line

/* if you want to comment on a lot of lines
then  "/* " and end with */


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// now @Config and @Autonomous is how we import functions
@Config
@Autonomous(name = "SetupCode",group = "Practice")
// what is this (name=) stuff?? well this is just sentax that allows us on the
//controler to distainguish what code we will run

//now what is this public class nonsense well this just means that this class is able to be edited
// What is extends LinearOpMode well think of where we are coding as a Chapter in the book of linearopmode
// this just gives us the ablity to run our code lol
//now I want yall to watch this video about java classes and how they work I am sorry its a video but I amm shit at explaining
//  it is hella boring but i will ask if you watched it https://www.youtube.com/watch?v=4KZ0pChE2gs

public class setUpTeach extends LinearOpMode {

    // now this is where we create our veriables
    // if we are going to use a number or a string or another object that is going to be refereced
    // A LOT then it is important to write them here

    // when working with veriables and creating them there is a lot of ways to go about

    // the way we do it is we first say what the veriavle type is
    // then we spave and name then we do = to set what the named veriable is.
    // ofc you can put a ; to let the veriable have no inherate value.


    private final double travelSpeed = 45.0, travelAccel = 30.0;

    //what is private final?? well in classes aka where we are typing Private means its exlusive to this section of code
    //what is double - double is the Veriable type . A Double is a number with a decimal.
    //what is with the comma well the comma is a way of shortinging what we have to write and making 2 veriables in one line




    //when we are nameming something we use a form of typing called Cammal Case quick expo
    // no spaces and you capitalize the firstLetter of eachWord but not for the firstWord
    //here is an example "theDogRuns"  or "graemeHatesAllCraftJuniorsHeDoesntReally"



// now this Pose2d is different in that instead of a veriable this is actually considered an object
    // because Pose2d is a class itself with its own fucntions and veriables and a pose2d is what the class makes
    // so when creating a pose2d you actually (this is the same with all objects)
// first you type what the object is in this case it is a Pose2d
    // then ofc space name it
    //next you use a builder which is normally just the object type again but you add ()
    // in this case we do = Pose2d ()
    //now what we put into these () are paremeters
    //this case we are saying the pose2d that is names startPose is at (x36,y34, heading of 90)

    private final Pose2d startPose = Pose2d (36,34,Math.toRadians(90));

    SampleMecanumDrive drive;
    //what is this well this is an object that is horriably named drive is acutally our robot!!
    //so when making our robot move up well we have to go drve.moveup its and object that we tag the commands to


    //why do we need @Overide because in linearOpMode it has the same function however when we press start we want to call/play the code in this class not in linearOpMode

    @Override
    public void runOpMode() throws InterruptedException {

        //this is where we code out sqeuences.

        //look at postSetUp


    }
}

