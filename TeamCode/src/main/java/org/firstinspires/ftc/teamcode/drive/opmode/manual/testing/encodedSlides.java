package org.firstinspires.ftc.teamcode.drive.opmode.manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@TeleOp(group = "testing")
public class encodedSlides extends LinearOpMode {
    DcMotorEx slideLeft, slideRight, slideTop;

    @Override
    public void runOpMode() {
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slideTop = hardwareMap.get(DcMotorEx.class, "slideTop");

        // Reset the encoder during initialization
        slideLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideTop.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Set the motor's target position to 300 ticks
        slideLeft.setTargetPosition(4200);
        slideRight.setTargetPosition(-400);
        slideTop.setTargetPosition(-2000);

        // Switch to RUN_TO_POSITION mode
        slideLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slideTop.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Start the motor moving by setting the max velocity to 200 ticks per second
        //slideLeft.setVelocity(800);
        //slideRight.setVelocity(800);
        //slideTop.setVelocity(500);

        // While the Op Mode is running, show the motor's status via telemetry
        while (opModeIsActive()) {
            telemetry.addData("Left Slide Motor", slideLeft.getCurrentPosition());
            telemetry.addData("Right Slide Motor", slideRight.getCurrentPosition());
            telemetry.addData("Top Slide Motor", slideTop.getCurrentPosition());
            telemetry.update();
        }
    }
}