package org.firstinspires.ftc.teamcode.drive.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.Controller;

@Disabled
@TeleOp(name = "servo tester", group = "testing")
public class servoTester extends LinearOpMode {
    private Controller controller1;

    public double leftPos = 0.0, rightPos = 0.0;

    private Servo leftServo, rightServo;

    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new Controller(gamepad1);

        leftServo = hardwareMap.servo.get("leftGripServo");
        rightServo = hardwareMap.servo.get("rightGripServo");

        waitForStart();

        while (opModeIsActive()) {
            controller1.update();

            if (controller1.dpadUpOnce()) { leftPos += 10; }
            if (controller1.dpadDownOnce()) { leftPos -= 10; }
            if (controller1.dpadLeftOnce()) { leftPos += 1; }
            if (controller1.dpadRightOnce()) { leftPos -= 1; }

            if (controller1.triangleOnce()) { rightPos += 10; }
            if (controller1.crossOnce()) { rightPos -= 10; }
            if (controller1.squareOnce()) { rightPos -= 1; }
            if (controller1.circleOnce()) { rightPos += 1; }

            leftServo.setPosition(leftPos / 270);
            rightServo.setPosition(rightPos / 270);

            telemetry.addData("left pos", leftPos);
            telemetry.addData("right pos", rightPos);
            telemetry.update();
        }
    }
}
