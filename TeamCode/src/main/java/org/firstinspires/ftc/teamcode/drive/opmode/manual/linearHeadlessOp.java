package org.firstinspires.ftc.teamcode.drive.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.helpers.Controller;
import org.firstinspires.ftc.teamcode.drive.opmode.helpers.Robot;
import org.firstinspires.ftc.teamcode.drive.opmode.helpers.Slide;

@TeleOp(group = "beta")
public class linearHeadlessOp extends LinearOpMode {
    private Robot robot;
    private Slide slides;
    private Controller controller1, controller2;

    private boolean headlessMode = false;
    private boolean grip = false;

    private final double driveMultiplier = 0.70;
    private final double adjustMultiplier = 0.25;
    private double multiplier = driveMultiplier;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry);
        slides = new Slide(hardwareMap, telemetry);
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);

        robot.runWithoutEncoders();
        robot.runWithBrakes();

        while (opModeInInit()) {
            controller1.update();
            controller2.update();

            if (controller1.crossOnce()) {
                headlessMode = !headlessMode;
            }

            if (controller1.squareOnce()) {
                robot.resetHeading();
            }

            telemetry.addData("Gyro Ready?", robot.isGyroCalibrated() ? "yes" : "no");
            telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
            telemetry.update();
        }

        while (opModeIsActive()) {
            controller1.update();
            controller2.update();

            robot.loop();

            if (controller1.crossOnce()) {
                headlessMode = !headlessMode;
            }

            if (controller1.squareOnce()) {
                robot.resetHeading();
            }

            if (controller1.circleOnce()) {
                multiplier = multiplier == driveMultiplier ? adjustMultiplier : driveMultiplier;
            }

            if (controller2.leftTriggerOnce()) {
                grip = false;
            }

            if (controller2.rightTriggerOnce()) {
                grip = true;
            }

            telemetry.addData("Headless Mode (cross)", headlessMode ? "yes" : "no");
            telemetry.addData("Heading (reset: square)", robot.getHeadingDegrees());
            telemetry.update();

            final double x = -Math.pow(controller1.left_stick_x, 3.0);
            final double y = Math.pow(controller1.left_stick_y, 3.0);
            final double rot = Math.pow(controller1.right_trigger - controller1.left_trigger, 3.0);

            final double direction = Math.atan2(x, y) + (headlessMode ? robot.getHeading() : 0.0);
            final double magnitude = Math.min(1.0, Math.sqrt(x * x + y * y));

            double y_proc = -1 * magnitude * Math.sin(direction + Math.PI / 2.0);
            double x_proc = magnitude * Math.cos(direction + Math.PI / 2.0);

            if (controller2.Triangle()) {
                y_proc = 0.75;
                x_proc = 0.0;
            }
            if (controller2.Square()) {
                y_proc = 0.0;
                x_proc = -0.75;
            }
            if (controller2.Cross()) {
                y_proc = -0.75;
                x_proc = 0.0;
            }
            if (controller2.Circle()) {
                y_proc = 0.0;
                x_proc = 0.75;
            }

            final double leftFront = y_proc + x_proc + rot;
            final double leftRear = y_proc - x_proc - rot;
            final double rightFront = y_proc - x_proc + rot;
            final double rightRear = y_proc + x_proc - rot;

            robot.setMotors(leftFront, rightFront, leftRear, rightRear, multiplier);

            final double slideLeft = Math.pow(controller2.left_stick_y, 3.0);
            final double slideRight = Math.pow(controller2.left_stick_y, 3.0);
            final double slideTop = Math.pow(controller2.right_stick_y, 3.0);
            final boolean gripPower = grip;

            // Apply power to slide motors and gripper
            slides.manualHeightControl(Math.pow(controller2.left_stick_y, 3.0));
            slides.manualExtensionControl(Math.pow(controller2.right_stick_x, 3.0));
            slides.setGrip(grip);

            /* TODO: Currently destroys the robot, fix it
            if (controller2.dpadUpOnce()) {
                slides.goToJunction(Slide.heights.HIGH);
            } else if (controller2.dpadRightOnce()) {
                slides.goToJunction(Slide.heights.MID);
            } else if (controller2.dpadDownOnce()) {
                slides.goToJunction(Slide.heights.LOW);
            }
            */
        }
    }
}
