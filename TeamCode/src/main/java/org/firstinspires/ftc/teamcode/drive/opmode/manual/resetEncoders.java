package org.firstinspires.ftc.teamcode.drive.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.opmode.helpers.Slide;

@Disabled
@TeleOp
public class resetEncoders extends LinearOpMode {
    private Slide slides;

    @Override
    public void runOpMode() {
        slides = new Slide(hardwareMap, telemetry);
        slides.initArm();
    }
}
