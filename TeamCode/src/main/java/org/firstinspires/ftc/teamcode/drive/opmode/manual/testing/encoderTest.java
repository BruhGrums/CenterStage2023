package org.firstinspires.ftc.teamcode.drive.opmode.manual.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "testing")
public class encoderTest extends OpMode {
    DcMotor parallelEncoder;
    DcMotor perpendicularEncoder;

    public void init() {
        parallelEncoder = hardwareMap.dcMotor.get("parallelEncoder");
        perpendicularEncoder = hardwareMap.dcMotor.get("leftFront");
        telemetry.addLine("ready");
        telemetry.update();
    }

    public void loop() {
        telemetry.addData("parallel", parallelEncoder.getCurrentPosition());
        telemetry.addData("perpendicular", perpendicularEncoder.getCurrentPosition());
        telemetry.update();
    }
}
