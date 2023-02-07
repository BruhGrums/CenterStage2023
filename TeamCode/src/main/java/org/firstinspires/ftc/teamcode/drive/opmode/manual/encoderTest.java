package org.firstinspires.ftc.teamcode.drive.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "testing")
public class encoderTest extends OpMode {
    DcMotor rotaryEncoder;

    public void init() {
        rotaryEncoder = hardwareMap.dcMotor.get("rotaryEncoder");
        telemetry.addLine("ready");
        telemetry.update();
    }

    public void loop() {
        telemetry.addData("encoder pos", rotaryEncoder.getCurrentPosition());
        telemetry.update();
    }
}
