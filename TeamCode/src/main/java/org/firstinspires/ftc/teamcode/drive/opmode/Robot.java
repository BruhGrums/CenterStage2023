package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Hardware definitions and access for a robot with a four-motor
 * drive train and a gyro sensor.
 */
public class Robot {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // Create motor, servo, and gyro objects
    private final DcMotor leftFront, leftRear, rightFront, rightRear;
    private final DcMotor slideLeft, slideRight, slideTop;
    private final BNO055IMU imu;
    private final Servo leftGripServo, rightGripServo;

    // Create variables for headless operation
    private double headingOffset = 0.0;  // Allows headless mode to correct for rotation
    private Orientation angles;          // Uses builtin libraries to retrieve heading angle
    private Acceleration gravity;        // Builtin function for the control hub to find orientation

    // Create robot class
    public Robot(final HardwareMap _hardwareMap, final Telemetry _telemetry) {
        // Pass variables through to Robot class
        hardwareMap = _hardwareMap;
        telemetry = _telemetry;

        // Ask the driver hub which port each motor is attached to based on robot configuration
        // Configuration is managed at the three dots on the top right of the driver hub
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        // Reverse left side motors
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set up gyro. This is black magic. I have no idea what it does.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        // Do the same thing we did earlier with the drive motors, just for the slide
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        slideTop = hardwareMap.dcMotor.get("slideTop");

        // One of the slide motors MUST be reversed or teh two motors will fight each other
        slideLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the slide motors to brake whenever we don't give any input
        // This helps hold the slide still and reduce the workload on the arm driver
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up the servos how we did the rest of the motors
        leftGripServo = hardwareMap.servo.get("leftGripServo");
        rightGripServo = hardwareMap.servo.get("rightGripServo");
    }

    // This function accepts a motor mode followed by a list of DcMotor objects
    private void setMotorMode(DcMotor.RunMode mode, DcMotor... motors) {
        // Iterate over each DcMotor object and set their motor mode
        for (DcMotor motor : motors) {
            motor.setMode(mode);
        }
    }

    // Invoke setMotorMode() to turn on encoders
    public void runUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, leftFront, leftRear, rightFront, rightRear);
    }

    // Invoke setMotorMode() to turn off encoders
    public void runWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, leftFront, leftRear, rightFront, rightRear);
    }

    // Invoke setMotorMode() to turn on slide encoders
    public void runSlideUsingEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, slideLeft, slideRight);
    }

    // Invoke setMotorMode() to turn on slide encoders
    public void runSlideWithoutEncoders() {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, slideLeft, slideRight);
    }

    // Does the same thing as setMotorMode(), just with a zeroPowerMode
    private void setBrake(DcMotor.ZeroPowerBehavior mode, DcMotor... motors) {
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(mode);
        }
    }

    // Invoke setBrake() to turn on drive motor brakes
    public void runWithBrakes() {
        setBrake(DcMotor.ZeroPowerBehavior.BRAKE, leftFront, leftRear, rightFront, rightRear);
    }

    // Invoke setBrake() to turn off drive motor brakes
    public void runWithoutBrakes() {
        setBrake(DcMotor.ZeroPowerBehavior.FLOAT, leftFront, leftRear, rightFront, rightRear);
    }

    // Return true if the gyro is calibrated
    public boolean isGyroCalibrated() {
        return imu.isGyroCalibrated();
    }

    // Refreshes gyro values when called by an opmode
    // These are computationally expensive tasks so don't call this extra times for fun
    public void loop() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu.getGravity();
    }

    // Return the raw heading
    private double getRawHeading() {
        return angles.firstAngle;
    }

    // Subtract our heading offset from the raw heading to give us relative heading
    // Then modulo by 2pi to make sure that we are between 0 and 2pi
    public double getHeading() {
        return (getRawHeading() - headingOffset) % (2.0 * Math.PI);
    }

    // Change our current heading to degrees and return it
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }

    // Set the heading offset to our current heading
    // This means that any angles after this will be measured relative to this heading
    public void resetHeading() {
        headingOffset = getRawHeading();
    }

    /**
     * Find the maximum absolute value of a set of numbers.
     *
     * @param xs Some number of double arguments
     * @return double maximum absolute value of all arguments
     */
    private static double maxAbs(double... xs) {
        double ret = Double.MIN_VALUE;
        for (double x : xs) {
            if (Math.abs(x) > ret) {
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    /**
     * Set motor powers
     * <p>
     * All powers will be scaled by the greater of 1.0 or the largest absolute
     * value of any motor power.
     *
     * @param _leftFront Left front motor
     * @param _leftRear Left rear motor
     * @param _rightFront Right front motor
     * @param _rightRear Right rear motor
     */
    // Basically, if for whatever reason we request more than 100% speed from a motor, this function
    // automatically reduces the power of all the motors proportionally
    public void setMotors(double _leftFront, double _leftRear, double _rightFront, double _rightRear, double multiplier) {
        final double scale = maxAbs(1.0, _leftFront, _leftRear, _rightFront, _rightRear);
        leftFront.setPower((_leftFront * multiplier) / scale);
        leftRear.setPower((_leftRear * multiplier) / scale);
        rightFront.setPower((_rightFront * multiplier) / scale);
        rightRear.setPower((_rightRear * multiplier) / scale);
    }

    // Provide a convenient way to set all the slide motors
    public void setSlideMotors(double _slideLeft, double _slideRight, double _slideTop) {
        slideLeft.setPower(_slideLeft);
        slideRight.setPower(_slideRight);
        slideTop.setPower(_slideTop);
    }

    // gripPower takes the position of the gripper from 0 to 1 (0 is open, 1 is closed)
    // The stowed variable pulls the grippers further back than normal operation, needs to be
    // calibrated better
    public void setGrip(boolean grip) {
        double leftOpen = 0.0, leftClosed = 105.0;
        double rightOpen = 270.0, rightClosed = 175.0;

        if (grip) {
            leftGripServo.setPosition(leftClosed / 270);
            rightGripServo.setPosition(rightClosed / 270);
        } else if (!grip) {
            leftGripServo.setPosition(leftOpen / 270);
            rightGripServo.setPosition(rightOpen / 270);
        }
    }
}