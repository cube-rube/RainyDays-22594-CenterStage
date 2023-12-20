package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class BasicDrive {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotorEx leftFrontDrive;
    private final DcMotorEx leftBackDrive;
    private final DcMotorEx rightFrontDrive;
    private final DcMotorEx rightBackDrive;
    private final IMU imu;
    private final ElapsedTime runtime;
    private final FtcDashboard dashboard;

    public static final double     COUNTS_PER_MOTOR_REV    = 537.7;
    public static final double     DRIVE_GEAR_REDUCTION    = 0.5;     // No External Gearing.
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.7;
    public static final double     TURN_SPEED              = 0.5;

    static final double DRIVE_SPEED_TPS = 2153; // 77% of max tps?????????
    public static double kP = 0.00055;

    /*
    public static double kI = 0;

    public static double kD = 0;

    double intengalSumRf = 0;
    double intengalSumRb = 0;
    double intengalSumLf = 0;
    double intengalSumLb = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastErRf = 0;
    private double lastErRb = 0;
    private double lastErLf = 0;
    private double lastErLb = 0;
    */

    private enum DriveState {
        ROBOT,
        FIELD
    }
    private enum ButtonState {
        PRESSED,
        HELD,
        RELEASED
    }
    private ButtonState buttonState = ButtonState.RELEASED;

    private DriveState driveState = DriveState.ROBOT;

    public BasicDrive(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad1;
        runtime = new ElapsedTime();
        this.dashboard = dashboard;

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "motor_lf");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "motor_lb");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "motor_rf");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "motor_rb");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("BasicDrive:", "Initialized");
    }

    public void tele() {
        switch (buttonState) {
            case PRESSED:
                switch (driveState) {
                    case FIELD:
                        driveState = DriveState.ROBOT;
                        break;
                    case ROBOT:
                        driveState = DriveState.FIELD;
                }
                if (gamepad.right_bumper) {
                    buttonState = ButtonState.HELD;
                } else {
                    buttonState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.right_bumper) {
                    buttonState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.right_bumper) {
                    buttonState = ButtonState.PRESSED;
                }
                break;
        }
        switch (driveState) {
            case ROBOT:
                //driveRobotCentric();
                driveRobotCentricEncoder();
            case FIELD:
                driveFieldCentric();
        }
        telemetry.addData("Drive_State: ", driveState);
    }

    public void driveRobotCentricEncoder() {
        double max;

        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x * 1.1;
        double yaw     =  gamepad.right_trigger - gamepad.left_trigger;
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setVelocity(leftFrontPower * DRIVE_SPEED_TPS);
        rightFrontDrive.setVelocity(rightFrontPower * DRIVE_SPEED_TPS);
        leftBackDrive.setVelocity(leftBackPower * DRIVE_SPEED_TPS);
        rightBackDrive.setVelocity(rightBackPower * DRIVE_SPEED_TPS);
    }

    public void driveRobotCentric() {
        double max;

        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x * 1.1;
        double yaw     =  gamepad.right_trigger - gamepad.left_trigger;

        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void driveFieldCentricEncoder() {
        double max;

        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x;
        double yaw     =  gamepad.right_trigger - gamepad.left_trigger;

        if (gamepad.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotLateral = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotAxial   = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);


        rotLateral = rotLateral * 1.1;

        double leftFrontPower  = rotAxial + rotLateral + yaw;
        double rightFrontPower = rotAxial - rotLateral - yaw;
        double leftBackPower   = rotAxial - rotLateral + yaw;
        double rightBackPower  = rotAxial + rotLateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));


        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        /*
        leftFrontPower  = driveSimpleFunc(leftFrontPower);
        rightFrontPower = driveSimpleFunc(leftFrontPower);
        leftBackPower   = driveSimpleFunc(leftFrontPower);
        rightBackPower  = driveSimpleFunc(leftFrontPower);
        */

        /*
        leftFrontPower  = driveFunc(leftFrontPower);
        rightFrontPower = driveFunc(leftFrontPower);
        leftBackPower   = driveFunc(leftFrontPower);
        rightBackPower  = driveFunc(leftFrontPower);
        */

        double leftFrontSpeed = leftFrontDrive.getVelocity();
        double rightFrontSpeed = rightFrontDrive.getVelocity();
        double leftBackSpeed = leftBackDrive.getVelocity();
        double rightBackSpeed = rightBackDrive.getVelocity();

        double leftFrontError = (leftFrontPower * DRIVE_SPEED_TPS) * 0.75 - leftFrontSpeed;
        double rightFrontError = (rightFrontPower * DRIVE_SPEED_TPS) * 0.75 - rightFrontSpeed;
        double leftBackError = (leftBackPower * DRIVE_SPEED_TPS) * 0.75 - leftBackSpeed;
        double rightBackError = (rightBackPower * DRIVE_SPEED_TPS) * 0.75 - rightBackSpeed;

        /*
        intengalSumLf += leftFrontError * timer.seconds();;
        double derivativeLf = (leftFrontError - lastErLf) / timer.seconds();
        lastErLf = leftFrontError;

        intengalSumLb += leftBackError * timer.seconds();;
        double derivativeLb = (leftBackError - lastErLb) / timer.seconds();
        lastErLb = leftBackError;

        intengalSumRf += rightFrontError * timer.seconds();;
        double derivativeRf = (rightFrontError - lastErRf) / timer.seconds();
        lastErRf = rightFrontError;

        intengalSumRb += rightBackError * timer.seconds();;
        double derivativeRb = (rightBackError - lastErRb) / timer.seconds();
        lastErRb = rightBackError;
        */

        /*
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("reference_lf", leftFrontPower * DRIVE_SPEED_TPS * 0.75);
        packet.put("encoder_lf", leftFrontSpeed);
        packet.put("reference_rf", rightFrontPower * DRIVE_SPEED_TPS * 0.75);
        packet.put("encoder_rf", rightFrontSpeed);
        packet.put("reference_lb", leftBackPower * DRIVE_SPEED_TPS * 0.75);
        packet.put("encoder_lb", leftBackSpeed);
        packet.put("reference_rb", rightBackPower * DRIVE_SPEED_TPS * 0.75);
        packet.put("encoder_rb", rightBackSpeed);
        dashboard.sendTelemetryPacket(packet);
        */

        /*
        leftFrontDrive.setPower((leftFrontPower * 0.75 + kP * leftFrontError + kI * intengalSumLf + derivativeLf * kD));
        rightFrontDrive.setPower((rightFrontPower * 0.75 + kP * rightFrontError + kI * intengalSumRf + derivativeRf * kD));
        leftBackDrive.setPower((leftBackPower * 0.75 + kP * leftBackError + kI * intengalSumLb + derivativeLb * kD));
        rightBackDrive.setPower((rightBackPower * 0.75 + kP * rightBackError + kI * intengalSumRb + derivativeRb * kD));
        */

        leftFrontDrive.setPower((leftFrontPower * 0.75 + kP * leftFrontError));
        rightFrontDrive.setPower((rightFrontPower * 0.75 + kP * rightFrontError));
        leftBackDrive.setPower((leftBackPower * 0.75 + kP * leftBackError));
        rightBackDrive.setPower((rightBackPower * 0.75 + kP * rightBackError));
    }

    public void driveFieldCentric() {
        double max;

        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x;
        double yaw     =  gamepad.right_trigger - gamepad.left_trigger;

        if (gamepad.start) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotLateral = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotAxial   = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);


        rotLateral = rotLateral * 1.1;

        double leftFrontPower  = rotAxial + rotLateral + yaw;
        double rightFrontPower = rotAxial - rotLateral - yaw;
        double leftBackPower   = rotAxial - rotLateral + yaw;
        double rightBackPower  = rotAxial + rotLateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void forwardWithIMU() {
        double axial   = -gamepad.left_stick_y;
        double lateral = 0;

        if (gamepad.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotLateral = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotAxial   = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);

        double leftFrontPower  = rotAxial + rotLateral;
        double rightFrontPower = rotAxial - rotLateral;
        double leftBackPower   = rotAxial - rotLateral;
        double rightBackPower  = rotAxial + rotLateral;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void forward() {
        double axial = -gamepad.left_stick_y;

        leftFrontDrive.setPower(axial);
        rightFrontDrive.setPower(axial);
        leftBackDrive.setPower(axial);
        rightBackDrive.setPower(axial);
    }

    // https://www.desmos.com/calculator/xvnaxukpug?lang=ru
    public double driveFunc(double x) {
        // double a = 0.17082039325;
        double a = (-5 + 3 * Math.sqrt(5)) / 10;
        if (x >= 0) {
            return -(0.2 / (x - a - 1)) - a;
        } else {
            return 0.2 / (-x - a - 1) + a;
        }
    }

    // https://www.desmos.com/calculator/xvnaxukpug?lang=ru
    public double driveSimpleFunc(double x) {
        return x * x * x;
    }

    public void encoderDriveY(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftTarget);
            leftBackDrive.setTargetPosition(newLeftTarget);
            rightFrontDrive.setTargetPosition(newRightTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));


            while (linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            linearOpMode.sleep(250);   // optional pause after each move.
        }
    }

    public void encoderDriveX(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (linearOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            leftFrontDrive.setTargetPosition(newLeftTarget);
            leftBackDrive.setTargetPosition(-newRightTarget);
            rightFrontDrive.setTargetPosition(-newLeftTarget);
            rightBackDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));


            while (linearOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && rightFrontDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        leftFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion//;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            linearOpMode.sleep(250);   // optional pause after each move.
        }
    }

    public void testing() {
        telemetry.addData("LeftFrontDrive Encoder: ", leftFrontDrive.getCurrentPosition());
        telemetry.addData("LeftBackDrive Encoder: ", leftBackDrive.getCurrentPosition());
        telemetry.addData("RightFrontDrive Encoder: ", rightFrontDrive.getCurrentPosition());
        telemetry.addData("RightBackDrive Encoder: ", rightBackDrive.getCurrentPosition());
        telemetry.addData("LeftStickY", -gamepad.left_stick_y);
        telemetry.addData("LeftStickX", gamepad.left_stick_x);
    }

    public void runtimeReset() {
        runtime.reset();
    }
}
