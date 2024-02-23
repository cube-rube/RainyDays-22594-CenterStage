package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.autonomous.PoseCache;
import org.firstinspires.ftc.teamcode.teleop.MainTeleOp;

@Config
public class OperatorDrive {
    private final LinearOpMode linearOpMode;
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

    static final double DRIVE_SPEED_TPS = 2796 * 0.77; // 77% of max tps?????????

    double kP = 0;

    double kI = 0;

    double kD = 0;

    double intengalSumRf = 0;
    double intengalSumRb = 0;
    double intengalSumLf = 0;
    double intengalSumLb = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastErRf = 0;
    private double lastErRb = 0;
    private double lastErLf = 0;
    private double lastErLb = 0;


    public enum DriveState {
        BACKDROP,
        FIELD
    }
    private enum ButtonState {
        PRESSED,
        HELD,
        RELEASED
    }
    private ButtonState buttonState = ButtonState.RELEASED;

    public DriveState driveState = DriveState.FIELD;
    double globalAngle;
    Orientation lastAngles = new Orientation();


    public OperatorDrive(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        this.linearOpMode = linearOpMode;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
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
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
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

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("BasicDrive: Initialized");
    }

    public void driveRobotCentricEncoder() {
        double max;

        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x * 1.1;
        double yaw     = -gamepad.right_trigger + gamepad.left_trigger;
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
        double yaw     =  -gamepad.right_trigger + gamepad.left_trigger;

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

        double SPEED_MULTIPLIER = 1.0;
        if (gamepad.b) {
            SPEED_MULTIPLIER = 0.5;
        }

        leftFrontDrive.setPower(leftFrontPower * SPEED_MULTIPLIER);
        rightFrontDrive.setPower(rightFrontPower * SPEED_MULTIPLIER);
        leftBackDrive.setPower(leftBackPower * SPEED_MULTIPLIER);
        rightBackDrive.setPower(rightBackPower * SPEED_MULTIPLIER);
    }

    public void driveFieldCentricEncoder() {
        double max;

        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x;
        double yaw     =  -gamepad.right_trigger + gamepad.left_trigger;

        /*
        axial = driveFunc(axial);
        lateral = driveFunc(lateral);
        yaw = driveFunc(yaw);
        */

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

        double leftFrontSpeed = leftFrontDrive.getVelocity();
        double rightFrontSpeed = rightFrontDrive.getVelocity();
        double leftBackSpeed = leftBackDrive.getVelocity();
        double rightBackSpeed = rightBackDrive.getVelocity();

        double leftFrontError = (leftFrontPower * DRIVE_SPEED_TPS) * 0.75 - leftFrontSpeed;
        double rightFrontError = (rightFrontPower * DRIVE_SPEED_TPS) * 0.75 - rightFrontSpeed;
        double leftBackError = (leftBackPower * DRIVE_SPEED_TPS) * 0.75 - leftBackSpeed;
        double rightBackError = (rightBackPower * DRIVE_SPEED_TPS) * 0.75 - rightBackSpeed;


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
        timer.reset();


        leftFrontDrive.setPower((leftFrontPower * 0.75 + kP * leftFrontError + kI * intengalSumLf + derivativeLf * kD));
        rightFrontDrive.setPower((rightFrontPower * 0.75 + kP * rightFrontError + kI * intengalSumRf + derivativeRf * kD));
        leftBackDrive.setPower((leftBackPower * 0.75 + kP * leftBackError + kI * intengalSumLb + derivativeLb * kD));
        rightBackDrive.setPower((rightBackPower * 0.75 + kP * rightBackError + kI * intengalSumRb + derivativeRb * kD));
    }

    public void driveFieldCentric() {
        double max;

        double axial   = -gamepad.left_stick_y;
        double lateral =  gamepad.left_stick_x;
        double yaw     =  gamepad.right_trigger - gamepad.left_trigger;

        if (gamepad.start) {
            imu.resetYaw();
            PoseCache.pose = new Pose2d(PoseCache.pose.vec(), Math.toRadians(0));
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

        double SPEED_MULTIPLIER = 1.0;
        if (gamepad.b) {
            SPEED_MULTIPLIER = 0.5;
        }

        leftFrontDrive.setPower(leftFrontPower * SPEED_MULTIPLIER);
        rightFrontDrive.setPower(rightFrontPower * SPEED_MULTIPLIER);
        leftBackDrive.setPower(leftBackPower * SPEED_MULTIPLIER);
        rightBackDrive.setPower(rightBackPower * SPEED_MULTIPLIER);

        telemetry.addData("heading", botHeading);
        telemetry.addData("Auto_heading", PoseCache.pose.getHeading());
    }

    private double lastError = 0;
    private double integralSum = 0;
    public static double INTEGRAL_SUM_MAX = 0.14;
    public static double MULTIPLIER = 0.5;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(0.014, 0, 0.7, 0); // 2.2 0.2 0.014
    double reference = 0;

    public void backdropDrive() {
        double angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        if (reference == 0) {
            reference = 90;
            if (angle < 0) {
                reference = -90;
            }
        }


        double error = angle - reference;
        double derivative = (error - lastError) * timer.seconds();
        if (Math.abs(integralSum * coefficients.i) >= INTEGRAL_SUM_MAX && Math.signum(integralSum) == Math.signum(error)) {
            integralSum = INTEGRAL_SUM_MAX / coefficients.i * Math.signum(integralSum);
        } else {
            integralSum += error * timer.seconds();
        }


        double power = (error * coefficients.p) + (integralSum * coefficients.i) + (derivative * coefficients.d);
        if (Math.abs(error) > 0.2) {
            power += coefficients.f * Math.signum(error);
        }

        double axial   = -gamepad.left_stick_y * MULTIPLIER;
        double lateral =  gamepad.left_stick_x * MULTIPLIER;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotLateral = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
        double rotAxial   = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);


        rotLateral = rotLateral * 1.1;

        double leftFrontPower  = rotAxial + power + rotLateral;
        double rightFrontPower = rotAxial - power - rotLateral;
        double leftBackPower   = rotAxial + power - rotLateral;
        double rightBackPower  = rotAxial - power + rotLateral;

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

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("reference_angle", reference);
        packet.put("angle", angle);
        packet.put("integral_sum_angle", integralSum);
        packet.put("integral_power_angle", integralSum * coefficients.i);
        dashboard.sendTelemetryPacket(packet);

        lastError = error;
        timer.reset();
    }

    public void drive() {
        switch (driveState) {
            case BACKDROP:
                backdropDrive();
                break;
            case FIELD:
                driveFieldCentric();
                reference = 0;
                lastError = 0;
                integralSum = 0;
                break;

        }
    }

    public void sendAngle() {
        telemetry.addData("imu_yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("imu_orientation", getAngle());
    }


     // Resets the cumulative angle tracking to zero
    private void resetAngle() {
        lastAngles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }


    /**     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.     */
    private double getAngle()
    {        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;

        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    /**     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.     */
    private double checkDirection()    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;
        angle = getAngle();
        if (angle == 0)
            correction = 0;             // no adjustment.        else
        correction = -angle;        // reverse sign of angle for correction.
        correction = correction * gain;
        return correction;
    }


    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate( double power)
    {
        int degrees = (int)getAngle();

        while (degrees < 90) {

            leftFrontDrive.setPower(power);
            leftBackDrive.setPower(power);
            rightFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
        }
        while (degrees > 90)
        {   // turn left.
            leftFrontDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            rightFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
        }

        // rotate until turn is completed.
        // turn the motors off.
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        // reset angle tracking on new heading.
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

    public double driveFunc(double x) {
        double a = 0.17082039325;
        // double a = (-5 + 3 * Math.sqrt(5)) / 10;
        if (x > 0) {
            return -(0.2 / (x - a - 1)) - a;
        } else if (x < 0) {
            return 0.2 / (-x - a - 1) + a;
        } else {
            return 0;
        }
    }

    public double driveSimpleFunc(double x) {
        return x * x * x;
    }

    public void telemetry() {
        telemetry.addLine("---------------");
        telemetry.addLine("BasicDrive:");
        telemetry.addData("left_front_encoder", leftFrontDrive.getCurrentPosition());
        telemetry.addData("left_back_encoder: ", leftBackDrive.getCurrentPosition());
        telemetry.addData("right_front_encoder: ", rightFrontDrive.getCurrentPosition());
        telemetry.addData("right_back_encoder: ", rightBackDrive.getCurrentPosition());
        telemetry.addData("gp1_left_stick_y", -gamepad.left_stick_y);
        telemetry.addData("gp1_left_stick_x", gamepad.left_stick_x);
        telemetry.addData("gp1_right_trigger", gamepad.right_trigger);
        telemetry.addData("gp1_left_trigger", gamepad.left_trigger);
        telemetry.addData("gp1_options", gamepad.options);
        telemetry.addData("gp1_start", gamepad.start);
        telemetry.addData("gp1_back", gamepad.back);
    }

    public void runtimeReset() {
        runtime.reset();
    }
}
