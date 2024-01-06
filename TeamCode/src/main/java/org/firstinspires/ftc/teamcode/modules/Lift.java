package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift {
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotorEx motorRight;
    private final DcMotorEx motorLeft;
    public final ElapsedTime timer;
    private final FtcDashboard dashboard;

    public static double kP = 0.01;
    public static double kI = 0.005;
    public static double kD = 0.0000055;
    public static double kG = 0.1;

    public static double maxPos = 1170, minPos = 40;
    public static double speed = 16000;
    private final double[] positions = {minPos, (maxPos - minPos) / 4, (maxPos - minPos) * 3 / 4, maxPos};
    private double reference = 0;
    private double lastErrorLeft = 0;
    private double lastErrorRight = 0;
    private double integralSumLeft = 0;
    private double integralSumRight = 0;
    public static double pos = 0;

    public Lift(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;
        timer = new ElapsedTime();
        this.dashboard = dashboard;

        motorRight = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        motorLeft = hardwareMap.get(DcMotorEx.class, "left_lift_motor");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Lift: ", "Initialized");
    }

    public void opControl() {
        reference += (-gamepad.left_stick_y) * speed * timer.seconds();
        reference = pos;
        if (reference > maxPos) {
            reference = maxPos;
        }
        if (reference < minPos) {
            reference = minPos;
        }

        double encoderPosLeft = motorLeft.getCurrentPosition();
        double encoderPosRight = motorRight.getCurrentPosition();

        double errorLeft = reference - encoderPosLeft;
        double errorRight = reference - encoderPosRight;

        double derivativeLeft = (errorLeft - lastErrorLeft) / timer.seconds();
        double derivativeRight = (errorRight - lastErrorRight) / timer.seconds();

        integralSumLeft += errorLeft * timer.seconds();
        integralSumRight += errorRight * timer.seconds();

        double outLeft = (kP * errorLeft) + (kI * integralSumLeft) + (kD * derivativeLeft) + kG;
        double outRight = (kP * errorRight) + (kI * integralSumRight) + (kD * derivativeRight) + kG;


        motorLeft.setPower(outLeft);
        motorRight.setPower(outRight);

        lastErrorLeft = errorLeft;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("reference", reference);
        packet.put("encoder_left", encoderPosLeft);
        packet.put("encoder_right", encoderPosRight);
        packet.put("errorLeft", errorLeft);
        packet.put("power_out_left", outLeft);
        packet.put("power_out_right", outRight);
        packet.put("seconds_per_call", timer.seconds());
        packet.put("IntLeft", integralSumLeft);
        packet.put("IntRight", integralSumRight);
        dashboard.sendTelemetryPacket(packet);
        telemetry.addData("LiftR_encoder: ", motorRight.getCurrentPosition());
        telemetry.addData("LiftL_encoder: ", motorLeft.getCurrentPosition());
        telemetry.addData("controller", -gamepad.left_stick_y);
        timer.reset();
    }

    public void opControlPos() {
        if (gamepad.dpad_up) {
            for (int i = 0; i < positions.length - 1; i += 1) {
                if (motorRight.getCurrentPosition() < positions[i + 1] && motorRight.getCurrentPosition() > positions[i]) {
                    if (reference == positions[i + 1] &&
                            Math.abs(motorRight.getCurrentPosition() - positions[i + 1]) <= 40 &&
                            positions.length > i + 2) {
                        reference = positions[i + 2];
                    } else {
                        reference = positions[i + 1];
                    }
                }
            }
        }
        else if (gamepad.dpad_down) {
            for (int i = 0; i < positions.length - 1; i += 1) {
                if (motorRight.getCurrentPosition() < positions[i + 1] && motorRight.getCurrentPosition() > positions[i]) {
                    if (reference == positions[i] &&
                            Math.abs(motorRight.getCurrentPosition() - positions[i]) <= 40 &&
                            0 <= i - 1) {
                        reference = positions[i - 1];
                    } else {
                        reference = positions[i];
                    }
                }
            }
        }
        telemetry.addData("reference", reference);
        telemetry.addData("encoderPos", motorRight.getCurrentPosition());
        opControl();
    }


    public void moveToPos(double pos) {
        double encoderPos = motorRight.getCurrentPosition();
        double error;
        double lastError = 0;
        double derivative;
        double integralSum = 0;
        double out;

        timer.reset();

        while (Math.abs(encoderPos - pos) > 40) {
            encoderPos = motorRight.getCurrentPosition();
            error = pos - encoderPos;
            derivative = (error - lastError) / timer.seconds();
            integralSum += error * timer.seconds();
            out = (kP * error) + (kI * integralSum) + (kD * derivative) + kG;

            motorRight.setPower(out);
            motorLeft.setPower(out);

            lastError = error;

            timer.reset();
        }
    }

    public void runtimeReset() {
        timer.reset();
    }

    public void testing() {
        int posRight = motorRight.getCurrentPosition();
        int posLeft = motorLeft.getCurrentPosition();

        telemetry.addData("LiftR_encoder: ", posRight);
        telemetry.addData("LiftL_encoder: ", posLeft);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("LiftR_encoder", posRight);
        packet.put("LiftL_encoder", posLeft);
        packet.put("Diff", Math.abs(posRight - posLeft));
        dashboard.sendTelemetryPacket(packet);
    }
}