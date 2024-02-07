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

import static org.firstinspires.ftc.teamcode.modules.LiftConstants.FIRST_LINE_POS;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.MIN_POS;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.MAX_POS;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.SPEED;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.THIRD_LINE_POS;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.kP;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.kI;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.kD;
import static org.firstinspires.ftc.teamcode.modules.LiftConstants.kG;



@Config
public class Lift {
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotorEx motorRight;
    private final DcMotorEx motorLeft;
    public final ElapsedTime timer;
    private final FtcDashboard dashboard;
    private final double[] positions = {MIN_POS, (MAX_POS - MIN_POS) / 4, (MAX_POS - MIN_POS) * 3 / 4, MAX_POS};
    private double reference = 0;
    private double lastErrorLeft = 0;
    private double lastErrorRight = 0;
    private double integralSumLeft = 0;
    private double integralSumRight = 0;
    public static double pos = 0;

    public Lift(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;
        timer = new ElapsedTime();
        this.dashboard = dashboard;

        motorRight = hardwareMap.get(DcMotorEx.class, "right_lift_motor");
        motorLeft = hardwareMap.get(DcMotorEx.class, "left_lift_motor");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Lift: Initialized");
    }

    public void opControl() {
        reference += (-gamepad.left_stick_y) * SPEED * timer.seconds();
        reference = pos;
        PIDControl();
    }

    public void resetEncoders() {
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void PIDControl() {
        /*
        if (reference > MAX_POS) {
            reference = MAX_POS;
        }
        if (reference < MIN_POS) {
            reference = MIN_POS;
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
        lastErrorRight = errorRight;

         */

        if (reference > MAX_POS) {
            reference = MAX_POS;
        }
        if (reference < MIN_POS) {
            reference = MIN_POS;
        }

        double encoderPosLeft = motorLeft.getCurrentPosition();

        double errorLeft = reference - encoderPosLeft;

        double derivativeLeft = (errorLeft - lastErrorLeft) / timer.seconds();

        integralSumLeft += errorLeft * timer.seconds();

        double outLeft = (kP * errorLeft) + (kI * integralSumLeft) + (kD * derivativeLeft) + kG;

        motorLeft.setPower(outLeft);
        motorRight.setPower(outLeft);

        lastErrorLeft = errorLeft;

        // 100% рабочий энкодер - левый


        TelemetryPacket packet = new TelemetryPacket();
        packet.put("reference_lift", reference);
        packet.put("encoder_left", encoderPosLeft);
        // packet.put("encoder_right", encoderPosRight);
        packet.put("errorLeft", errorLeft);
        packet.put("power_out_left", outLeft);
        // packet.put("power_out_right", outRight);
        packet.put("seconds_per_call", timer.seconds());
        packet.put("IntLeft", integralSumLeft);
        packet.put("IntRight", integralSumRight);
        dashboard.sendTelemetryPacket(packet);
        telemetry.addLine("---------------");
        telemetry.addLine("Lift:");
        telemetry.addData("reference", reference);
        telemetry.addData("left_encoder", motorLeft.getCurrentPosition());
        telemetry.addData("right_encoder", motorRight.getCurrentPosition());
        telemetry.addData("gp2_left_stick_y", -gamepad.left_stick_y);
        timer.reset();
    }

    public void opControlPos() {
        if (gamepad.dpad_right) {
            reference = THIRD_LINE_POS;
        } else if (gamepad.dpad_up) {
            //reference = SECOND_LINE_POS;
        } else if (gamepad.dpad_left) {
            reference = FIRST_LINE_POS;
        } else if (gamepad.dpad_down) {
            reference = MIN_POS;
        }
        reference += (-gamepad.left_stick_y) * SPEED * timer.seconds();
        PIDControl();
    }

    public void opControlPosTB() {
        if (gamepad.dpad_up) {
            for (int i = 0; i < positions.length - 1; i += 1) {
                if (motorRight.getCurrentPosition() < positions[i + 1] && motorRight.getCurrentPosition() > positions[i]) {
                    if (reference == positions[i + 1] &&
                            Math.abs(motorRight.getCurrentPosition() - positions[i + 1]) <= 10 &&
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
                            Math.abs(motorRight.getCurrentPosition() - positions[i]) <= 10 &&
                            0 <= i - 1) {
                        reference = positions[i - 1];
                    } else {
                        reference = positions[i];
                    }
                }
            }
        }
        PIDControl();
    }

    public void moveToPos(double pos) {
        reference = pos;
        int counter = 150;

        while (counter > 0) {
            PIDControl();
            counter -= 1;
        }
    }

    public void setReference(double reference) {
        this.reference = reference;
    }

    public double getReference() {
        return reference;
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