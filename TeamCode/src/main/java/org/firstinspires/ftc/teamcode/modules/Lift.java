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
    private final DcMotorEx motor;
    public final ElapsedTime runtime;
    private final FtcDashboard dashboard;

    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0.00055;
    public static double kG = 0.31;

    public static double maxPos = 490, minPos = 0;
    public static double speed = 500;
    private final double[] positions = {minPos, (maxPos - minPos) / 4, (maxPos - minPos) * 3 / 4, maxPos};
    private double currentPos = 0;
    private double lastError = 0;
    private double integralSum = 0;

    public Lift(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;
        runtime = new ElapsedTime();
        this.dashboard = dashboard;

        motor = hardwareMap.get(DcMotorEx.class, "lift_motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Lift: ", "Initialized");
    }

    public void telePID() {
        currentPos += (-gamepad.left_stick_y) * speed * runtime.seconds();
        if (currentPos > maxPos) {
            currentPos = maxPos;
        }
        if (currentPos < minPos) {
            currentPos = minPos;
        }

        double encoderPos = motor.getCurrentPosition();
        double error = currentPos - encoderPos;
        double derivative = (error - lastError) / runtime.seconds();
        integralSum += error * runtime.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative) + kG;

        motor.setPower(out);

        lastError = error;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("reference", currentPos);
        packet.put("encoder", encoderPos);
        packet.put("error", error);
        packet.put("power_out", out);
        packet.put("seconds_per_call", runtime.seconds());
        dashboard.sendTelemetryPacket(packet);
        runtime.reset();
    }

    public void auto() {
        currentPos = minPos;

        double encoderPos = motor.getCurrentPosition();
        double error = currentPos - encoderPos;
        double derivative = (error - lastError) / runtime.seconds();
        integralSum += error * runtime.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative) + kG;

        motor.setPower(out);

        lastError = error;

        runtime.reset();
    }

    public void teleWithPos() {
        if (gamepad.dpad_up) {
            for (int i = 0; i < positions.length - 1; i += 1) {
                if (motor.getCurrentPosition() < positions[i + 1] && motor.getCurrentPosition() > positions[i]) {
                    if (currentPos == positions[i + 1] &&
                            Math.abs(motor.getCurrentPosition() - positions[i + 1]) <= 40 &&
                            positions.length > i + 2) {
                        currentPos = positions[i + 2];
                    } else {
                        currentPos = positions[i + 1];
                    }
                }
            }
        }
        else if (gamepad.dpad_down) {
            for (int i = 0; i < positions.length - 1; i += 1) {
                if (motor.getCurrentPosition() < positions[i + 1] && motor.getCurrentPosition() > positions[i]) {
                    if (currentPos == positions[i] &&
                            Math.abs(motor.getCurrentPosition() - positions[i]) <= 40 &&
                            0 <= i - 1) {
                        currentPos = positions[i - 1];
                    } else {
                        currentPos = positions[i];
                    }
                }
            }
        }
        telemetry.addData("reference", currentPos);
        telemetry.addData("encoderPos", motor.getCurrentPosition());
        telePID();
    }

    public void runtimeReset() {
        runtime.reset();
    }
    public void testing() {
        telemetry.addData("Lift_encoder: ", motor.getCurrentPosition());
    }
}