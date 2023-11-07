package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.misc.RobotConst.kP;
import static org.firstinspires.ftc.teamcode.misc.RobotConst.kI;
import static org.firstinspires.ftc.teamcode.misc.RobotConst.kD;
import static org.firstinspires.ftc.teamcode.misc.RobotConst.kG;
import static org.firstinspires.ftc.teamcode.misc.RobotConst.pos;

public class Lift {
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotorEx motor;
    private ElapsedTime runtime;
    private FtcDashboard dashboard;

    private int maxPos = 600;
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

    public void easyTele() {
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(0.5);
        if (-gamepad.left_stick_y == 0) {
            motor.setTargetPosition(motor.getCurrentPosition());
        }
        else if (-gamepad.left_stick_y > 0) {
            if (motor.getCurrentPosition() + 100 <= maxPos) {
                motor.setTargetPosition(motor.getCurrentPosition() + 100);
            }
        }
        else if (-gamepad.left_stick_y < 0) {
            if (motor.getCurrentPosition() - 100 >= 0) {
                motor.setTargetPosition(motor.getCurrentPosition() - 100);
            }
            else {
                motor.setTargetPosition(0);
            }
        }
        // motor.setPower(-gamepad.left_stick_y);
        telemetry.addData("Lift targeting: ", motor.getTargetPosition());
    }

    public void tele() {
        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            motor.setTargetPosition(590);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        motor.setPower(1);
    }

    public void telePID() {
        currentPos += (-gamepad.left_stick_y) * 5000 * runtime.seconds();
        if (currentPos > maxPos) {
            currentPos = maxPos;
        }
        if (currentPos < 0) {
            currentPos = 0;
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
        packet.put("out", out);
        telemetry.addData("reference", currentPos);
        telemetry.addData("encoder", encoderPos);
        telemetry.addData("out", out);
        telemetry.addData("error", error);
        telemetry.addData("delta", runtime.seconds());
        dashboard.sendTelemetryPacket(packet);
        runtime.reset();
    }

    public void runtimeReset() {
        runtime.reset();
    }

    public void testing() {
        telemetry.addData("Lift_motor:", motor.getCurrentPosition());
    }
}