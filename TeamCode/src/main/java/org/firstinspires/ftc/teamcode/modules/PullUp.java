package org.firstinspires.ftc.teamcode.modules;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.modules.utility.ButtonState;

@Config
public class PullUp {
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private final DcMotor motor;
    public final ElapsedTime timer;
    private final FtcDashboard dashboard;
    private double tempPower = 0;
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0;
    public static double speed = 2000;
    public static double reference = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private boolean holding = false;
    private ButtonState dpadDownState = ButtonState.RELEASED;


    public PullUp(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad1;
        this.dashboard = dashboard;
        timer = new ElapsedTime();

        motor = hardwareMap.get(DcMotor.class, "motor_up");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        telemetry.addLine("PullUp: Initialized");
    }

    public void opControl() {
        if (holding) {
            motor.setPower(tempPower);
        } else {
            motor.setPower(-gamepad.right_stick_y);
        }
        if (gamepad.dpad_down) {
            tempPower = -gamepad.right_stick_y;
        }
        switch (dpadDownState) {
            case PRESSED:
                holding = !holding;
                if (holding) {
                    tempPower = -gamepad.right_stick_y;
                }
                if (gamepad.dpad_down) {
                    dpadDownState = ButtonState.HELD;
                } else {
                    dpadDownState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.dpad_down) {
                    dpadDownState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.dpad_down) {
                    dpadDownState = ButtonState.PRESSED;
                }
                break;
        }
        telemetry.addLine("---------------");
        telemetry.addLine("PullUp:");
        telemetry.addData("holding", holding);
        telemetry.addData("motor_power: ", motor.getPower());
        telemetry.addData("gp1_right_stick_y", -gamepad.right_stick_y);
    }

    public void opControlPID() {
        reference += (-gamepad.left_stick_y) * speed * timer.seconds();
        double encoderPos = motor.getCurrentPosition();
        double error = reference - encoderPos;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative) + kG;

        motor.setPower(out);

        lastError = error;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("reference", reference);
        packet.put("encoder", encoderPos);
        packet.put("error", error);
        packet.put("power_out", out);
        packet.put("seconds_per_call", timer.seconds());
        dashboard.sendTelemetryPacket(packet);

        timer.reset();
    }
}
