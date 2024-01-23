package org.firstinspires.ftc.teamcode.modules;


import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.AIM_POS;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.LIFT_POS;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.MAX_POS;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.MIN_POS;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.SPEED;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.kD;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.kG;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.kI;
import static org.firstinspires.ftc.teamcode.modules.PullUpConstants.kP;

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
    private double reference = 0;
    private double lastError = 0;
    private double integralSum = 0;
    private boolean holding = false;
    private ButtonState dpadDownState = ButtonState.RELEASED;
    private ButtonState dpadUpState = ButtonState.RELEASED;
    private enum PullUpState {
        DOWN,
        AIMING,
        CATCH,
        LIFT
    }
    private PullUpState pullUpState = PullUpState.DOWN;
    public static int pos = 0;


    public PullUp(LinearOpMode linearOpMode, FtcDashboard dashboard) {
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;
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

    public void PIDControl() {
        double encoderPos = motor.getCurrentPosition();
        double error = reference - encoderPos;
        double derivative = (error - lastError) / timer.seconds();
        integralSum += error * timer.seconds();
        double out = (kP * error) + (kI * integralSum) + (kD * derivative) + kG;

        motor.setPower(out);

        lastError = error;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("reference_pull", reference);
        packet.put("encoder", encoderPos);
        packet.put("error", error);
        packet.put("power_out", out);
        packet.put("seconds_per_call", timer.seconds());
        dashboard.sendTelemetryPacket(packet);

        timer.reset();
    }

    public void PIDTesting() {
        reference = pos;
        PIDControl();
    }

    public void opControlPos() {
        switch (dpadUpState) {
            case PRESSED:
                switch (pullUpState) {
                    case DOWN: pullUpState = PullUpState.AIMING;
                        break;
                    case AIMING: pullUpState = PullUpState.CATCH;
                        break;
                    case CATCH: pullUpState = PullUpState.LIFT;
                        break;
                    case LIFT: pullUpState = PullUpState.CATCH;
                        break;
                }
                if (gamepad.x) {
                    dpadUpState = ButtonState.HELD;
                } else {
                    dpadUpState = ButtonState.RELEASED;
                }
                break;
            case HELD:
                if (!gamepad.x) {
                    dpadUpState = ButtonState.RELEASED;
                }
                break;
            case RELEASED:
                if (gamepad.x) {
                    dpadUpState = ButtonState.PRESSED;
                }
                break;
        }

        switch (pullUpState) {
            case DOWN: reference = MIN_POS;
                break;
            case AIMING: reference = AIM_POS;
                break;
            case CATCH: reference = MAX_POS;
                break;
            case LIFT: reference = LIFT_POS;
                break;
        }

        telemetry.addData("ButtonUpState", dpadUpState);
        reference += (-gamepad.right_stick_y) * SPEED * timer.seconds();
        if (-gamepad.right_stick_y < 0) {
            pullUpState = PullUpState.DOWN;
        }

        PIDControl();
    }

}
