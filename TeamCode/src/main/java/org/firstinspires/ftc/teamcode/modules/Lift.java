package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift {
    private LinearOpMode linearOpMode;
    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private DcMotor motor;
    private int posMax = 1000000000;


    public Lift(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        hardwareMap = linearOpMode.hardwareMap;
        telemetry = linearOpMode.telemetry;
        gamepad = linearOpMode.gamepad2;

        motor = hardwareMap.get(DcMotor.class, "lift_motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            if (motor.getCurrentPosition() + 100 <= posMax) {
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
        if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        motor.setPower(-gamepad.left_stick_y);
    }

    public void testing() {
        telemetry.addData("Lift_motor:", motor.getCurrentPosition());
    }
}
