package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Robot.Robot;

@TeleOp(name="Main Mecanum RR", group="Driver Controlled")
public class MainTeleOp extends OpMode {
	private Robot robot = null;

	@Override
	public void init() {
		robot = new Robot(gamepad1, telemetry, hardwareMap);
		robot.setCarouselMotor(hardwareMap.get(DcMotorEx.class, "carousel/frontEncoder"));
		robot.setArm(hardwareMap.get(DcMotorEx.class, "arm/leftEncoder"), hardwareMap.get(AnalogInput.class, "armPot"));
		//robot.setLinearSlide(hardwareMap.get(DcMotorEx.class, "slide"));
		//robot.setIntake(hardwareMap.get(DcMotorEx.class, "intake/rightEncoder"), hardwareMap.get(AnalogInput.class, "armPot"));
	}

	@Override
	public void loop() {
		robot.update();
	}
}
