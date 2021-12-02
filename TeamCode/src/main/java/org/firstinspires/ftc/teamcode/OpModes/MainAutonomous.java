package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Main Autonomous RR", group="Autonomous")
public class MainAutonomous extends LinearOpMode {
	private Robot robot = null;

	@Override
	public void runOpMode() throws InterruptedException {
		robot = new Robot(null, telemetry, hardwareMap);
		robot.setCarouselMotor(hardwareMap.get(DcMotorEx.class, "carousel/frontEncoder"));
		robot.setArm(hardwareMap.get(DcMotorEx.class, "arm/leftEncoder"), hardwareMap.get(AnalogInput.class, "armPot"));
		robot.setLinearSlide(hardwareMap.get(DcMotorEx.class, "slide"));
		//robot.setIntake(hardwareMap.get(DcMotorEx.class, "intake/rightEncoder"));
		waitForStart();

		if (!isStopRequested()) {
			robot.runAuto();
		}
	}
}
