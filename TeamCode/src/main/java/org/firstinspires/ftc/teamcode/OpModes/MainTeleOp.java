package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Main Mecanum RR", group="Driver Controlled")
public class MainTeleOp extends OpMode {
	private SampleMecanumDrive drive = null;
	@Override
	public void init() {
		drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(PoseStorage.currentPos);
	}

	@Override
	public void loop() {
		drive.setWeightedDrivePower(
				new Pose2d(
						-gamepad1.left_stick_y,
						-gamepad1.left_stick_x,
						-gamepad1.right_stick_x
				)
		);
		drive.update();
		Pose2d poseEstimate = drive.getPoseEstimate();
		telemetry.addData("x: ", poseEstimate);
		telemetry.addData("y: ", poseEstimate.getY());
		telemetry.addData("heading: ", poseEstimate.getHeading());
		telemetry.update();
	}
}
