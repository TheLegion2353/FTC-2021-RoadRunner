package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(800);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				.setStartPose(new Pose2d(12.0, 62.0, -Math.PI / 2.0))
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

				.followTrajectorySequence(drive ->
						drive.trajectorySequenceBuilder(drive.getPoseEstimate())
								//.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
								.lineToConstantHeading(new Vector2d(-14.0, 62.0))  // go in front of the shipping hub
								.resetVelConstraint()
								.addTemporalMarker(() -> {
									//arm.setPosition(1.2);
									//wrist.setPosition(0.36);
								})
								.waitSeconds(5)
								.lineToConstantHeading(new Vector2d(-14.0, 43.0))  // go in front of the shipping hub
								.addTemporalMarker(() -> {
									//intake.setPosition(0.0);
								})
								.waitSeconds(0.5)
								.lineToConstantHeading(new Vector2d(-14.0, 53.5))
								.addTemporalMarker(() -> {
									//arm.setPosition(.67);
									//wrist.setPosition(0.0);
									//intake.setPosition(1.0);
								})
								.waitSeconds(2)
								.addTemporalMarker(() -> {
									//arm.kill();
									//wrist.kill();
								})
								.lineToLinearHeading(new Pose2d(-14.00, 59.0, Math.PI))  // Rotate
								.UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
									//carousel.setSpeed(-0.5);
								})
								//.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
								.waitSeconds(3.5)
								.addTemporalMarker(() -> {
									//carousel.setSpeed(0.0);
								})
								.lineToConstantHeading(new Vector2d(13.0, 63.5))  // Line up to park
								//.setVelConstraint(drivetrain.getDrivetrain().getVelocityConstraint(10.0, MAX_ANG_VEL, TRACK_WIDTH))
								.lineToConstantHeading(new Vector2d(32, 65))
								.resetVelConstraint()
								.build()
				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}