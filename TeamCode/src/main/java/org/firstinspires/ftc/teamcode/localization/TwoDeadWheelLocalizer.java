package org.firstinspires.ftc.teamcode.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;

@Config
public final class TwoDeadWheelLocalizer implements Localizer {
    public static class Params {
        public double parallelYTicks = 0.0; // Y position of the parallel encoder (tick units)
        public double perpXTicks = 0.0;     // X position of the perpendicular encoder (tick units)
    }

    public static Params PARAMS = new Params();
    public final Encoder parallel, perp;
    public final double inPerTick;
    private int lastParallelPos, lastPerpPos;
    private boolean initialized;

    public TwoDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick) {
        parallel = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "intake")));

        // Set direction if needed
        parallel.setDirection(DcMotorSimple.Direction.REVERSE);

        this.inPerTick = inPerTick;

        FlightRecorder.write("TWO_DEAD_WHEEL_PARAMS", PARAMS);
    }

    public Twist2dDual<Time> update() {
        PositionVelocityPair parallelPosVel = parallel.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("TWO_DEAD_WHEEL_INPUTS", new TwoDeadWheelInputsMessage(parallelPosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastParallelPos = parallelPosVel.position;
            lastPerpPos = perpPosVel.position;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        int parallelPosDelta = parallelPosVel.position - lastParallelPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<>(new double[]{parallelPosDelta * inPerTick, parallelPosVel.velocity * inPerTick}),
                        new DualNum<>(new double[]{perpPosDelta * inPerTick, perpPosVel.velocity * inPerTick})
                ),
                new DualNum<>(new double[]{0.0, 0.0}) // No rotation can be inferred from 2 wheel odometry alone
        );

        lastParallelPos = parallelPosVel.position;
        lastPerpPos = perpPosVel.position;

        return twist;
    }
}

