package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

public final class TwoDeadWheelInputsMessage {


    public long timestamp;

    public PositionVelocityPair parallel;
    public PositionVelocityPair perp;

    public TwoDeadWheelInputsMessage(PositionVelocityPair parallel, PositionVelocityPair perp) {
        this.timestamp = System.nanoTime();
        this.parallel = parallel;
        this.perp = perp;
    }
}
