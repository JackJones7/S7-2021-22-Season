package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class CarouselSideAutonomusRed extends CarosuelSideAutonomus{

    //set team to red team
    @Override
    public void setTeam() {
        blueTeam = false;
    }

}
