package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class CarouselSideAutonomousRed extends CarouselSideAutonomous {

    //set team to red team
    @Override
    public void setTeam() {
        blueTeam = false;
    }

}
