package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous

public class WarehouseSideAutonomusRed extends WarehouseSideAutonomus {

    @Override
    public void setTeam() {
        blueTeam = false;
    }
} 
