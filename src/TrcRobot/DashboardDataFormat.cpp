#include "DashboardDataFormat.h"
#include "RobotInfo.h"

DashboardDataFormat::DashboardDataFormat(void)
{
}

DashboardDataFormat::~DashboardDataFormat()
{
}

void DashboardDataFormat::SendVisionData(
        vector<ParticleAnalysisReport> *particles,
        int  vertIdx,
        int  horiIdx,
        bool isHot)
{
    Dashboard &dash =
            DriverStation::GetInstance()->GetHighPriorityDashboardPacker();
    
    //
    // Vision target Info.
    //
    dash.AddCluster();                  //Begin: Target Info
    {
        int targetsSent = 0;

        if (vertIdx != -1)
        {
            ParticleAnalysisReport *p = &(particles->at(vertIdx));

            dash.AddCluster();      //  Begin: Target Rect
            {
                //
                // The dashboard will select color based on this bool.
                // It will highlight it with red when Hot.
                //
                dash.AddBoolean(!isHot);  
                dash.AddU32(p->boundingRect.left);
                dash.AddU32(p->boundingRect.top);
                dash.AddU32(p->boundingRect.left +
                            p->boundingRect.width);
                dash.AddU32(p->boundingRect.top +
                            p->boundingRect.height);
            }
            dash.FinalizeCluster(); //  End: Target Rect
            targetsSent++;
        }
        
        if (horiIdx != -1)
        {
            ParticleAnalysisReport *p = &(particles->at(horiIdx));

            dash.AddCluster();      //  Begin: Target Rect
            {
                //
                // The dashboard will select color based on this bool
                // It will highlight it with red when Hot.
                //
                dash.AddBoolean(!isHot);  
                dash.AddU32(p->boundingRect.left);
                dash.AddU32(p->boundingRect.top);
                dash.AddU32(p->boundingRect.left +
                            p->boundingRect.width);
                dash.AddU32(p->boundingRect.top +
                            p->boundingRect.height);
            }
            dash.FinalizeCluster(); //  End: Target Rect
            targetsSent++;
        }

        //
        // If we have less than 4 targets, we need to send null info for
        // the rest.
        //
        while (targetsSent < 4)
        {
            targetsSent++;
            dash.AddCluster();          //  Begin: Target Rect
            {
                dash.AddBoolean(false);
                dash.AddU32(0);
                dash.AddU32(0);
                dash.AddU32(0);
                dash.AddU32(0);
            }
            dash.FinalizeCluster();     //  End: Target Rect
        }
    }
    dash.FinalizeCluster();             //End: Target Info
    dash.Finalize();
}

void DashboardDataFormat::SendLowPriorityData(float speed)
{
    Dashboard &dash =
            DriverStation::GetInstance()->GetLowPriorityDashboardPacker();
    
//    DriverStationLCD::GetInstance()->PrintfLine(DriverStationLCD::kUser_Line6, "sent: %5.2f", speed);
    dash.AddCluster();
    {
        dash.AddFloat(speed);
    }
    dash.FinalizeCluster();
    dash.Finalize();
}
