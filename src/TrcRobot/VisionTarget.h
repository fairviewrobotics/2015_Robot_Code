#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Math.h"

//Camera constants used for distance calculation
#define X_IMAGE_RES 320     //X Image resolution in pixels
#define Y_IMAGE_RES 240     //Y Image resolution in pixels, should be 120, 240 or 480
//#define VERT_VIEW_ANGLE 49       //Axis M1013
//#define VERT_VIEW_ANGLE 41.7       //Axis 206 camera
#define VERT_VIEW_ANGLE 37.4  //Axis M1011 camera

//Score limits used for target identification
#define RECTANGULARITY_LIMIT 40
#define ASPECT_RATIO_LIMIT 55

//Score limits used for hot target determination
#define TAPE_WIDTH_LIMIT 50
#define VERTICAL_SCORE_LIMIT 50
#define LR_SCORE_LIMIT 50

//Minimum area of particles to be considered
#define AREA_MINIMUM 150

//Maximum number of particles to process
#define MAX_PARTICLES 8

#define FOCAL_LENGTH    390.37285

struct TargetReport
{
    int verticalIndex;
    int horizontalIndex;
    double totalScore;
    double leftScore;
    double rightScore;
    double tapeWidthScore;
    double verticalScore;
    double distance;
    double offset;
    bool Hot;
};

/**
 *  This module defines and implements the VisionTarget subsytem. The
 *  VisionTarget subsystem consists of an IP camera and a ring light.
 *	It takes a snapshot from the camera and detect the targets in the
 *  snapshot. It scores all the targets and returns the highest score
 *  target.
 */
class VisionTarget: public CoopTask
{
private:
    struct Scores
    {
        double rectangularity;
        double aspectRatioVertical;
        double aspectRatioHorizontal;
    };
    
    DashboardDataFormat*m_dashboardDataFormat;
    Relay               m_ringLight;
    AxisCamera&         m_camera;
    Threshold           m_colorThresholds;
    VisionTask          m_visionTask;
    ParticleFilterCriteria2 m_filterCriteria[1];

public:

    /**
     *  Constructor.
     */
    VisionTarget(DashboardDataFormat *dashboardDataFormat)
        : m_dashboardDataFormat(dashboardDataFormat)
        , m_ringLight(RELAY_RINGLIGHT_POWER, Relay::kForwardOnly)
        , m_camera(AxisCamera::GetInstance(CAMERA_IP))
        //
        // Good green values: RGB(0/100, 80/140, 40/140)
        // Obtained from camera snapshot in mspaint.
        // 2014 Vision sample has the following HSV values:
        //  HSV(105, 137, 230, 255, 133, 183)
        //  Old RGB(0, 150, 10, 160, 20, 160)
        //
         , m_colorThresholds(0, 10, 10, 150, 10, 150)
         , m_visionTask(&m_camera,
                        IMAQ_IMAGE_RGB,
                        &m_colorThresholds,
                        1,       //number of filter criteria
                        false,
                        m_filterCriteria,
                        ARRAYSIZE(m_filterCriteria))
    {
        //
        // Initialize camera.
        //
        m_camera.WriteResolution(AxisCamera::kResolution_320x240);
        m_camera.WriteCompression(20);
        m_camera.WriteBrightness(20);
        //
        // Initialize filter criteria.
        //
        m_filterCriteria[0].parameter = IMAQ_MT_AREA;
        m_filterCriteria[0].lower = AREA_MINIMUM;
        m_filterCriteria[0].upper = 65535;
        m_filterCriteria[0].calibrated = false;
        m_filterCriteria[0].exclude = false;

        RegisterTask(MOD_NAME, TASK_START_MODE | TASK_STOP_MODE);
    }   //VisionTarget

    /**
     *  Destructor.
     */
    virtual
    ~VisionTarget(void)
    {
        m_visionTask.SetTaskEnabled(false);
        m_ringLight.Set(Relay::kOff);
        UnregisterTask();
    }   //~VisionTarget

    void TaskStartMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            m_ringLight.Set(Relay::kOn);
            m_visionTask.SetTaskEnabled(true);
        }
    }   //TaskStartMode

    void TaskStopMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            m_visionTask.SetTaskEnabled(false);
            m_ringLight.Set(Relay::kOff);
        }
    }   //TaskStopMode
    
    /**
     * Computes the estimated distance to a target using the height of the particle in the image. For more information and graphics
     * showing the math behind this approach see the Vision Processing section of the ScreenStepsLive documentation.
     * 
     * @param image The image to use for measuring the particle estimated rectangle
     * @param report The Particle Analysis Report for the particle
     * @return The estimated distance to the target in feet.
     */
    double computeDistance (BinaryImage *image, ParticleAnalysisReport *report)
    {
        double rectLong, height, distance;
        int targetHeight;

        imaqMeasureParticle(image->GetImaqImage(),
                            report->particleIndex,
                            0,
                            IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE,
                            &rectLong);
        //
        //using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
        //on skewed rectangles
        //
        height = min((double)report->boundingRect.height, rectLong);
        targetHeight = 32;
        //
        // (Y_IMAGE_RES/2)/focal_length = tan(VERT_VIEW_ANGLE/2)
        // => focal_length = (Y_IMAGE_RES/2)/tan(VERT_VIEW_ANGLE/2)
        // => focal_length = (240/2)/tan(37.4/2) 
        // => focal_length = 354.5247 
        //
        // height/focal_length = targetHeight/distance
        // => distance = targetHeight*focal_length/height
        // => distance = targetHeight*((Y_IMAGE_RES/2)/tan(VERT_VIEW_ANGLE/2))/height
        // => distance = targetHeight*Y_IMAGE_RES/(2*height*tan(VERT_VIEW_ANGLE/2))
        // => distance = Y_IMAGE_RES*targetHeight/(height*2*tan(VERT_VIEW_ANGLE/2))
        // => distance = Y_IMAGE_RES*targetHeight/(2*tan(VERT_VIEW_ANGLE/2))/height
        // => distance = focal_length*targetHeight/height
        //
        distance = FOCAL_LENGTH*targetHeight/height;
        return distance;
    }

    double computeOffset(ParticleAnalysisReport *report, double distance, double offsetAdj)
    {
        double targetOffset;
        //
        // (X_IMAGE_RES/2)/focal_length = tan(HORI_VIEW_ANGLE/2)
        // => focal_length = (X_IMAGE_RES/2)/tan(HORI_VIEW_ANGLE/2)
        //
        // offset/focal_length = targetOffset/distance
        // => targetOffset = offset*distance/focal_length
        //
        //  where offset is the distance between the particle center and the
        //  the center of the screen.
        // offset = center_mass_x - X_IMAGE_RES/2
        //
        targetOffset = (report->center_mass_x - X_IMAGE_RES/2)*distance/
                       FOCAL_LENGTH + offsetAdj;

        return targetOffset;
    }

    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
     * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
     * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
     * and particle perimeter= 2x+2y
     * 
     * @param image The image containing the particle to score, needed to perform additional measurements
     * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
     * @param outer Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
     * @return The aspect ratio score (0-100)
     */
    double scoreAspectRatio(BinaryImage *image, ParticleAnalysisReport *report, bool vertical)
    {
        double rectLong, rectShort;
        double idealAspectRatio, aspectRatio;
        idealAspectRatio = vertical ? (4.0/32) : (23.5/4);  //Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall
      
        imaqMeasureParticle(image->GetImaqImage(),
                            report->particleIndex,
                            0,
                            IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE,
                            &rectLong);
        imaqMeasureParticle(image->GetImaqImage(),
                            report->particleIndex,
                            0,
                            IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE,
                            &rectShort);
        
        //Divide width by height to measure aspect ratio
        if(report->boundingRect.width > report->boundingRect.height){
            //particle is wider than it is tall, divide long by short
            aspectRatio = ratioToScore(((rectLong/rectShort)/idealAspectRatio));
        } else {
            //particle is taller than it is wide, divide short by long
            aspectRatio = ratioToScore(((rectShort/rectLong)/idealAspectRatio));
        }
        return aspectRatio;     //force to be in range 0-100
    }

    /**
     * Compares scores to defined limits and returns true if the particle appears to be a target
     * 
     * @param scores The structure containing the scores to compare
     * @param vertical True if the particle should be treated as a vertical target, false to treat it as a horizontal target
     * 
     * @return True if the particle meets all limits, false otherwise
     */
    bool scoreCompare(Scores scores, bool vertical){
        bool isTarget = true;

        isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
        if(vertical){
            isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
        } else {
            isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
        }

        return isTarget;
    }
    
    /**
     * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
     * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
     * 
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
    double scoreRectangularity(ParticleAnalysisReport *report){
        if(report->boundingRect.width*report->boundingRect.height !=0){
            return 100*report->particleArea/(report->boundingRect.width*report->boundingRect.height);
        } else {
            return 0;
        }   
    }   
    
    /**
     * Converts a ratio with ideal value of 1 to a score. The resulting function is piecewise
     * linear going from (0,0) to (1,100) to (2,0) and is 0 for all inputs outside the range 0-2
     */
    double ratioToScore(double ratio)
    {
        //
        // ratio is the ratio of the real value to the ideal value.
        // In the perfect world, ratio should be 1.0. To score the ratio,
        // we calculate the difference between the real ratio to the ideal
        // ratio (1.0). The difference should be as close to zero as possible.
        // We then subtract the absolute difference from 1.0 so now we should
        // have a value as close to 1.0 as possible. We multiple this number
        // by 100 to convert it to a percentage value. We then limit this
        // percentage value between 0 and 100.
        //
        return (max(0.0, min(100.0*(1.0-fabs(1.0-ratio)), 100.0)));
    }
    
    /**
     * Takes in a report on a target and compares the scores to the defined score limits to evaluate
     * if the target is a hot target or not.
     * 
     * Returns True if the target is hot. False if it is not.
     */
    bool hotOrNot(TargetReport target)
    {
        bool isHot = true;
        
        isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
        isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
        isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);
        
        return isHot;
    }

    bool GetTargetInfo(TargetReport &target)
    {
        bool fSuccess = false;
        vector<ParticleAnalysisReport> *reports;
        BinaryImage *filteredImage = NULL;

#ifdef _DEBUG_VISIONTARGET
        static UINT32 prevTime = 0;
        UINT32 currTime = GetMsecTime();
        if(prevTime == 0)
        {
            prevTime = currTime;
        }
#endif
        reports = m_visionTask.VisionGetTargets(&filteredImage);
        if ((reports != NULL) && (reports->size() > 0))
        {
            Scores *scores;
            int verticalTargets[MAX_PARTICLES];
            int horizontalTargets[MAX_PARTICLES];
            int verticalTargetCount = 0;
            int horizontalTargetCount = 0;

#ifdef _DEBUG_VISIONTARGET
            if (currTime > prevTime)
            {
                TInfo(("Targeting took %d msec", currTime - prevTime));
                prevTime = currTime;
            }
#endif

            //Iterate through each particle, scoring it and determining whether
            // it is a target or not
            scores = new Scores[reports->size()];
            for (unsigned int i = 0;
                 i < MAX_PARTICLES && i < reports->size();
                 i++)
            {
                ParticleAnalysisReport *report = &(reports->at(i));
                
                //Score each particle on rectangularity and aspect ratio
                scores[i].rectangularity = scoreRectangularity(report);
                scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, true);
                scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, false);

#ifdef _DEBUG_VISIONTARGET
                TInfo(("\n[%d]: Center[%d,%d], Rect[%d,%d/%d,%d]",
                       i, report->center_mass_x, report->center_mass_y,
                       report->boundingRect.left, report->boundingRect.top,
                       report->boundingRect.width, report->boundingRect.height));
                TInfo(("Scores: rect=%5.2f, ARvert=%5.2f, ARhori=%5.2f",
                       scores[i].rectangularity,
                       scores[i].aspectRatioVertical,
                       scores[i].aspectRatioHorizontal));
#endif
                //Check if the particle is a horizontal target, if not, check if it's a vertical target
                if(scoreCompare(scores[i], false))
                {
#ifdef _DEBUG_VISIONTARGET
                    TInfo(("particle: %d is a Horizontal Target.", i));
#endif
                    horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
                }
                else if (scoreCompare(scores[i], true))
                {
#ifdef _DEBUG_VISIONTARGET
                    TInfo(("particle: %d is a Vertical Target.", i));
#endif
                    verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
                }
#ifdef _DEBUG_VISIONTARGET
                else
                {
                    TInfo(("particle: %d is not a Target", i));
                }
#endif
            }

            //Zero out scores and set verticalIndex to first target in case there are no horizontal targets
            target.totalScore = target.leftScore
                              = target.rightScore
                              = target.tapeWidthScore
                              = target.verticalScore
                              = 0.0;
            target.horizontalIndex = -1;
            target.verticalIndex = (verticalTargetCount > 0)? verticalTargets[0]: -1;
            target.distance = target.offset = 0.0;
            target.Hot = false;

            for (int i = 0; i < verticalTargetCount; i++)
            {
                ParticleAnalysisReport *verticalReport = &(reports->at(verticalTargets[i]));
                double vertWidth;

                imaqMeasureParticle(filteredImage->GetImaqImage(),
                                    verticalReport->particleIndex,
                                    0,
                                    IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE,
                                    &vertWidth);

                for (int j = 0; j < horizontalTargetCount; j++)
                {
                    ParticleAnalysisReport *horizontalReport = &(reports->at(horizontalTargets[j]));
                    double horizWidth,
                           horizHeight,
                           leftScore,
                           rightScore,
                           tapeWidthScore,
                           verticalScore,
                           total;

                    //Measure equivalent rectangle sides for use in score calculation
                    imaqMeasureParticle(filteredImage->GetImaqImage(),
                                        horizontalReport->particleIndex,
                                        0,
                                        IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE,
                                        &horizWidth);
                    imaqMeasureParticle(filteredImage->GetImaqImage(),
                                        horizontalReport->particleIndex,
                                        0,
                                        IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE,
                                        &horizHeight);
                    
                    //Determine if the horizontal target is in the expected location to the left of the vertical target
                    leftScore = ratioToScore(1.2*(verticalReport->boundingRect.left - horizontalReport->center_mass_x)/horizWidth);
                    //Determine if the horizontal target is in the expected location to the right of the  vertical target
                    rightScore = ratioToScore(1.2*(horizontalReport->center_mass_x - verticalReport->boundingRect.left - verticalReport->boundingRect.width)/horizWidth);
                    //Determine if the width of the tape on the two targets appears to be the same
                    tapeWidthScore = ratioToScore(vertWidth/horizHeight);
                    //Determine if the vertical location of the horizontal target appears to be correct
                    verticalScore = ratioToScore(1-(verticalReport->boundingRect.top - horizontalReport->center_mass_y)/(4*horizHeight));
                    total = leftScore > rightScore ? leftScore:rightScore;
                    total += tapeWidthScore + verticalScore;

                    //If the target is the best detected so far store the information about it
                    if(total > target.totalScore)
                    {
                        target.horizontalIndex = horizontalTargets[j];
                        target.verticalIndex = verticalTargets[i];
                        target.totalScore = total;
                        target.leftScore = leftScore;
                        target.rightScore = rightScore;
                        target.tapeWidthScore = tapeWidthScore;
                        target.verticalScore = verticalScore;
                    }
                }
                //Determine if the best target is a Hot target
                target.Hot = hotOrNot(target);
            }

            if (target.verticalIndex != -1)
            {
                //Information about the target is contained in the "target" structure
                //To get measurement information such as sizes or locations use the
                //horizontal or vertical index to get the particle report as shown below
                ParticleAnalysisReport *verticalReport = &(reports->at(target.verticalIndex));
                target.distance = computeDistance(filteredImage,
                                                  verticalReport);
                target.offset = computeOffset(verticalReport, target.distance, 0.0);

#ifdef _DEBUG_VISIONTARGET
                TInfo(("Target acquired: vertIdx=%d, horiIdx=%d",
                       target.verticalIndex, target.horizontalIndex));
                TInfo(("\ttotalScore=%5.2f, leftScore=%5.2f, rightScore=%5.2f",
                       target.totalScore,
                       target.leftScore,
                       target.rightScore));
                TInfo(("\ttapeWidthScore=%5.2f, verticalScore=%5.2f, Hot=%d",
                       target.tapeWidthScore,
                       target.verticalScore,
                       target.Hot));
                TInfo(("\ttargetDistance=%5.2f, targetOffset=%5.2f",
                       target.distance,
                       target.offset));
#endif
                fSuccess = true;
            }
        
            m_dashboardDataFormat->SendVisionData(reports,
                                                  target.verticalIndex,
                                                  target.horizontalIndex,
                                                  target.Hot);

            SAFE_DELETE(scores);
        }

        SAFE_DELETE(filteredImage);
        SAFE_DELETE(reports);

        return fSuccess;
    }   //GetTargetInfo

}; //class VisionTarget
