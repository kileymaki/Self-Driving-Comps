#ifndef _sdcVisibleObject_hh_
#define _sdcVisibleObject_hh_

#include <gazebo/common/common.hh>
#include "sdcAngle.hh"
#include "sdcLidarRay.hh"

namespace gazebo
{
    class sdcVisibleObject {
    public:
        sdcLidarRay left;
        sdcLidarRay right;
        double dist;
        double lineSlope;
        double lineIntercept;

        sdcVisibleObject();
        sdcVisibleObject(sdcLidarRay right, sdcLidarRay left, double dist);

        bool IsSameObject(sdcVisibleObject other);
        math::Vector2d EstimateUpdate();
        math::Vector2d GetProjectedPosition(int numSteps);
        void Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist);
        void Update(sdcVisibleObject newObject);

        void SetTracking(bool isTracking);
        bool IsTracking();
        math::Vector2d GetCenterPoint();
        double GetEstimatedSpeed();
        double GetEstimatedYSpeed();
        double GetEstimatedXSpeed();

        math::Vector2d FitLineToPoints(std::vector<math::Vector2d> points, math::Vector2d newPoint);

    private:
        static const double UNCERTAINTY_RATIO;

        math::Vector2d centerpoint;

        std::vector<math::Vector2d> prevPoints;

        double estimatedXSpeed;
        double estimatedYSpeed;

        double confidence;

        bool tracking;
        bool brandSpankinNew;

        math::Vector2d GetCenterPoint(sdcLidarRay left, sdcLidarRay right, double dist);
    };
}
#endif
