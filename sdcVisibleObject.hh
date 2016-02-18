#ifndef _sdcVisibleObject_hh_
#define _sdcVisibleObject_hh_

#include <gazebo/common/common.hh>
#include "sdcAngle.hh"
#include "sdcLidarRay.hh"

namespace gazebo
{
    class sdcVisibleObject {
    public:
        sdcVisibleObject(sdcLidarRay right, sdcLidarRay left, double dist);
        bool IsSameObject(sdcVisibleObject other);
        math::Vector2d EstimateUpdate();
        void Update(sdcLidarRay newLeft, sdcLidarRay newRight, double newDist);
        void Update(sdcVisibleObject newObject);
        void SetTracking(bool isTracking);
        math::Vector2d GetCenterPoint();
        math::Vector2d GetCenterPoint(sdcLidarRay left, sdcLidarRay right);

    private:
        static const double UNCERTAINTY_RATIO;

        sdcLidarRay left;
        sdcLidarRay right;
        double dist;

        math::Vector2d centerpoint;

        double estimatedSpeed;
        sdcAngle estimatedDirection;
        public: double confidence;

        bool tracking;
    };
}
#endif
