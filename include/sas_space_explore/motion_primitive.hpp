#ifndef INCLUDE_SAS_SPACE_EXPLORE_MOTION_PRIMITIVE_HPP_
#define INCLUDE_SAS_SPACE_EXPLORE_MOTION_PRIMITIVE_HPP_

#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <opt_utils/circle.hpp>
#include <opt_utils/vector2d.hpp>
#include "sas_space_explore/basic_element_utils.hpp"

namespace hmpl
{
class MotionPrimitiveSet
{
 public:
    // default constructor
    MotionPrimitiveSet();
    // default destructor
	~MotionPrimitiveSet();
    /**
     * Initialize motion primitive for every heading solution angle and
     * steer solution angle
     * @param car car geometry parameters
     * @param direction_num used to calculate heading solution
     * @param steer_num  used to calcalate steer solution angle, must even, can't equal zero
     */
	void Init(const sas_element::CarParameter& car, int direction_num, int steer_num);
    /**
     * Get curves of required heading and steering angle's motion primitive
     * @param orientation_index required index of heading angle
     * @param steer_index required index of steering angle
     * @return curves of corresponding motion primitive
     */
    sas_element::MotionPrimitiveState *GetStateOfPrimitive(int orientation_index, int steer_index);
    // numbers of steer solution angles
	int GetSteerNum();
	// get max length of motion primitive's curve
	double GetMaximumStepLength();
    // heading solution angle (radian)
	double m_min_orientation_angle;
    // steering solution angle (radian)
    double m_min_steer_step;

 private:
	// calculate motion primitive states when heading angle equal zero
	void SetFirstEntry();
	// use constant arc length sampling rule to sample the motion primitive
	// for heading angle equal zero
	void CreateFirstEntry();
	// calculate other heading solution angle's motion primitive's curves
	// by rotating Zero heading angle's motion primitive's curves
	void FillOtherEntries();
	/**
	 * Rotate Zero heading angle's motion primitive's curve
	 * @param entry Zero heading angle's motion primitive
	 * @param direct Rotation angle
	 */
	void RotateEntry(std::vector<sas_element::MotionPrimitiveState>& entry, int direct);
    // car geometry parameters
    sas_element::CarParameter m_car;
	// curves of motion primitive
	std::vector<std::vector<sas_element::MotionPrimitiveState> > motion_primitive_state_sets_;
	// numbers of steer solution angles
	int m_steer_num;
	// max length of motion primitive's curve
	double m_max_trajectory_length;
	// solution numbers of heading angle
	int m_directions;
};

} // namespace hmpl

#endif /* INCLUDE_SAS_SPACE_EXPLORE_MOTION_PRIMITIVE_HPP_ */
