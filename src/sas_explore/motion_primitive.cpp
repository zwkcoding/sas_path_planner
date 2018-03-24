#include "sas_space_explore/motion_primitive.hpp"

namespace hmpl
{
//
MotionPrimitiveSet::MotionPrimitiveSet()
    : m_steer_num(32),
      m_directions(32),
      m_min_steer_step(0),
      m_max_trajectory_length(0) {

}
MotionPrimitiveSet::~MotionPrimitiveSet() {}

void MotionPrimitiveSet::Init(const sas_element::CarParameter& car, int direction_num, int steer_num) {
	m_car = car;
	m_directions = direction_num;
	m_min_orientation_angle = 2 * M_PI / m_directions;
	// consider steer angle with zero angle
	m_steer_num = steer_num + 1;
    m_steer_num = m_steer_num < 3 ? 3 : m_steer_num;
	m_min_steer_step = 2 * m_car.max_steer / steer_num;

	SetFirstEntry();
	CreateFirstEntry();
	FillOtherEntries();
}

void MotionPrimitiveSet::SetFirstEntry() {
    //
	int iteration = cvFloor(m_steer_num / 2);
    // resize top level vector
	motion_primitive_state_sets_.resize(m_directions);
	for(size_t i = 0; i < m_directions; i++) {
        // resize each of the contained vectors
	    motion_primitive_state_sets_[i].resize(m_steer_num);
	}

	// steer=0, orientation=0,
	// vehicle towards + x axle
	int n = 0;
	motion_primitive_state_sets_[0][n].steering_center_position.x			= 0;
	motion_primitive_state_sets_[0][n].steering_center_position.y			= 0;
	motion_primitive_state_sets_[0][n].steering_radius			= 0;
	motion_primitive_state_sets_[0][n].beginning_heading_index		= 0;
	motion_primitive_state_sets_[0][n].ending_heading_index		= 0;
	motion_primitive_state_sets_[0][n].delta_heading_radian      = 0;
	motion_primitive_state_sets_[0][n].steering_angle_radian		= 0;
	motion_primitive_state_sets_[0][n].arc_length			    = m_car.wheelbase *
			                                  m_min_orientation_angle / m_min_steer_step;
	motion_primitive_state_sets_[0][n].ending_position.x			= motion_primitive_state_sets_[0][n].arc_length;
	motion_primitive_state_sets_[0][n].ending_position.y			= 0;
	m_max_trajectory_length = motion_primitive_state_sets_[0][0].arc_length;

    // counter clockwise trajectories
	// odd number scope [1, direction fraction(e.g 32) -1]
	n = 1;
	for(int j = 1; j <= iteration; j++) {
	    double R = m_car.wheelbase / tan(m_min_steer_step * j);
		motion_primitive_state_sets_[0][n].steering_center_position.x		= 0;
		motion_primitive_state_sets_[0][n].steering_center_position.y		= R;
		motion_primitive_state_sets_[0][n].steering_radius		= R;
		motion_primitive_state_sets_[0][n].beginning_heading_index	= 0;
		//  ending_heading_index mean end_orientation
		motion_primitive_state_sets_[0][n].ending_heading_index	= MIN(j, m_directions / 2);
		motion_primitive_state_sets_[0][n].delta_heading_radian  = MIN(j, m_directions / 2) * m_min_orientation_angle;
		motion_primitive_state_sets_[0][n].steering_angle_radian	= m_min_steer_step * j;
		motion_primitive_state_sets_[0][n].arc_length			= m_min_orientation_angle * R
											  * MIN(j, m_directions / 2);
		motion_primitive_state_sets_[0][n].ending_position.x		= sin(m_min_orientation_angle * MIN(j, m_directions / 2)) * R;
		motion_primitive_state_sets_[0][n].ending_position.y		= R - cos(m_min_orientation_angle * MIN(j, m_directions / 2)) * R;
		n += 2;
	}

    // clockwise trajectories
	// even number scope [2, direction fraction(e.g 32)]
	n = 2;
	for(int j = 1; j <= iteration; j++) {
		double R = m_car.wheelbase / tan(m_min_steer_step * j);
		motion_primitive_state_sets_[0][n].steering_center_position.x		= 0;
		motion_primitive_state_sets_[0][n].steering_center_position.y		= -R;
		motion_primitive_state_sets_[0][n].steering_radius		= R;
		motion_primitive_state_sets_[0][n].beginning_heading_index	= 0;
		motion_primitive_state_sets_[0][n].ending_heading_index	= m_directions - MIN(j, m_directions / 2);
		motion_primitive_state_sets_[0][n].steering_angle_radian	= -m_min_steer_step * j;
		motion_primitive_state_sets_[0][n].delta_heading_radian  = -MIN(j, m_directions / 2) * m_min_orientation_angle;
		motion_primitive_state_sets_[0][n].arc_length			= m_min_orientation_angle * R * MIN(j, m_directions / 2);
		motion_primitive_state_sets_[0][n].ending_position.x		= sin(m_min_orientation_angle * MIN(j, m_directions / 2)) * R;
		motion_primitive_state_sets_[0][n].ending_position.y		= cos(m_min_orientation_angle * MIN(j, m_directions / 2)) * R - R;
		n += 2;
	}
}

void MotionPrimitiveSet::CreateFirstEntry() {
	Vector2D<double> tmppoint{}, origin_point{};
    // horizontal right : x +
    // vertical upwards : y +
	for(int n = 0; n < m_steer_num; n++) {
        // counter clockwise
		if(motion_primitive_state_sets_[0][n].steering_angle_radian > 0) {
			double angle = motion_primitive_state_sets_[0][n].ending_heading_index * 2 * M_PI / m_directions;
			double r = motion_primitive_state_sets_[0][n].steering_radius;
			// unit arc length(constant = 1) sampling
			for(double a = 0; a <= angle; a += 1/r) {
                tmppoint = sas_element::RotateWithCenter(a, origin_point,motion_primitive_state_sets_[0][n].steering_center_position);
				motion_primitive_state_sets_[0][n].primitive_path.push_back(tmppoint);
			}
		}
        // clockwise
		else if (motion_primitive_state_sets_[0][n].steering_angle_radian < 0) {
			double angle = (motion_primitive_state_sets_[0][n].ending_heading_index-m_directions) * 2 * M_PI / m_directions;
			double r = motion_primitive_state_sets_[0][n].steering_radius;
			for(double a = 0; a >= angle; a -= 1/r) {
				tmppoint = sas_element::RotateWithCenter(a, origin_point, motion_primitive_state_sets_[0][n].steering_center_position);
				motion_primitive_state_sets_[0][n].primitive_path.push_back(tmppoint);
			}
		}
		// straight move without steering
		else {
			for(int i = 0; i < motion_primitive_state_sets_[0][0].ending_position.x; i++) {
				tmppoint.x = i;
				tmppoint.y = 0;
				motion_primitive_state_sets_[0][n].primitive_path.push_back(tmppoint);
			}
		}
	}
}

void MotionPrimitiveSet::FillOtherEntries() {
    for(int i = 1; i < m_directions; i++) {
        motion_primitive_state_sets_[i] = motion_primitive_state_sets_[0];
        RotateEntry(motion_primitive_state_sets_[i], i);
    }
}


void  MotionPrimitiveSet::RotateEntry(std::vector<sas_element::MotionPrimitiveState>& entry, int direct) {
    Vector2D<double> origin_point{}, temp_dir{};
    double angle = direct * m_min_orientation_angle;
    // for each steer
    for(int i = 0; i < entry.size(); i++) {
        entry[i].beginning_heading_index = direct;
        entry[i].ending_heading_index = (direct + entry[i].ending_heading_index + m_directions) % m_directions;
        // calculate steering centers position by rotating original steering centers around origin
        temp_dir = sas_element::RotateWithCenter(angle, entry[i].steering_center_position, origin_point);
        entry[i].steering_center_position = temp_dir;

        temp_dir = sas_element::RotateWithCenter(angle, entry[i].ending_position, origin_point);
        entry[i].ending_position = temp_dir;
        // calculate curve of other motion primitive
        for(int m = 0; m < entry[i].primitive_path.size(); m++) {
            temp_dir = sas_element::RotateWithCenter(angle, entry[i].primitive_path[m], origin_point);
            entry[i].primitive_path[m] = temp_dir;
        }
    }
}

sas_element::MotionPrimitiveState *MotionPrimitiveSet::GetStateOfPrimitive(int orientation_index, int steer_index) {
    return &motion_primitive_state_sets_[orientation_index][steer_index];
}

double MotionPrimitiveSet::GetMaximumStepLength() {
    return m_max_trajectory_length;
}

int MotionPrimitiveSet::GetSteerNum() {
    return m_steer_num;
}

} // namespace hmpl
