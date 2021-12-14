/****************************************************************************
 *
 * Copyright (c) 2021 Autonomous Systems Lab, ETH Zurich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file npfg.cpp
 * Implementation of a lateral-directional nonlinear path following guidance
 * law with excess wind handling.
 *
 * Authors and acknowledgements in header.
 */

#include "npfg.hpp"
#include <lib/ecl/geo/geo.h>
#include <px4_platform_common/defines.h>
#include <float.h>

using matrix::Vector2d;
using matrix::Vector2f;

void NPFG::evaluate(const Vector2f &ground_vel, const Vector2f &wind_vel, Vector2f &unit_path_tangent,
		    float signed_track_error, const float path_curvature, bool in_front_of_wp /*= false*/,
		    const Vector2f &bearing_vec_to_point /*= Vector2f{0.0f, 0.0f}*/)
{
	const float ground_speed = ground_vel.norm();

	Vector2f air_vel = ground_vel - wind_vel;
	const float airspeed = air_vel.norm();

	const float wind_speed = wind_vel.norm();

	float track_error = fabsf(signed_track_error);

	// track error bound is dynamic depending on ground speed
	track_error_bound_ = trackErrorBound(ground_speed, time_const_);
	float normalized_track_error = normalizedTrackError(track_error, track_error_bound_);

	// look ahead angle based purely on track proximity
	float look_ahead_ang = lookAheadAngle(normalized_track_error);

	bearing_vec_ = bearingVec(unit_path_tangent, look_ahead_ang, signed_track_error);

	// on-track wind triangle projections
	float wind_cross_upt = cross2D(wind_vel, unit_path_tangent);
	float wind_dot_upt = wind_vel.dot(unit_path_tangent);

	// calculate the bearing feasibility on the track at the current closest point
	feas_on_track_ = bearingFeasibility(wind_cross_upt, wind_dot_upt, airspeed, wind_speed);

	// update control parameters considering upper and lower stability bounds (if enabled)
	// must be called before trackErrorBound() as it updates time_const_
	adapted_period_ = adaptPeriod(ground_speed, airspeed, wind_speed, track_error,
				      path_curvature, wind_vel, unit_path_tangent, feas_on_track_);
	p_gain_ = pGain(adapted_period_, damping_);
	time_const_ = timeConst(adapted_period_, damping_);

	// specific waypoint logic complications... handles case where we are following
	// waypoints and are in front of the first of the segment.
	// TODO: find a way to get this out of the main eval method!
	if (in_front_of_wp && unit_path_tangent.dot(bearing_vec_) < unit_path_tangent.dot(bearing_vec_to_point)) {
		// we are in front of the first waypoint and the bearing to the point is
		// shallower than that to the line. reset path params to fly directly to
		// the first waypoint.

		// TODO: probably better to blend these instead of hard switching (could
		// effect the adaptive tuning if we switch between these cases with wind
		// gusts)

		// pretend we are on track, recalculate necessary quantities

		signed_track_error = 0.0f;
		normalized_track_error = 0.0f;
		unit_path_tangent = bearing_vec_to_point;

		bearing_vec_ = bearing_vec_to_point;
		look_ahead_ang = 0.0f;

		wind_cross_upt = cross2D(wind_vel, unit_path_tangent);
		wind_dot_upt = wind_vel.dot(unit_path_tangent);
		feas_on_track_ = bearingFeasibility(wind_cross_upt, wind_dot_upt, airspeed, wind_speed);

		adapted_period_ = adaptPeriod(ground_speed, airspeed, wind_speed, track_error,
					      path_curvature, wind_vel, unit_path_tangent, feas_on_track_);
		p_gain_ = pGain(adapted_period_, damping_);
		time_const_ = timeConst(adapted_period_, damping_);
	}

	track_proximity_ = trackProximity(look_ahead_ang);

	// wind triangle projections
	const float wind_cross_bearing = cross2D(wind_vel, bearing_vec_);
	const float wind_dot_bearing = wind_vel.dot(bearing_vec_);

	// continuous representation of the bearing feasibility
	feas_ = bearingFeasibility(wind_cross_bearing, wind_dot_bearing, airspeed, wind_speed);

	// we consider feasibility of both the current bearing as well as that on the track at the current closest point
	const float feas_combined = feas_ * feas_on_track_;

	min_ground_speed_ref_ = minGroundSpeed(normalized_track_error, feas_combined);

	// reference air velocity with directional feedforward effect for following
	// curvature in wind and magnitude incrementation depending on minimum ground
	// speed violations and/or high wind conditions in general
	air_vel_ref_ = refAirVelocity(wind_vel, bearing_vec_, wind_cross_bearing,
				      wind_dot_bearing, wind_speed, min_ground_speed_ref_);
	airspeed_ref_ = air_vel_ref_.norm();

	// lateral acceleration demand based on heading error
	const float lateral_accel = lateralAccel(air_vel, air_vel_ref_, airspeed);

	// lateral acceleration needed to stay on curved track (assuming no heading error)
	lateral_accel_ff_ = lateralAccelFF(unit_path_tangent, ground_vel, wind_dot_upt,
					   wind_cross_upt, airspeed, wind_speed, signed_track_error, path_curvature);

	// total lateral acceleration to drive aircaft towards track as well as account
	// for path curvature. The full effect of the feed-forward acceleration is smoothly
	// ramped in as the vehicle approaches the track and is further smoothly
	// zeroed out as the bearing becomes infeasible.
	lateral_accel_ = lateral_accel + feas_combined * track_proximity_ * lateral_accel_ff_;
} // evaluate

float NPFG::adaptPeriod(const float ground_speed, const float airspeed, const float wind_speed,
			const float track_error, const float path_curvature, const Vector2f &wind_vel,
			const Vector2f &unit_path_tangent, const float feas_on_track) const
{
	float period = period_;
	const float air_turn_rate = fabsf(path_curvature * airspeed);
	const float wind_factor = windFactor(airspeed, wind_speed);

	if (en_period_lb_) {
		// lower bound for period not considering path curvature
		const float period_lb_zero_curvature = periodLB(0.0f, wind_factor, feas_on_track) * PERIOD_SAFETY_FACTOR;

		// lower bound for period *considering path curvature
		float period_lb = periodLB(air_turn_rate, wind_factor, feas_on_track) * PERIOD_SAFETY_FACTOR;

		// recalculate the time constant and track error bound considering the zero
		// curvature, lower-bounded period and subsequently recalculate the normalized
		// track error
		const float time_const = timeConst(period_lb_zero_curvature, damping_);
		const float track_error_bound = trackErrorBound(ground_speed, time_const);
		const float normalized_track_error = normalizedTrackError(track_error, track_error_bound);

		// calculate nominal track proximity with lower bounded time constant
		// (only a numerical solution can find corresponding track proximity
		// and adapted gains simultaneously)
		const float look_ahead_ang = lookAheadAngle(normalized_track_error);
		const float track_proximity = trackProximity(look_ahead_ang);

		// ramp in curvature dependent lower bound
		period_lb = (ramp_in_adapted_period_) ? period_lb * track_proximity + (1.0f - track_proximity) *
			    period_lb_zero_curvature :
			    period_lb;

		// lower bounded period
		period = math::max(period_lb, period);

		// only allow upper bounding ONLY if lower bounding is enabled (is otherwise
		// dangerous to allow period decrements without stability checks).
		// NOTE: if the roll time constant is not accurately known, lower-bound
		// checks may be too optimistic and reducing the period can still destabilize
		// the system! enable this feature at your own risk.
		if (en_period_ub_) {

			const float period_ub = periodUB(air_turn_rate, wind_factor, feas_on_track);

			if (en_period_ub_ && PX4_ISFINITE(period_ub) && period > period_ub) {
				// NOTE: if the roll time constant is not accurately known, reducing
				// the period here can destabilize the system!
				// enable this feature at your own risk!

				// upper bound the period (for track keeping stability), prefer lower bound if violated
				const float period_adapted = math::max(period_lb, period_ub);

				// transition from the nominal period to the adapted period as we get
				// closer to the track
				period = (ramp_in_adapted_period_) ? period_adapted * track_proximity + (1.0f - track_proximity) * period :
					 period_adapted;
			}
		}
	}

	return period;
} // adaptPeriod

float NPFG::normalizedTrackError(const float track_error, const float track_error_bound) const
{
	return math::constrain(track_error / track_error_bound, 0.0f, 1.0f);
}

float NPFG::windFactor(const float airspeed, const float wind_speed) const
{
	// See [TODO: include citation] for definition/elaboration of this approximation.
	if (wind_speed > airspeed || airspeed < EPSILON) {
		return 2.0f;

	} else {
		return 2.0f * (1.0f - sqrtf(1.0f - math::min(1.0f, wind_speed / airspeed)));
	}
} // windFactor

float NPFG::periodUB(const float air_turn_rate, const float wind_factor, const float feas_on_track) const
{
	if (air_turn_rate * wind_factor > EPSILON) {
		// multiply air turn rate by feasibility on track to zero out when we anyway
		// should not consider the curvature
		return 4.0f * M_PI_F * damping_ / (air_turn_rate * wind_factor * feas_on_track);
	}

	return INFINITY;
} // periodUB

float NPFG::periodLB(const float air_turn_rate, const float wind_factor, const float feas_on_track) const
{
	// this method considers a "conservative" lower period bound, i.e. a constant
	// worst case bound for any wind ratio, airspeed, and path curvature

	// the lower bound for zero curvature and no wind OR damping ratio < 0.5
	const float period_lb = M_PI_F * roll_time_const_ / damping_;

	if (air_turn_rate * wind_factor < EPSILON || damping_ < 0.5f) {
		return period_lb;

	} else {
		// the lower bound for tracking a curved path in wind with damping ratio > 0.5
		const float period_windy_curved_damped = 4.0f * M_PI_F * roll_time_const_ * damping_;

		// blend the two together as the bearing on track becomes less feasible
		return period_windy_curved_damped * feas_on_track + (1.0f - feas_on_track) * period_lb;
	}
} // periodLB

float NPFG::trackProximity(const float look_ahead_ang) const
{
	const float sin_look_ahead_ang = sinf(look_ahead_ang);
	return sin_look_ahead_ang * sin_look_ahead_ang;
} // trackProximity

float NPFG::trackErrorBound(const float ground_speed, const float time_const) const
{
	if (ground_speed > 1.0f) {
		return ground_speed * time_const;

	} else {
		// limit bound to some minimum ground speed to avoid singularities in track
		// error normalization. the following equation assumes ground speed minimum = 1.0
		return 0.5f * time_const * (ground_speed * ground_speed + 1.0f);
	}
} // trackErrorBound

float NPFG::pGain(const float period, const float damping) const
{
	return 4.0f * M_PI_F * damping / period;
} // pGain

float NPFG::timeConst(const float period, const float damping) const
{
	return period * damping;
} // timeConst

float NPFG::lookAheadAngle(const float normalized_track_error) const
{
	return M_PI_F * 0.5f * (normalized_track_error - 1.0f) * (normalized_track_error - 1.0f);
} // lookAheadAngle

Vector2f NPFG::bearingVec(const Vector2f &unit_path_tangent, const float look_ahead_ang,
			  const float signed_track_error) const
{
	const float cos_look_ahead_ang = cosf(look_ahead_ang);
	const float sin_look_ahead_ang = sinf(look_ahead_ang);

	Vector2f unit_path_normal(-unit_path_tangent(1.0f), unit_path_tangent(0.0f)); // right handed 90 deg (clockwise) turn
	Vector2f unit_track_error = -((signed_track_error < 0.0f) ? -1.0f : 1.0f) * unit_path_normal;

	return cos_look_ahead_ang * unit_track_error + sin_look_ahead_ang * unit_path_tangent;
} // bearingVec

float NPFG::minGroundSpeed(const float normalized_track_error, const float feas)
{
	// minimum ground speed demand from track keeping logic
	min_gsp_track_keeping_ = 0.0f;

	if (en_track_keeping_ && en_wind_excess_regulation_) {
		// zero out track keeping speed increment when bearing is feasible
		min_gsp_track_keeping_ = (1.0f - feas) * min_gsp_track_keeping_max_ * math::constrain(
						 normalized_track_error * inv_nte_fraction_, 0.0f,
						 1.0f);
	}

	// minimum ground speed demand from minimum forward ground speed user setting
	float min_gsp_desired = 0.0f;

	if (en_min_ground_speed_ && en_wind_excess_regulation_) {
		min_gsp_desired = min_gsp_desired_;
	}

	return math::max(min_gsp_track_keeping_, min_gsp_desired);
} // minGroundSpeed

Vector2f NPFG::refAirVelocity(const Vector2f &wind_vel, const Vector2f &bearing_vec,
			      const float wind_cross_bearing, const float wind_dot_bearing, const float wind_speed,
			      const float min_ground_speed) const
{
	Vector2f air_vel_ref;

	if (min_ground_speed > wind_dot_bearing && (en_min_ground_speed_ || en_track_keeping_) && en_wind_excess_regulation_) {
		// minimum ground speed and/or track keeping

		// airspeed required to achieve minimum ground speed along bearing vector
		const float airspeed_min = sqrtf((min_ground_speed - wind_dot_bearing) * (min_ground_speed - wind_dot_bearing) +
						 wind_cross_bearing * wind_cross_bearing);

		if (airspeed_min > airspeed_max_) {
			if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed)) {
				// we will not maintain the minimum ground speed, but can still achieve the bearing at maximum airspeed
				const float airsp_dot_bearing = projectAirspOnBearing(airspeed_max_, wind_cross_bearing);
				air_vel_ref = solveWindTriangle(wind_cross_bearing, airsp_dot_bearing, bearing_vec);

			} else {
				// bearing is maximally infeasible, employ mitigation law
				air_vel_ref = infeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed_max_);
			}

		} else if (airspeed_min > airspeed_nom_) {
			// the minimum ground speed is achievable within the nom - max airspeed range
			// solve wind triangle with for air velocity reference with minimum airspeed
			const float airsp_dot_bearing = projectAirspOnBearing(airspeed_min, wind_cross_bearing);
			air_vel_ref = solveWindTriangle(wind_cross_bearing, airsp_dot_bearing, bearing_vec);

		} else {
			// the minimum required airspeed is less than nominal, so we can track the bearing and minimum
			// ground speed with our nominal airspeed reference
			const float airsp_dot_bearing = projectAirspOnBearing(airspeed_nom_, wind_cross_bearing);
			air_vel_ref = solveWindTriangle(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
		}

	} else {
		// wind excess regulation and/or mitigation

		if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_nom_, wind_speed)) {
			// bearing is nominally feasible, solve wind triangle for air velocity reference using nominal airspeed
			const float airsp_dot_bearing = projectAirspOnBearing(airspeed_nom_, wind_cross_bearing);
			air_vel_ref = solveWindTriangle(wind_cross_bearing, airsp_dot_bearing, bearing_vec);

		} else if (bearingIsFeasible(wind_cross_bearing, wind_dot_bearing, airspeed_max_, wind_speed)
			   && en_wind_excess_regulation_) {
			// bearing is maximally feasible
			if (wind_dot_bearing <= 0.0f) {
				// we only increment the airspeed to regulate, but not overcome, excess wind
				// NOTE: in the terminal condition, this will result in a zero ground velocity configuration
				air_vel_ref = wind_vel;

			} else {
				// the bearing is achievable within the nom - max airspeed range
				// solve wind triangle with for air velocity reference with minimum airspeed
				const float airsp_dot_bearing = 0.0f; // right angle to the bearing line gives minimal airspeed usage
				air_vel_ref = solveWindTriangle(wind_cross_bearing, airsp_dot_bearing, bearing_vec);
			}

		} else {
			// bearing is maximally infeasible, employ mitigation law
			const float airspeed_input = (en_wind_excess_regulation_) ? airspeed_max_ : airspeed_nom_;
			air_vel_ref = infeasibleAirVelRef(wind_vel, bearing_vec, wind_speed, airspeed_input);
		}
	}

	return air_vel_ref;
} // refAirVelocity

float NPFG::projectAirspOnBearing(const float airspeed, const float wind_cross_bearing) const
{
	// NOTE: wind_cross_bearing must be less than airspeed to use this function
	// it is assumed that bearing feasibility is checked and found feasible (e.g. bearingIsFeasible() = true) prior to entering this method
	// otherwise the return will be erroneous
	return sqrtf(math::max(airspeed * airspeed - wind_cross_bearing * wind_cross_bearing, 0.0f));
} // projectAirspOnBearing

int NPFG::bearingIsFeasible(const float wind_cross_bearing, const float wind_dot_bearing, const float airspeed,
			    const float wind_speed) const
{
	return (fabsf(wind_cross_bearing) < airspeed) && ((wind_dot_bearing > 0.0f) || (wind_speed < airspeed));
} // bearingIsFeasible

Vector2f NPFG::solveWindTriangle(const float wind_cross_bearing, const float airsp_dot_bearing,
				 const Vector2f &bearing_vec) const
{
	// essentially a 2D rotation with the speeds (magnitudes) baked in
	return Vector2f{airsp_dot_bearing * bearing_vec(0) - wind_cross_bearing * bearing_vec(1),
			wind_cross_bearing * bearing_vec(0) + airsp_dot_bearing * bearing_vec(1)};
} // solveWindTriangle

Vector2f NPFG::infeasibleAirVelRef(const Vector2f &wind_vel, const Vector2f &bearing_vec, const float wind_speed,
				   const float airspeed) const
{
	// NOTE: wind speed must be greater than airspeed, and airspeed must be greater than zero to use this function
	// it is assumed that bearing feasibility is checked and found infeasible (e.g. bearingIsFeasible() = false) prior to entering this method
	// otherwise the normalization of the air velocity vector could have a division by zero
	Vector2f air_vel_ref = sqrtf(math::max(wind_speed * wind_speed - airspeed * airspeed, 0.0f)) * bearing_vec - wind_vel;
	return air_vel_ref.normalized() * airspeed;
} // infeasibleAirVelRef

float NPFG::bearingFeasibility(float wind_cross_bearing, const float wind_dot_bearing, const float airspeed,
			       const float wind_speed) const
{
	if (wind_dot_bearing < 0.0f) {
		wind_cross_bearing = wind_speed;

	} else {
		wind_cross_bearing = fabsf(wind_cross_bearing);
	}

	float sin_arg = sinf(M_PI_F * 0.5f * math::constrain((airspeed - wind_cross_bearing) / airspeed_buffer_, 0.0f, 1.0f));
	return sin_arg * sin_arg;
} // bearingFeasibility

float NPFG::lateralAccelFF(const Vector2f &unit_path_tangent, const Vector2f &ground_vel,
			   const float wind_dot_upt, const float wind_cross_upt, const float airspeed,
			   const float wind_speed, const float signed_track_error, const float path_curvature) const
{
	// NOTE: all calculations within this function take place at the closet point
	// on the path, as if the aircraft were already tracking the given path at
	// this point with zero angular error. this allows us to evaluate curvature
	// induced requirements for lateral acceleration incrementation and ramp them
	// in with the track proximity and further consider the bearing feasibility
	// in excess wind conditions (this is done external to this method).

	// path frame curvature is the instantaneous curvature at our current distance
	// from the actual path (considering e.g. concentric circles emanating outward/inward)
	const float path_frame_curvature = path_curvature / math::max(1.0f - path_curvature * signed_track_error,
					   fabsf(path_curvature) * MIN_RADIUS);

	// limit tangent ground speed to along track (forward moving) direction
	const float tangent_ground_speed = math::max(ground_vel.dot(unit_path_tangent), 0.0f);

	const float path_frame_rate = path_frame_curvature * tangent_ground_speed;

	// speed ratio = projection of ground vel on track / projection of air velocity
	// on track
	const float speed_ratio = (1.0f + wind_dot_upt / math::max(projectAirspOnBearing(airspeed, wind_cross_upt), EPSILON));

	// note the use of airspeed * speed_ratio as oppose to ground_speed^2 here --
	// the prior considers that we command lateral acceleration in the air mass
	// relative frame while the latter does not
	return airspeed * speed_ratio * path_frame_rate;
} // lateralAccelFF

float NPFG::lateralAccel(const Vector2f &air_vel, const Vector2f &air_vel_ref, const float airspeed) const
{
	// lateral acceleration demand only from the heading error

	const float dot_air_vel_err = air_vel.dot(air_vel_ref);
	const float cross_air_vel_err = cross2D(air_vel, air_vel_ref);

	if (dot_air_vel_err < 0.0f) {
		// hold max lateral acceleration command above 90 deg heading error
		return p_gain_ * ((cross_air_vel_err < 0.0f) ? -airspeed : airspeed);

	} else {
		// airspeed/airspeed_ref is used to scale any incremented airspeed reference back to the current airspeed
		// for acceleration commands in a "feedback" sense (i.e. at the current vehicle airspeed)
		return p_gain_ * cross_air_vel_err / airspeed_ref_;
	}
} // lateralAccel

/*******************************************************************************
 * PX4 NAVIGATION INTERFACE FUNCTIONS (provide similar functionality to ECL_L1_Pos_Controller)
 */

void NPFG::navigateWaypoints(const Vector2d &waypoint_A, const Vector2d &waypoint_B,
			     const Vector2d &vehicle_pos, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	// similar to logic found in ECL_L1_Pos_Controller method of same name
	// BUT no arbitrary max approach angle, approach entirely determined by generated
	// bearing vectors

	path_type_loiter_ = false;

	Vector2f vector_A_to_B = getLocalPlanarVector(waypoint_A, waypoint_B);
	Vector2f vector_A_to_vehicle = getLocalPlanarVector(waypoint_A, vehicle_pos);

	if (vector_A_to_B.norm() < EPSILON) {
		// the waypoints are on top of each other and should be considered as a
		// single waypoint, fly directly to it
		unit_path_tangent_ = -vector_A_to_vehicle.normalized();
		signed_track_error_ = 0.0f;
		evaluate(ground_vel, wind_vel, unit_path_tangent_, signed_track_error_, 0.0f);

	} else if (vector_A_to_B.dot(vector_A_to_vehicle) < 0.0f) {
		// we are in front of waypoint A, fly directly to it until the bearing generated
		// to the line segement between A and B is shallower than that from the
		// bearing to the first waypoint (A).
		unit_path_tangent_ = vector_A_to_B.normalized();
		signed_track_error_ = cross2D(unit_path_tangent_, vector_A_to_vehicle);
		evaluate(ground_vel, wind_vel, unit_path_tangent_, signed_track_error_, 0.0f, true, -vector_A_to_vehicle.normalized());

	} else {
		// track the line segment between A and B
		unit_path_tangent_ = vector_A_to_B.normalized();
		signed_track_error_ = cross2D(unit_path_tangent_, vector_A_to_vehicle);
		evaluate(ground_vel, wind_vel, unit_path_tangent_, signed_track_error_, 0.0f);
	}

	updateRollSetpoint();
} // navigateWaypoints

void NPFG::navigateLoiter(const Vector2d &loiter_center, const Vector2d &vehicle_pos,
			  float radius, int8_t loiter_direction, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	path_type_loiter_ = true;

	radius = math::max(radius, MIN_RADIUS);

	Vector2f vector_center_to_vehicle = getLocalPlanarVector(loiter_center, vehicle_pos);
	const float dist_to_center = vector_center_to_vehicle.norm();

	// find the direction from the circle center to the closest point on its perimeter
	// from the vehicle position
	Vector2f unit_vec_center_to_closest_pt;

	if (dist_to_center < 0.1f) {
		// the logic breaks down at the circle center, employ some mitigation strategies
		// until we exit this region
		if (ground_vel.norm() < 0.1f) {
			// arbitrarily set the point in the northern top of the circle
			unit_vec_center_to_closest_pt = Vector2f{1.0f, 0.0f};

		} else {
			// set the point in the direction we are moving
			unit_vec_center_to_closest_pt = ground_vel.normalized();
		}

	} else {
		// set the point in the direction of the aircraft
		unit_vec_center_to_closest_pt = vector_center_to_vehicle.normalized();
	}

	// 90 deg clockwise rotation * loiter direction
	unit_path_tangent_ = float(loiter_direction) * Vector2f{-unit_vec_center_to_closest_pt(1), unit_vec_center_to_closest_pt(0)};

	// positive in direction of path normal
	signed_track_error_ = -loiter_direction * (dist_to_center - radius);

	float path_curvature = float(loiter_direction) / radius;

	evaluate(ground_vel, wind_vel, unit_path_tangent_, signed_track_error_, path_curvature);

	updateRollSetpoint();
} // navigateLoiter


/*
 * Function to calculate a trochoid segment based on the minmum parameter representation
 */
/*
 * TODO: Currently editing:
 * - Use const where possible
 * - Adjust the use of .norm() in entire function
 * - Get solution for veh_pos Vector2d and waypoint as Vector2f
 *
 * Adjustements from the version in testbed made:
 * - Changed to cosf and sinf
 * - Replaced Eigen with Matrix
 * - Used <= instead of == to compare float
 *
 * */
void NPFG::navigateTrochoid(const float x0, const float y0, const float h0, const float v, const float w,
                                const float omega, const float dt, const float T,
                                const Vector2f &veh_pos, const Vector2f &ground_vel,
                                const Vector2f &wind_vel)
{
    // General parameters --------------------------------------------------------------------------------------------
    bool wp_switching = false;


    // Startup -------------------------------------------------------------------------------------------------------
    // Detect if new segment is started and do initial calculations
    if (wp_dt_ <= 0.0f && abs(last_x0_ - x0) >= 1.0f){
        wp_dt_ = T / ceil(T / dt);  // calculate adjusted inter-sampling distance
        wp_curr_ = 0;               // reset wp counter
        segment_complete_ = false;  // reset segment complete flag
        // Calculate initial three waypoints from segment start
        wp1_ = Vector2f{troch_x(x0, y0, h0, v, w, omega, wp_curr_*wp_dt_),
                        troch_y(x0, y0, h0, v, w, omega, wp_curr_*wp_dt_)};
        wp2_ = Vector2f{troch_x(x0, y0, h0, v, w, omega, (wp_curr_+1)*wp_dt_),
                        troch_y(x0, y0, h0, v, w, omega, (wp_curr_+1)*wp_dt_)};
        wp3_ = Vector2f{troch_x(x0, y0, h0, v, w, omega, (wp_curr_+2)*wp_dt_),
                        troch_y(x0, y0, h0, v, w, omega, (wp_curr_+2)*wp_dt_)};
    }
	last_x0_ = x0;


    // Waypoint switching ---------------------------------------------------------------------------------------------
    // Calculate all necessary vectors to built the bisector two switch to next sector
    Vector2f q_curr_temp = wp2_ - wp1_;
    Vector2f q_curr = q_curr_temp / q_curr_temp.norm();	 // current segment vector
    // Not working due to error: error: ‘class matrix::Matrix<float, 2, 1>’ has no member named ‘norm’
    // Vector2f q_curr = (wp2_ - wp1_) / (wp2_ - wp1_).norm();     // current segment vector

    Vector2f q_next_temp = wp3_ - wp2_;
    Vector2f q_next = q_next_temp / q_next_temp.norm();      // next segment vector
    // Vector2f q_next = (wp3_ - wp2_) / (wp3_ - wp2_).norm();      // next segment vector

    Vector2f q_bis_next_temp1 = q_curr + q_next;
    Vector2f q_bis_next_temp2 = q_curr - q_next;
    Vector2f q_bis_next = q_bis_next_temp1 / q_bis_next_temp2.norm(); // next bisector vector
    // Vector2f q_bis_next = (q_curr + q_next) / (q_curr - q_next).norm(); // next bisector vector

    // Check if vehicle is on or beyond bisector plane H [pow_uav in H(wp2_, q_bisect)]
    // Exception: For last segment check if vehicle is beyond segment normal
	// PX4_INFO_RAW("Current waypoint CNT: %i \n", wp_curr_);
	// PX4_INFO_RAW("Current waypoint POS: %f %f \n", (double)wp2_(0), (double)wp2_(1));
    // last segment (to adjust if segments should be switched earlier)
    if ((wp_curr_+1)*wp_dt_ >= T){
	// Vector2f wp2_vec_temp = veh_pos - wp2_;
        if (((getSubtracVector(veh_pos, wp2_))).dot(q_curr) > 0){
            wp_curr_ += 1;
            wp_switching = true;
        }
    }
    // all other segments
    else{
        if ((getSubtracVector(veh_pos, wp2_)).dot(q_bis_next) > 0){
            wp_curr_ += 1;
            wp_switching = true;
        }
    }

    // Check if final segment point is reached
    if (wp_curr_ * wp_dt_ >= T){
        wp_dt_ = 0.0f;   	// reset inter-sampling distance to zero to reset segment function
        segment_complete_ = true;     // set segment complete flag true (used in mission_block to detect segment complete)
    }

    // Redefine new waypoints if waypoint switching occurred (shift back and calculate new waypoint)
    if (wp_switching == true){
        wp0_ = wp1_;
        wp1_ = wp2_;
        wp2_ = wp3_;
        wp3_ = Vector2f{troch_x(x0, y0, h0, v, w, omega, (wp_curr_+2)*wp_dt_),
                        troch_y(x0, y0, h0, v, w, omega, (wp_curr_+2)*wp_dt_)};
    }

    // Path following ------------------------------------------------------------------------------------------------

    // determine vectors for segment sector

    // last segment vector (for first segment take path normal)
    Vector2f q_last;    // past segment

    // build past segment
    // first segment
    if (wp_curr_ == 0){
        q_last = Vector2f{q_curr(1), -q_curr(0)};
    }
    // all other segments
    else{
        Vector2f q_last_int = (wp1_ - wp0_) / (getSubtracVector(wp1_, wp0_)).norm();
        q_last = (q_curr + q_last_int) / (getAddVector(q_last_int, q_curr)).norm();
    }

    // Reevaluate vectors after waypoint switching
    q_curr = (wp2_ - wp1_) / (getSubtracVector(wp2_, wp1_)).norm();      // current segment vector
    q_next = (wp3_ - wp2_) / (getSubtracVector(wp3_, wp2_)).norm();      // next segment vector

    Vector2f q_segment_first = (getSubtracVector(q_curr, q_last)).normalized();  // first bisector spanning sector
    Vector2f q_segment_second = (getSubtracVector(q_next, q_curr)).normalized(); // second bisector spanning sector

    // Version with use of Hyperplane of Eigen library (not usable on PX4)
    /*
    const Hyperplane<float,2> sector_line1 = Hyperplane<float,2>::Through(wp1_, wp1_+q_segment_first);   // first line of sector
    const Hyperplane<float,2> sector_line2 = Hyperplane<float,2>::Through(wp2_, wp2_+q_segment_second);   // second line of sector

    Vector2f sector_origin = sector_line1.intersection(sector_line2);   // origin point of the sector
    */

    // determine segment sector
   // Parametrization of first sector line
   float a1 = (getAddVector(wp1_, q_segment_first))(1) - wp1_(1);
   float b1 = wp1_(0) - (getAddVector(wp1_, q_segment_first))(0);
   float c1 = a1*wp1_(0) + b1*wp1_(1);

   // Parametrization of second sector line
   float a2 = (getAddVector(wp2_, q_segment_second))(1) - wp2_(1);
   float b2 = wp2_(0) - (getAddVector(wp2_, q_segment_second))(0);
   float c2 = a2*wp2_(0) + b2*wp2_(1);

   float det =  a1*b2 - a2*b1;  // determinant
   Vector2f sector_origin((b2*c1 - b1*c2)/det, (a1*c2 - a2*c1)/det);   // origin point of the sector

   // Version with use of Hyperplane of Eigen library (not usable on PX4)
    /*
   const Hyperplane<float,2> origin_vehicle = Hyperplane<float,2>::Through(sector_origin, sector_origin+(veh_pos-sector_origin));  // from sector origin to vehicle
   const Hyperplane<float,2> segment_line = Hyperplane<float,2>::Through(wp1_, wp2_); // line of current segment

   Vector2f segment_intersection = origin_vehicle.intersection(segment_line);   // closest point of segment
    */

    // Parametrization of line from sector origin to vehicle position
    float a3 = (getAddVector(sector_origin, (veh_pos-sector_origin)))(1) - sector_origin(1);
    float b3 = sector_origin(0) - (getAddVector(sector_origin, (veh_pos-sector_origin)))(0);
    float c3 = a3*sector_origin(0) + b3*sector_origin(1);

    // Parametrization of line of current segment
    float a4 = wp2_(1) - wp1_(1);
    float b4 = wp1_(0) - wp2_(0);
    float c4 = a4*wp1_(0) + b4*wp1_(1);

    float det2 =  a3*b4 - a4*b3;  // determinant
    Vector2f segment_intersection((b4*c3 - b3*c4)/det2, (a3*c4 - a4*c3)/det2);   // intersection point on the segment

   float segment_length = (getSubtracVector(wp2_, wp1_)).norm();    // length of current segment
   float segment_distance = (getSubtracVector(segment_intersection, wp1_)).norm();  // segment distance to intersection point

   float sigma = segment_distance / segment_length;    // calculate sigma to approx current time
   float t_sigma = sigma * wp_dt_;     // approximate segment time
   float t_curr = wp_curr_ * wp_dt_ + t_sigma;   // current time approximation for control input calculation

   // Compute control inputs ------------------------------------------------------------------------------------

   // If wp_dt is zero at this time do not calculate new control inputs, freeze last ones
   if (wp_dt_ > 0.0f){

   // Closest point on path
   Vector2f closest_point_on_path(troch_x(x0, y0, h0, v, w, omega, t_curr),
                                  troch_y(x0, y0, h0, v, w, omega, t_curr));

   // Tangent at closest point
   Vector2f path_tangent(troch_dx(x0, y0, h0, v, w, omega, t_curr),
                            troch_dy(x0, y0, h0, v, w, omega, t_curr));

   // Control output tangent
   unit_path_tangent_ = path_tangent.normalized();

   // Control output curvature (curvature at closest point)
   path_curvature_ = troch_curv(x0, y0, h0, v, w, omega, t_curr);

   // Build path normal to determine sign track error
   Vector2f path_normal((path_curvature_>0 ? -1 : 1) * path_tangent(1),
                        -(path_curvature_>0 ? -1 : 1) * path_tangent(0));

   // Vehicle distance to closest point
   float track_error = (getSubtracVector(veh_pos, closest_point_on_path)).norm();

   // Sign the track error [inside curvature + / outside curvature -]
   signed_track_error_ = ((getSubtracVector(veh_pos, closest_point_on_path)).dot(path_normal) > 0 ? 1.0f : -1.0f) * track_error;

   // Debug orientation error
   if (path_curvature_ < 0){
	   signed_track_error_ = - signed_track_error_;
   }
   // Handover to controller -------------------------------------------------------------------------------------
   evaluate(ground_vel, wind_vel, unit_path_tangent_, signed_track_error_, path_curvature_);
   updateRollSetpoint();

   }

   // Debug out
   /*
   PX4_INFO_RAW("---------------------------------------");
   PX4_INFO_RAW("Tangent (x/y): %f %f \n", (double)unit_path_tangent_(0), (double)unit_path_tangent_(1));
   PX4_INFO_RAW("Curvature: %f \n", (double)path_curvature_);
   PX4_INFO_RAW("Signed track error: %f \n", (double)signed_track_error_);
   PX4_INFO_RAW("---------------------------------------");
   */
   else{


   // Handover to controller -------------------------------------------------------------------------------------
   evaluate(ground_vel, wind_vel, unit_path_tangent_, -signed_track_error_, -path_curvature_);
   updateRollSetpoint();
   }

} // navigateTrochoid()

// Functions to calculate trochoid paths
// Calculate x-position
float NPFG::troch_x(float x0, float y0, float h0, float v, float w, float d1omega, float t){
   return (v/d1omega)*(sinf(d1omega*t+h0) - sinf(h0)) + w*t + x0;
}

// Calculate y-position
float NPFG::troch_y(float x0, float y0, float h0, float v, float w, float d1omega, float t){
   return -(v/d1omega)*(cosf(d1omega*t + h0) - cosf(h0)) + y0;
}

// Calculate dx (tangent)
float NPFG::troch_dx(float x0, float y0, float h0, float v, float w, float d1omega, float t){
   return v*cosf(d1omega*t+h0) + w;
}

// Calculate dy (tangent)
float NPFG::troch_dy(float x0, float y0, float h0, float v, float w, float d1omega, float t){
   return v*sinf(d1omega*t+h0);
}

// Calculate curvature
float NPFG::troch_curv(float x0, float y0, float h0, float v, float w, float d1omega, float t){
   return (v*d1omega*(v + w*cosf(d1omega*t+h0)))/(powf((powf(v, 2)+2*v*w*cosf(d1omega*t+h0)+powf(w, 2)),1.5));
}

/*
 * Function to calculate a clohtoid segment based on the minmum parameter representation
 */
/*
 * TODO: Currently editing:
 - xxx
 *
 * */
void NPFG::navigateClothoid(const float x0, const float y0, const float h0, const float v, const float w,
					const float ubar_alpha, const float t1, const float dt, const float T, const float dt_int,
					const matrix::Vector2f &veh_pos, const matrix::Vector2f &ground_vel,
					const matrix::Vector2f &wind_vel)
{
    // General parameters --------------------------------------------------------------------------------------------
    bool wp_switching = false;


    // Startup -------------------------------------------------------------------------------------------------------
    // Detect if new segment is started and do initial calculations
    if (wp_dt_ <= 0.0f && abs(last_x0_ - x0) >= 1.0f){
        wp_dt_ = T / ceil(T / dt);  // calculate adjusted inter-sampling distance
        wp_curr_ = 0;               // reset wp counter
        segment_complete_ = false;  // reset segment complete flag
        // Calculate initial three waypoints from segment start
        wp1_ = Vector2f{x0,
                        y0};
        wp2_ = Vector2f{cloth_x(h0, v, w, ubar_alpha, wp1_(0), 0, 1*wp_dt_, dt_int, T, t1),
                        cloth_y(h0, v, w, ubar_alpha, wp1_(1), 0, 1*wp_dt_, dt_int, T, t1)};
        wp3_ = Vector2f{cloth_x(h0, v, w, ubar_alpha, wp2_(0), 1*wp_dt_, 2*wp_dt_, dt_int, T, t1),
                        cloth_y(h0, v, w, ubar_alpha, wp2_(1), 1*wp_dt_, 2*wp_dt_, dt_int, T, t1)};
    }
	last_x0_ = x0;


    // Waypoint switching ---------------------------------------------------------------------------------------------
    // Calculate all necessary vectors to built the bisector two switch to next sector
    Vector2f q_curr_temp = wp2_ - wp1_;
    Vector2f q_curr = q_curr_temp / q_curr_temp.norm();	 // current segment vector
    // Not working due to error: error: ‘class matrix::Matrix<float, 2, 1>’ has no member named ‘norm’
    // Vector2f q_curr = (wp2_ - wp1_) / (wp2_ - wp1_).norm();     // current segment vector

    Vector2f q_next_temp = wp3_ - wp2_;
    Vector2f q_next = q_next_temp / q_next_temp.norm();      // next segment vector
    // Vector2f q_next = (wp3_ - wp2_) / (wp3_ - wp2_).norm();      // next segment vector

    Vector2f q_bis_next_temp1 = q_curr + q_next;
    Vector2f q_bis_next_temp2 = q_curr - q_next;
    Vector2f q_bis_next = q_bis_next_temp1 / q_bis_next_temp2.norm(); // next bisector vector
    // Vector2f q_bis_next = (q_curr + q_next) / (q_curr - q_next).norm(); // next bisector vector

    // Check if vehicle is on or beyond bisector plane H [pow_uav in H(wp2_, q_bisect)]
    // Exception: For last segment check if vehicle is beyond segment normal
	// PX4_INFO_RAW("Current waypoint CNT: %i \n", wp_curr_);
	// PX4_INFO_RAW("Current waypoint POS: %f %f \n", (double)wp2_(0), (double)wp2_(1));
    // last segment (to adjust if segments should be switched earlier)
    if ((wp_curr_+1)*wp_dt_ >= T){
	// Vector2f wp2_vec_temp = veh_pos - wp2_;
        if (((getSubtracVector(veh_pos, wp2_))).dot(q_curr) > 0){
            wp_curr_ += 1;
            wp_switching = true;
        }
    }
    // all other segments
    else{
        if ((getSubtracVector(veh_pos, wp2_)).dot(q_bis_next) > 0){
            wp_curr_ += 1;
            wp_switching = true;
        }
    }

    // Check if final segment point is reached
    if (wp_curr_ * wp_dt_ >= T){
        wp_dt_ = 0.0f;   	// reset inter-sampling distance to zero to reset segment function
        segment_complete_ = true;     // set segment complete flag true (used in mission_block to detect segment complete)
    }

    // Redefine new waypoints if waypoint switching occurred (shift back and calculate new waypoint)
    if (wp_switching == true){
        wp0_ = wp1_;
        wp1_ = wp2_;
        wp2_ = wp3_;
        wp3_ = Vector2f{cloth_x(h0, v, w, ubar_alpha, wp2_(0), (wp_curr_+1)*wp_dt_, (wp_curr_+2)*wp_dt_, dt_int, T, t1),
                        cloth_y(h0, v, w, ubar_alpha, wp2_(1), (wp_curr_+1)*wp_dt_, (wp_curr_+2)*wp_dt_, dt_int, T, t1)};
    }

    // Path following ------------------------------------------------------------------------------------------------

    // determine vectors for segment sector

    // last segment vector (for first segment take path normal)
    Vector2f q_last;    // past segment

    // build past segment
    // first segment
    if (wp_curr_ == 0){
        q_last = Vector2f{q_curr(1), -q_curr(0)};
    }
    // all other segments
    else{
        Vector2f q_last_int = (wp1_ - wp0_) / (getSubtracVector(wp1_, wp0_)).norm();
        q_last = (q_curr + q_last_int) / (getAddVector(q_last_int, q_curr)).norm();
    }

    // Reevaluate vectors after waypoint switching
    q_curr = (wp2_ - wp1_) / (getSubtracVector(wp2_, wp1_)).norm();      // current segment vector
    q_next = (wp3_ - wp2_) / (getSubtracVector(wp3_, wp2_)).norm();      // next segment vector

    Vector2f q_segment_first = (getSubtracVector(q_curr, q_last)).normalized();  // first bisector spanning sector
    Vector2f q_segment_second = (getSubtracVector(q_next, q_curr)).normalized(); // second bisector spanning sector

    // Version with use of Hyperplane of Eigen library (not usable on PX4)
    /*
    const Hyperplane<float,2> sector_line1 = Hyperplane<float,2>::Through(wp1_, wp1_+q_segment_first);   // first line of sector
    const Hyperplane<float,2> sector_line2 = Hyperplane<float,2>::Through(wp2_, wp2_+q_segment_second);   // second line of sector

    Vector2f sector_origin = sector_line1.intersection(sector_line2);   // origin point of the sector
    */

    // determine segment sector
   // Parametrization of first sector line
   float a1 = (getAddVector(wp1_, q_segment_first))(1) - wp1_(1);
   float b1 = wp1_(0) - (getAddVector(wp1_, q_segment_first))(0);
   float c1 = a1*wp1_(0) + b1*wp1_(1);

   // Parametrization of second sector line
   float a2 = (getAddVector(wp2_, q_segment_second))(1) - wp2_(1);
   float b2 = wp2_(0) - (getAddVector(wp2_, q_segment_second))(0);
   float c2 = a2*wp2_(0) + b2*wp2_(1);

   float det =  a1*b2 - a2*b1;  // determinant
   Vector2f sector_origin((b2*c1 - b1*c2)/det, (a1*c2 - a2*c1)/det);   // origin point of the sector

   // Version with use of Hyperplane of Eigen library (not usable on PX4)
    /*
   const Hyperplane<float,2> origin_vehicle = Hyperplane<float,2>::Through(sector_origin, sector_origin+(veh_pos-sector_origin));  // from sector origin to vehicle
   const Hyperplane<float,2> segment_line = Hyperplane<float,2>::Through(wp1_, wp2_); // line of current segment

   Vector2f segment_intersection = origin_vehicle.intersection(segment_line);   // closest point of segment
    */

    // Parametrization of line from sector origin to vehicle position
    float a3 = (getAddVector(sector_origin, (veh_pos-sector_origin)))(1) - sector_origin(1);
    float b3 = sector_origin(0) - (getAddVector(sector_origin, (veh_pos-sector_origin)))(0);
    float c3 = a3*sector_origin(0) + b3*sector_origin(1);

    // Parametrization of line of current segment
    float a4 = wp2_(1) - wp1_(1);
    float b4 = wp1_(0) - wp2_(0);
    float c4 = a4*wp1_(0) + b4*wp1_(1);

    float det2 =  a3*b4 - a4*b3;  // determinant
    Vector2f segment_intersection((b4*c3 - b3*c4)/det2, (a3*c4 - a4*c3)/det2);   // intersection point on the segment

   float segment_length = (getSubtracVector(wp2_, wp1_)).norm();    // length of current segment
   float segment_distance = (getSubtracVector(segment_intersection, wp1_)).norm();  // segment distance to intersection point

   float sigma = segment_distance / segment_length;    // calculate sigma to approx current time
   float t_sigma = sigma * wp_dt_;     // approximate segment time
   float t_curr = wp_curr_ * wp_dt_ + t_sigma;   // current time approximation for control input calculation

   // Compute control inputs ------------------------------------------------------------------------------------

   // If wp_dt is zero at this time do not calculate new control inputs, freeze last ones
   if (wp_dt_ > 0.0f){

   // Closest point on path
   Vector2f closest_point_on_path(cloth_x(h0, v, w, ubar_alpha, wp1_(0), (wp_curr_)*wp_dt_, t_curr, dt_int, T, t1),
                                   cloth_y(h0, v, w, ubar_alpha, wp1_(1), (wp_curr_)*wp_dt_, t_curr, dt_int, T, t1));

   // Tangent at closest point
   Vector2f path_tangent(cloth_dx(h0, v, w, ubar_alpha, t_curr, T, t1),
                          cloth_dy(h0, v, w, ubar_alpha, t_curr, T, t1));

   // Control output tangent
   unit_path_tangent_ = path_tangent.normalized();

   // Control output curvature (curvature at closest point)
   path_curvature_ = cloth_curv(h0, v, w, ubar_alpha, t_curr, T, t1);

   // Build path normal to determine sign track error
   Vector2f path_normal((path_curvature_>0 ? -1 : 1) * path_tangent(1),
                        -(path_curvature_>0 ? -1 : 1) * path_tangent(0));

   // Vehicle distance to closest point
   float track_error = (getSubtracVector(veh_pos, closest_point_on_path)).norm();

   // Sign the track error [inside curvature + / outside curvature -]
   signed_track_error_ = ((getSubtracVector(veh_pos, closest_point_on_path)).dot(path_normal) > 0 ? 1.0f : -1.0f) * track_error;

   // Debug orientation error
   if (path_curvature_ < 0){
	   signed_track_error_ = - signed_track_error_;
   }
   // Handover to controller -------------------------------------------------------------------------------------
   evaluate(ground_vel, wind_vel, unit_path_tangent_, signed_track_error_, path_curvature_);
   updateRollSetpoint();

   }

   // Debug out
   /*
   PX4_INFO_RAW("---------------------------------------");
   PX4_INFO_RAW("Tangent (x/y): %f %f \n", (double)unit_path_tangent_(0), (double)unit_path_tangent_(1));
   PX4_INFO_RAW("Curvature: %f \n", (double)path_curvature_);
   PX4_INFO_RAW("Signed track error: %f \n", (double)signed_track_error_);
   PX4_INFO_RAW("---------------------------------------");
   */
   else{

   // TODO: Really necessary policy with clothoids? Better lock current state?
   // Handover to controller -------------------------------------------------------------------------------------
   evaluate(ground_vel, wind_vel, unit_path_tangent_, -signed_track_error_, -path_curvature_);
   updateRollSetpoint();
   }

} // navigateClothoid()

// Helper functions for navigateClothoid()
// Functions to calculate clothoid paths
// Calculate heading
float NPFG::cloth_psi(float h0, float ubar_alpha, float t, float tbar, float t1){
    // Max bank angle gets reached
    if (tbar >= 2*t1){
        // std::cout << "Max bank reached" << std::endl;
        if (0 <= t && t < t1) {
            return ubar_alpha * (powf(t, 2) / 2) + h0;
        }
        else if (t1 <= t && t < tbar-t1){
            return ubar_alpha*t1*t+h0-ubar_alpha*powf(t1,2)/2;
        }
        else {
            return ubar_alpha*(t*tbar-(powf(t,2)/2))+h0-ubar_alpha*0.5f*(2*powf(t1, 2)+powf(tbar, 2)-2*tbar*t1);

        }
    }
        // Max bank angle gets not reached
    else {
        // std::cout << "Max bank not reached" << std::endl;
        if (t <= tbar/2){
            return ubar_alpha*(powf(t,2)/2)+h0;
        }
        else {
            return ubar_alpha*(t*tbar-(powf(t,2)/2))+h0-ubar_alpha*(powf(tbar,2)/4);
        }
    }
}

// Calculate derivative of heading (for calculation of curvature)
float NPFG::cloth_dpsi(float ubar_alpha, float t, float tbar, float t1){
    // Max bank angle gets reached
    if (tbar >= 2*t1){
        // std::cout << "Max bank reached" << std::endl;
        if (0 <= t && t < t1)
            return ubar_alpha*t;
        else if (t1 <= t && t < tbar-t1){
            return ubar_alpha*t1;
        }
        else {
            return ubar_alpha*(tbar - t);
        }
    }
        // Max bank angle gets not reached
    else {
        // std::cout << "Max bank not reached" << std::endl;
        if (t <= tbar/2){
            return ubar_alpha*t;
        }
        else {
            return ubar_alpha*(tbar - t);
        }
    }
} // NPFGPX4N::cloth_dpsi

// Calculate curvature
float NPFG::cloth_curv(float h0, float v, float w, float ubar_alpha, float t, float tbar, float t1){
    float psi_val = cloth_psi(h0, ubar_alpha, t, tbar, t1);
    float dpsi_val = cloth_dpsi(ubar_alpha, t, tbar, t1);
    float curv_num = powf(v*sinf(psi_val), 2)*dpsi_val+v*(v*cosf(psi_val)+w)*cosf(psi_val)*dpsi_val;
    float curv_den = powf(powf(v, 2) * powf(sinf(psi_val), 2) + powf(v*cosf(psi_val)+w, 2), 1.5 );
    float curv = curv_num / curv_den;
    // TODO: Detected if curvature is zero and then set to certain threshold value
    if (abs(curv) < 0.00001f){
        return 0.0001;
    }
    return curv;
} // NPFGPX4N::cloth_curv

// Calculate x-position
float NPFG::cloth_x(float h0, float v, float w, float ubar_alpha,
                        float xs, float ts, float tf, float dt, float tbar, float t1){
    // Initialize integral with IC
    float x = xs;
    float t = 0;
    // Iterations for numerical integration
    int N = floor((tf-ts)/dt);
    // Integration loop
    for (int i=1; i < N; i++){
        // Current evaluation time
        t= ts + i*dt;
        // Numerical integration
        x += (v*cosf(cloth_psi(h0, ubar_alpha, t, tbar, t1)) + w) * dt;
    }
    return x;
} // NPFGPX4N::cloth_x

// Calculate y-position
float NPFG::cloth_y(float h0, float v, float w, float ubar_alpha,
                        float ys, float ts, float tf, float dt, float tbar, float t1){
    // Initialize integral with IC
    float y = ys;
    float t = 0;
    // Iterations for numerical integration
    int N = floor((tf-ts)/dt);
    // Integration loop
    for (int i=1; i < N; i++){
        // Current evaluation time
        t = ts + i*dt;
        // Numerical integration
        y += (v*sinf(cloth_psi(h0, ubar_alpha, t, tbar, t1))) * dt;
    }
    return y;
} // NPFGPX4N::cloth_y

// Calculate dx (tangent)
float NPFG::cloth_dx(float h0, float v, float w, float ubar_alpha, float t, float tbar, float t1) {
    return v*cosf(cloth_psi(h0, ubar_alpha, t, tbar, t1)) + w;
} // NPFGPX4N::cloth_dx

// Calculate dy (tangent)
float NPFG::cloth_dy(float h0, float v, float w, float ubar_alpha, float t, float tbar, float t1) {
    return v*sinf(cloth_psi(h0, ubar_alpha, t, tbar, t1));
} // NPFGPX4N::cloth_dy




void NPFG::navigateHeading(float heading_ref, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	path_type_loiter_ = false;

	Vector2f air_vel = ground_vel - wind_vel;
	unit_path_tangent_ = Vector2f{cosf(heading_ref), sinf(heading_ref)};
	signed_track_error_ = 0.0f;

	// use the guidance law to regular heading error - ignoring wind or inertial position
	evaluate(air_vel, Vector2f{0.0f, 0.0f}, unit_path_tangent_, signed_track_error_, 0.0f);

	updateRollSetpoint();
} // navigateHeading

void NPFG::navigateBearing(float bearing, const Vector2f &ground_vel, const Vector2f &wind_vel)
{
	path_type_loiter_ = false;

	unit_path_tangent_ = Vector2f{cosf(bearing), sinf(bearing)};

	signed_track_error_ = 0.0f;

	// no track error or path curvature to consider, just regulate ground velocity
	// to bearing vector
	evaluate(ground_vel, wind_vel, unit_path_tangent_, signed_track_error_, 0.0f);

	updateRollSetpoint();
} // navigateBearing

void NPFG::navigateLevelFlight(const float heading)
{
	path_type_loiter_ = false;

	airspeed_ref_ = airspeed_nom_;
	lateral_accel_ = 0.0f;
	feas_ = 1.0f;
	bearing_vec_ = Vector2f{cosf(heading), sinf(heading)};
	unit_path_tangent_ = bearing_vec_;
	signed_track_error_ = 0.0f;

	updateRollSetpoint();
} // navigateLevelFlight

float NPFG::switchDistance(float wp_radius) const
{
	return math::min(wp_radius, track_error_bound_ * switch_distance_multiplier_);
} // switchDistance

Vector2f NPFG::getLocalPlanarVector(const Vector2d &origin, const Vector2d &target) const
{
	/* this is an approximation for small angles, proposed by [2] */
	const double x_angle = math::radians(target(0) - origin(0));
	const double y_angle = math::radians(target(1) - origin(1));
	const double x_origin_cos = cos(math::radians(origin(0)));

	return Vector2f{
		static_cast<float>(x_angle * CONSTANTS_RADIUS_OF_EARTH),
		static_cast<float>(y_angle *x_origin_cos * CONSTANTS_RADIUS_OF_EARTH),
	};
} // getLocalPlanarVector

// Test function to cast Vector2d to Vector2f
Vector2f NPFG::castVector2dtoVector2f(const Vector2d &origin)
{
	// Static cast entries to float and return Vector2f
	return Vector2f{
		static_cast<float>(origin(0)),
		static_cast<float>(origin(1)),
	};
}

// Test function to subtract two Vector2f and explicitly return Vector2f instead of matrix
Vector2f NPFG::getSubtracVector(const Vector2f &target, const Vector2f & origin){
	return Vector2f{
		static_cast<float>(target(0) - origin(0)),
		static_cast<float>(target(1) - origin(1)),
	};
}
// Test function to subtract two Vector2f and explicitly return Vector2f instead of matrix
Vector2f NPFG::getAddVector(const Vector2f &target, const Vector2f & origin){
	return Vector2f{
		static_cast<float>(target(0) + origin(0)),
		static_cast<float>(target(1) + origin(1)),
	};
}

void NPFG::updateRollSetpoint()
{
	float roll_new = atanf(lateral_accel_ * 1.0f / CONSTANTS_ONE_G);
	roll_new = math::constrain(roll_new, -roll_lim_rad_, roll_lim_rad_);

	if (dt_ > 0.0f && roll_slew_rate_ > 0.0f) {
		// slew rate limiting active
		roll_new = math::constrain(roll_new, roll_setpoint_ - roll_slew_rate_ * dt_, roll_setpoint_ + roll_slew_rate_ * dt_);
	}

	if (PX4_ISFINITE(roll_new)) {
		roll_setpoint_ = roll_new;
	}
} // updateRollSetpoint
