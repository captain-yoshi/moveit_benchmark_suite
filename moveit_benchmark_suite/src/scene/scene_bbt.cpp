/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Captain Yoshi
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Captain Yoshi
   Desc:
*/

#include <ros/package.h>

#include <moveit/benchmark_suite/scene/util.h>

#include <moveit/benchmark_suite/scene/scene_bbt.h>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

namespace moveit {
namespace benchmark_suite {

const std::string PACKAGE_MESH_PATH = "package://moveit_benchmark_suite_model_set/yale_cmu_berkeley";

void bbtCollisionObjectMesh(std::vector<moveit_msgs::CollisionObject>& collision_objects,
                            std::vector<moveit_msgs::ObjectColor>& object_colors) {
	{  // Clear box 1
		moveit_msgs::CollisionObject object;

		double width = 0.292;
		double depth = 0.429;
		double height = 0.149;

		const std::string object_id = "clear_box_1";
		const std::string mesh_path = PACKAGE_MESH_PATH + "/998_clear_box/collision.stl";
		const std::string frame_id = "world";

		const Eigen::Vector3d scaling = Eigen::Vector3d(0.001, 0.001, 0.001);
		Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
		Eigen::Quaterniond quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
		                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
		                          Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

		frame.translation() = Eigen::Vector3d(depth / 2.0 + 0.16, width / 2.0 + 0.003, 0.005);
		frame.linear() = quat.matrix();

		createCollisionObjectMesh(object, object_id, mesh_path, frame, frame_id, scaling);
		collision_objects.push_back(object);
		object_colors.push_back(getObjectColorTransparent(object_id));
	}
	{  // Clear box 2
		moveit_msgs::CollisionObject object;

		double width = 0.292;
		double depth = 0.429;
		double height = 0.149;

		const std::string object_id = "clear_box_2";
		const std::string mesh_path = PACKAGE_MESH_PATH + "/998_clear_box/collision.stl";
		const std::string frame_id = "world";
		const Eigen::Vector3d scaling = Eigen::Vector3d(0.001, 0.001, 0.001);
		Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
		Eigen::Quaterniond quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
		                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
		                          Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());

		frame.translation() = Eigen::Vector3d(depth / 2.0 + 0.16, -width / 2.0 - 0.003, 0.005);
		frame.linear() = quat.matrix();

		createCollisionObjectMesh(object, object_id, mesh_path, frame, frame_id, scaling);
		collision_objects.push_back(object);
		object_colors.push_back(getObjectColorTransparent(object_id));
	}
	{  // Box lid
		moveit_msgs::CollisionObject object;

		double width = 0.292;
		double depth = 0.429;
		double height = 0.02;

		const std::string object_id = "box_lid";
		const std::string mesh_path = PACKAGE_MESH_PATH + "/999_box_lid/collision.stl";
		const std::string frame_id = "world";
		const Eigen::Vector3d scaling = Eigen::Vector3d(0.001, 0.001, 0.001);
		Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
		Eigen::Quaterniond quat = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()) *
		                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
		                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX());

		frame.translation() = Eigen::Vector3d(depth / 2.0 + 0.16, -height / 4.0, width / 2.0 - 0.005);
		frame.linear() = quat.matrix();

		createCollisionObjectMesh(object, object_id, mesh_path, frame, frame_id, scaling);
		collision_objects.push_back(object);
		object_colors.push_back(getObjectColorGhostWhite(object_id));
	}
	{  // Colored wood block
		moveit_msgs::CollisionObject object;

		double width = 0.0254;  // 1 inch
		double depth = 0.0254;
		double height = 0.0254;

		const std::string object_prefix_id = "wood_block_";
		const std::string mesh_path = PACKAGE_MESH_PATH + "/070-b_colored_wood_blocks/google_16k/collision.stl";
		const std::string frame_id = "world";
		const Eigen::Vector3d scaling = Eigen::Vector3d(1, 1, 1);

		int ctr = 1;
		Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();

		for (unsigned int i = 0; i < 4; i++) {
			for (unsigned int j = 0; j < 4; j++) {
				frame.translation() =
				    Eigen::Vector3d((0.16 + 0.429 / 2.0 + 3 * width) - (i * 2 * width),
				                    (-0.292 / 2 - 0.01 / 2 - 3 * width) + (j * 2 * width), 0.01 + width / 2);
				std::string object_id = object_prefix_id + std::to_string(ctr);
				createCollisionObjectMesh(object, object_id, mesh_path, frame, frame_id, scaling);
				collision_objects.push_back(object);
				object_colors.push_back(getObjectColorBlack(object_id));
				ctr++;
			}
		}
	}
}

void bbtCollisionObjectPrimitive(std::vector<moveit_msgs::CollisionObject>& collision_objects,
                                 std::vector<moveit_msgs::ObjectColor>& object_colors) {
	{  // Clear boxes
		moveit_msgs::CollisionObject object;

		const std::vector<std::string> object_ids = { "a_bottom", "b_front", "c_back", "d_left", "e_right" };
		const std::string frame_id = "world";

		// physical dimension of 5 box primitives
		double dim_x = 0.429;
		double dim_y = 0.292;
		double dim_z = 0.149;
		double dim_narrow = 0.01;
		double x_offset = 0.16;

		EigenSTL::vector_Vector3d dimensions = { { dim_x, dim_y, dim_narrow },
			                                      { dim_narrow, dim_y, dim_z },
			                                      { dim_narrow, dim_y, dim_z },
			                                      { dim_x, dim_narrow, dim_z },
			                                      { dim_x, dim_narrow, dim_z } };

		{  // Clear box 1
			EigenSTL::vector_Isometry3d frames;
			Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();

			// bottom
			double pos_x = dim_x / 2.0 + x_offset;
			double pos_y = dim_y / 2.0 + dim_narrow / 2.0;
			double pos_z = dim_narrow / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// front
			pos_x = dim_x + x_offset - dim_narrow / 2.0;
			pos_y = dim_y / 2.0 + dim_narrow / 2.0;
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// back
			pos_x = x_offset + dim_narrow / 2.0;
			pos_y = dim_y / 2.0 + dim_narrow / 2.0;
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// left
			pos_x = dim_x / 2.0 + x_offset;
			pos_y = dim_narrow;
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// right
			pos_x = dim_x / 2.0 + x_offset;
			pos_y = dim_y + dim_narrow / 2.0 - dim_narrow / 2.0;
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			for (unsigned int i = 0; i < object_ids.size(); i++) {
				std::string object_id = "clear_box_1-" + object_ids[i];

				shape_msgs::SolidPrimitive primitive;
				primitive.type = primitive.BOX;
				primitive.dimensions.resize(3);
				primitive.dimensions[primitive.BOX_X] = dimensions[i].x();
				primitive.dimensions[primitive.BOX_Y] = dimensions[i].y();
				primitive.dimensions[primitive.BOX_Z] = dimensions[i].z();

				createCollisionObjectPrimitive(object, object_id, primitive, frames[i], frame_id);
				collision_objects.push_back(object);
			}
		}
		{  // Clear box 2
			EigenSTL::vector_Isometry3d frames;
			Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();

			// bottom
			double pos_x = dim_x / 2.0 + x_offset;
			double pos_y = -(dim_y / 2.0 + dim_narrow / 2.0);
			double pos_z = dim_narrow / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// front
			pos_x = dim_x + x_offset - dim_narrow / 2.0;
			pos_y = -(dim_y / 2.0 + dim_narrow / 2.0);
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// back
			pos_x = x_offset + dim_narrow / 2.0;
			pos_y = -(dim_y / 2.0 + dim_narrow / 2.0);
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// left
			pos_x = dim_x / 2.0 + x_offset;
			pos_y = -dim_narrow;
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			// right
			pos_x = dim_x / 2.0 + x_offset;
			pos_y = -(dim_y + dim_narrow / 2.0 - dim_narrow / 2.0);
			pos_z = dim_z / 2.0;
			frame.translation() = Eigen::Vector3d(pos_x, pos_y, pos_z);
			frames.push_back(frame);

			for (unsigned int i = 0; i < object_ids.size(); i++) {
				std::string object_id = "clear_box_2-" + object_ids[i];

				shape_msgs::SolidPrimitive primitive;
				primitive.type = primitive.BOX;
				primitive.dimensions.resize(3);
				primitive.dimensions[primitive.BOX_X] = dimensions[i].x();
				primitive.dimensions[primitive.BOX_Y] = dimensions[i].y();
				primitive.dimensions[primitive.BOX_Z] = dimensions[i].z();

				createCollisionObjectPrimitive(object, object_id, primitive, frames[i], frame_id);
				collision_objects.push_back(object);
			}
		}
	}
	{  // Box Lid
		moveit_msgs::CollisionObject object;

		const std::string object_id = "box_lid";
		const std::string frame_id = "world";

		// physical dimension of 5 box primitives
		double dim_x = 0.429;
		double dim_y = 0.02;
		double dim_z = 0.292;
		double dim_narrow = 0.01;
		double x_offset = 0.16;

		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = dim_x;
		primitive.dimensions[primitive.BOX_Y] = dim_narrow;
		primitive.dimensions[primitive.BOX_Z] = dim_z;

		Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();
		frame.translation() = Eigen::Vector3d(x_offset + dim_x / 2.0, 0.0, dim_z / 2.0);

		createCollisionObjectPrimitive(object, object_id, primitive, frame, frame_id);
		collision_objects.push_back(object);
	}
	{  // Cubes
		moveit_msgs::CollisionObject object;

		const std::string object_prefix_id = "cube_";
		const std::string frame_id = "world";

		// physical dimension of 5 box primitives
		double dim_x = 0.0254;

		shape_msgs::SolidPrimitive primitive;
		primitive.type = primitive.BOX;
		primitive.dimensions.resize(3);
		primitive.dimensions[primitive.BOX_X] = dim_x;
		primitive.dimensions[primitive.BOX_Y] = dim_x;
		primitive.dimensions[primitive.BOX_Z] = dim_x;

		int ctr = 1;
		Eigen::Isometry3d frame = Eigen::Isometry3d::Identity();

		for (unsigned int i = 0; i < 4; i++) {
			for (unsigned int j = 0; j < 4; j++) {
				frame.translation() =
				    Eigen::Vector3d((0.16 + 0.429 / 2.0 + 3 * dim_x) - (i * 2 * dim_x),
				                    (-0.292 / 2 - 0.01 / 2 - 3 * dim_x) + (j * 2 * dim_x), 0.01 + dim_x / 2);
				std::string object_id = object_prefix_id + std::to_string(ctr);
				createCollisionObjectPrimitive(object, object_id, primitive, frame, frame_id);
				collision_objects.push_back(object);
				object_colors.push_back(getObjectColorBlack(object_id));
				ctr++;
			}
		}
	}

}  // namespace benchmark_suite
void bbtRobotStatePreGrasp(moveit::core::RobotState& robot_state) {
	std::map<const std::string, double> joint_position_map;

	joint_position_map.insert(std::make_pair(PANDA_JOINT1, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_JOINT2, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_JOINT3, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_JOINT4, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_JOINT5, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_JOINT6, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_JOINT7, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_FINGER_JOINT1, 1.0));
	joint_position_map.insert(std::make_pair(PANDA_FINGER_JOINT2, 1.0));

	for (auto const& joint_position : joint_position_map) {
		robot_state.setJointPositions(joint_position.first, &joint_position.second);
	}
}

}  // namespace benchmark_suite
}  // namespace moveit
