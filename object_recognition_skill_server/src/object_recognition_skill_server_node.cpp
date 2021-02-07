/**\file object_recognition_skill_server_node.cpp
 * \brief Action server node for object recognition
 *
 * @version 1.0
 * @author Carlos M. Costa
 */

#include <ros/ros.h>
#include <dynamic_robot_localization/common/verbosity_levels.h>
#include <object_recognition_skill_server/object_recognition_skill_server.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "object_recognition_skill_server");

	ros::NodeHandlePtr node_handle(new ros::NodeHandle());
	ros::NodeHandlePtr private_node_handle(new ros::NodeHandle("~"));

	std::string pcl_verbosity_level;
	private_node_handle->param("pcl_verbosity_level", pcl_verbosity_level, std::string("ERROR"));
	dynamic_robot_localization::verbosity_levels::setVerbosityLevelPCL(pcl_verbosity_level);

	std::string ros_verbosity_level;
	private_node_handle->param("ros_verbosity_level", ros_verbosity_level, std::string("INFO"));
	dynamic_robot_localization::verbosity_levels::setVerbosityLevelROS(ros_verbosity_level);

	object_recognition_skill_server::ObjectRecognitionSkillServer object_recognition;
	if (object_recognition.setupConfigurationFromParameterServer(node_handle, private_node_handle)) {
		object_recognition.start();
		return 0;
	} else
		return -1;
}
