/**\file object_recognition_skill_server.h
 * \brief File with ObjectRecognitionSkillServer class definition
 *
 * @version 1.0
 * @author carloscosta
 */

#pragma once

#include <algorithm>
#include <memory>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <object_recognition_skill_msgs/ObjectRecognitionSkillAction.h>
#include <dynamic_robot_localization/localization/localization.h>


namespace object_recognition_skill_server {
class ObjectRecognitionSkillServer {
	public:
	typedef actionlib::SimpleActionServer<object_recognition_skill_msgs::ObjectRecognitionSkillAction> ObjectRecognitionSkillActionServer;
	typedef pcl::PointXYZRGBNormal DRLPointType;
	ObjectRecognitionSkillServer() {}
	virtual ~ObjectRecognitionSkillServer() {}
	virtual bool setupConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle, ros::NodeHandlePtr &_private_node_handle);
	virtual void start();
	void processGoal(const object_recognition_skill_msgs::ObjectRecognitionSkillGoalConstPtr &_goal);
	void publishGoalFeedback(int _percentage);
	void publishGoalSucceeded();
	void publihGoalAborted(const std::string& _message);
	bool checkIfPreemptionWasRequested();

	protected:
	ros::NodeHandlePtr node_handle_;
	ros::NodeHandlePtr private_node_handle_;
	std::shared_ptr<ObjectRecognitionSkillActionServer> actionServer_;
	std::string action_server_name_;
	std::string clustering_module_parameter_server_namespace_;
	object_recognition_skill_msgs::ObjectRecognitionSkillFeedback feedback_;
	object_recognition_skill_msgs::ObjectRecognitionSkillResult result_;
	dynamic_robot_localization::Localization<DRLPointType> object_pose_estimator_;
	int number_of_recognition_retries_;
};
}
