/**\file object_recognition_skill_server.cpp
 * \brief File with ObjectRecognitionSkillServer class implementation
 *
 * @version 1.0
 * @author carloscosta
 */

#include <object_recognition_skill_server/object_recognition_skill_server.h>
#include <boost/bind.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace object_recognition_skill_server {

bool ObjectRecognitionSkillServer::setupConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle, ros::NodeHandlePtr &_private_node_handle) {
	node_handle_ = _node_handle;
	private_node_handle_ = _private_node_handle;
	private_node_handle_->param<std::string>("action_name", action_server_name_, "object_recognition_skill_server");
	private_node_handle_->param<std::string>("clustering_module_parameter_server_namespace", clustering_module_parameter_server_namespace_, "");
	private_node_handle_->param<int>("number_of_recognition_retries_", number_of_recognition_retries_, 3);
	object_pose_estimator_.setupConfigurationFromParameterServer(node_handle_, private_node_handle_);
	return true;
}


void ObjectRecognitionSkillServer::start() {
	actionServer_ = std::make_shared<ObjectRecognitionSkillActionServer>(*node_handle_, action_server_name_,
	                                                                     boost::bind(&ObjectRecognitionSkillServer::processGoal, this, _1), false);
	actionServer_->start();
	object_pose_estimator_.startLocalization(false);
	object_pose_estimator_.stopProcessingSensorData();
	object_pose_estimator_.startROSSpinner();
}


void ObjectRecognitionSkillServer::processGoal(const object_recognition_skill_msgs::ObjectRecognitionSkillGoalConstPtr &_goal) {
	feedback_ = object_recognition_skill_msgs::ObjectRecognitionSkillFeedback();
	result_ = object_recognition_skill_msgs::ObjectRecognitionSkillResult();

	if (!_goal->objectModel.empty() && !object_pose_estimator_.loadReferencePointCloudFromFile(_goal->objectModel) || !object_pose_estimator_.referencePointCloudLoaded()) {
		publihGoalAborted("Missing reference point cloud");
		return;
	}

	if (!clustering_module_parameter_server_namespace_.empty()) {
		private_node_handle_->setParam(clustering_module_parameter_server_namespace_ + "/min_cluster_index", _goal->clusterIndex);
		private_node_handle_->setParam(clustering_module_parameter_server_namespace_ + "/max_cluster_index", _goal->clusterIndex + 1);
	}

	dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus status = dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::FailedPoseEstimation;
	ros::Rate rate(10);

	for (int i = 0; i < number_of_recognition_retries_; ++i) {
		object_pose_estimator_.setupInitialPose();
		object_pose_estimator_.restartProcessingSensorData();

		while (true) {
			if (checkIfPreemptionWasRequested()) {
				object_pose_estimator_.stopProcessingSensorData();
				return;
			}

			ros::spinOnce();
			status = object_pose_estimator_.getSensorDataProcessingStatus();
			if (status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::WaitingForSensorData)
				rate.sleep();
			else
				break;
		}

		object_pose_estimator_.stopProcessingSensorData();
		if (status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::SuccessfulPoseEstimation)
			break;
	}

	if (status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::FailedPoseEstimation)
		publihGoalAborted("Pose estimation failed");
	else if (status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::SuccessfulPoseEstimation) {
		tf2::Transform pose_camera_frame = object_pose_estimator_.getAcceptedEstimatedPose().inverse();
		tf2::toMsg(pose_camera_frame, result_.pose);
		publishGoalSucceeded();
	}
}


void ObjectRecognitionSkillServer::publishGoalSucceeded() {
	result_.percentage = 100;
	result_.skillStatus = action_server_name_ + ": Succeeded";
	ROS_INFO_STREAM(result_.skillStatus);
	actionServer_->setSucceeded(result_, result_.skillStatus);
}


void ObjectRecognitionSkillServer::publihGoalAborted(const std::string& _message) {
	result_.percentage = 0;
	result_.skillStatus = action_server_name_ + ": Aborted | Reason: " + _message;
	ROS_INFO_STREAM(result_.skillStatus);
	actionServer_->setAborted(result_, result_.skillStatus);
}


void ObjectRecognitionSkillServer::publishGoalFeedback(int _percentage) {
	feedback_.percentage = _percentage;
	feedback_.skillStatus = action_server_name_ + " Executing (percentage: " + std::to_string(_percentage) + ")";
	ROS_INFO_STREAM(feedback_.skillStatus);
	actionServer_->publishFeedback(feedback_);
}


bool ObjectRecognitionSkillServer::checkIfPreemptionWasRequested() {
	if (actionServer_->isPreemptRequested() || !ros::ok()) {
		result_.skillStatus = action_server_name_ + ": Preempted";
		ROS_INFO_STREAM(result_.skillStatus);
		actionServer_->setPreempted(result_, result_.skillStatus);
		return true;
	} else {
		return false;
	}
}

}
