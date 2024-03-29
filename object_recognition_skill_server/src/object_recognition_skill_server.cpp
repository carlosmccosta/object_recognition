/**\file object_recognition_skill_server.cpp
 * \brief File with ObjectRecognitionSkillServer class implementation
 *
 * @version 1.0
 * @author Carlos M. Costa
 */

#include <object_recognition_skill_server/object_recognition_skill_server.h>
#include <boost/bind.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


namespace object_recognition_skill_server {

bool ObjectRecognitionSkillServer::setupConfigurationFromParameterServer(ros::NodeHandlePtr &_node_handle, ros::NodeHandlePtr &_private_node_handle) {
	node_handle_ = _node_handle;
	private_node_handle_ = _private_node_handle;
	private_node_handle_->param<std::string>("action_server_name", action_server_name_, "ObjectRecognitionSkill");
	private_node_handle_->param<bool>("use_object_model_caching", use_object_model_caching_, true);
	private_node_handle_->param<int>("number_of_recognition_retries", number_of_recognition_retries_, 3);
	private_node_handle_->param<bool>("on_failure_try_perception_on_other_clusters", on_failure_try_perception_on_other_clusters_, true);
	object_pose_estimator_.setupConfigurationFromParameterServer(node_handle_, private_node_handle_, "");
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

	ROS_INFO("Starting ObjectRecognitionSkillServer goal");

	std::string operation_mode_upper_case = _goal->operationMode;
	std::transform(operation_mode_upper_case.begin(), operation_mode_upper_case.end(), operation_mode_upper_case.begin(), ::toupper);

	bool referencePointCloudRequired = object_pose_estimator_.referencePointCloudRequired();
	object_pose_estimator_.setFilteredPointcloudSaveFilename("");
	object_pose_estimator_.setFilteredPointcloudSaveFrameId("");

	if (object_pose_estimator_.ambientPointcloudIntegrationActive()) {
		object_pose_estimator_.setAmbientPointcloudIntegrationFiltersPreprocessedPointcloudSaveFilename(_goal->objectModel);
	} else if (object_pose_estimator_.referencePointCloudRequired() && object_pose_estimator_.getMapUpdateMode() == dynamic_robot_localization::Localization<DRLPointType>::MapUpdateMode::NoIntegration) {
		if (operation_mode_upper_case == "SAVE_FILTERED_POINTCLOUD") {
			object_pose_estimator_.setReferencePointCloudRequired(false);
			object_pose_estimator_.setFilteredPointcloudSaveFilename(_goal->objectModel);
			object_pose_estimator_.setFilteredPointcloudSaveFrameId(_goal->filteredPointcloudSaveFrameId);
		} else {
			if (operation_mode_upper_case != "SETUP_WITHOUT_CACHING" && use_object_model_caching_ && (cached_object_model_ == _goal->objectModel || _goal->objectModel.empty()) && object_pose_estimator_.referencePointCloudLoaded()) {
				ROS_INFO_STREAM("Using cached model [" << cached_object_model_ << "]");
			} else {
				if ((!_goal->objectModel.empty() && !object_pose_estimator_.loadReferencePointCloudFromFile(_goal->objectModel)) || !object_pose_estimator_.referencePointCloudLoaded()) {
					publishGoalAborted("Missing reference point cloud");
					return;
				} else {
					if (!_goal->objectModel.empty())
						cached_object_model_ = _goal->objectModel;
				}
			}
		}
	}

	if (operation_mode_upper_case == "SETUP" || operation_mode_upper_case == "SETUP_WITHOUT_CACHING") {
		ROS_INFO("Object model setup finished");
		publishGoalSucceeded();
		return;
	}

	bool clustering_enabled = false;
	private_node_handle_->param<std::string>("clustering_module_parameter_server_namespace", clustering_module_parameter_server_namespace_, "");
	if (!clustering_module_parameter_server_namespace_.empty()) {
		if (private_node_handle_->hasParam(clustering_module_parameter_server_namespace_)) {
			if (clustering_module_parameter_server_namespace_.back() != '/') {
				clustering_module_parameter_server_namespace_ += "/";
			}

			private_node_handle_->setParam(clustering_module_parameter_server_namespace_ + "tf_name_for_sorting_clusters", _goal->tfNameForSortingClusters);

			ROS_INFO_STREAM("Setting perception cluster index from goal to [" << _goal->clusterIndex << "]");
			private_node_handle_->setParam(clustering_module_parameter_server_namespace_ + "min_cluster_index", _goal->clusterIndex);
			private_node_handle_->setParam(clustering_module_parameter_server_namespace_ + "max_cluster_index", _goal->clusterIndex + 1);
			clustering_enabled = true;
		}
	}

	private_node_handle_->param<std::string>("principal_component_analysis_module_parameter_server_namespace", principal_component_analysis_module_parameter_server_namespace_, "");
	if (!principal_component_analysis_module_parameter_server_namespace_.empty()) {
		if (private_node_handle_->hasParam(principal_component_analysis_module_parameter_server_namespace_)) {
			if (principal_component_analysis_module_parameter_server_namespace_.back() != '/') {
				principal_component_analysis_module_parameter_server_namespace_ += "/";
			}
			if (_goal->pcaCustomXFlipAxis.x != 0.0 || _goal->pcaCustomXFlipAxis.y != 0.0 || _goal->pcaCustomXFlipAxis.z != 0.0) {
				private_node_handle_->setParam(principal_component_analysis_module_parameter_server_namespace_ + "custom_x_flip_axis/x", _goal->pcaCustomXFlipAxis.x);
				private_node_handle_->setParam(principal_component_analysis_module_parameter_server_namespace_ + "custom_x_flip_axis/y", _goal->pcaCustomXFlipAxis.y);
				private_node_handle_->setParam(principal_component_analysis_module_parameter_server_namespace_ + "custom_x_flip_axis/z", _goal->pcaCustomXFlipAxis.z);
			}
		}
	}

	dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus status = dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::FailedPoseEstimation;
	ros::Rate rate(10);

	int number_of_clusters = 1;
	int current_cluster = 0;
	bool break_processing = false;
	bool goal_cluster_processed = false;
	bool set_initial_pose_from_goal = (_goal->initialPose.orientation.x + _goal->initialPose.orientation.y + _goal->initialPose.orientation.z + _goal->initialPose.orientation.w) != 0.0;

	while (current_cluster < number_of_clusters && !break_processing) {
		bool retry_other_clusters_active = on_failure_try_perception_on_other_clusters_ && clustering_enabled && goal_cluster_processed;
		bool current_cluster_is_different_than_goal_and_needs_to_be_processed = retry_other_clusters_active && current_cluster !=_goal->clusterIndex;
		if (!goal_cluster_processed || current_cluster_is_different_than_goal_and_needs_to_be_processed) {
			if (current_cluster_is_different_than_goal_and_needs_to_be_processed) {
				ROS_INFO_STREAM("Retrying perception on cluster with index [" << current_cluster << "]");
				private_node_handle_->setParam(clustering_module_parameter_server_namespace_ + "min_cluster_index", current_cluster);
				private_node_handle_->setParam(clustering_module_parameter_server_namespace_ + "max_cluster_index", current_cluster + 1);
			}

			for (int i = 0; i < number_of_recognition_retries_ + 1; ++i) {
				if (set_initial_pose_from_goal) {
					object_pose_estimator_.setInitialPoseFromPose(_goal->initialPose);
				} else {
					object_pose_estimator_.setupInitialPoseFromParameterServer();
				}

				object_pose_estimator_.restartProcessingSensorData();

				while (true) {
					if (checkIfPreemptionWasRequested()) {
						object_pose_estimator_.stopProcessingSensorData();
						object_pose_estimator_.setReferencePointCloudRequired(referencePointCloudRequired);
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
				if (status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::SuccessfulPreprocessing || status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::SuccessfulPoseEstimation) {
					break_processing = true;
					break;
				}
			}
		}

		if (on_failure_try_perception_on_other_clusters_ && clustering_enabled && !goal_cluster_processed) {
			private_node_handle_->param<int>(clustering_module_parameter_server_namespace_ + "number_of_clusters_detected", number_of_clusters, 1);
		} else {
			++current_cluster;
		}
		goal_cluster_processed = true;
	}

	if (status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::SuccessfulPoseEstimation) {
		tf2::Transform object_pose_in_camera_frame = object_pose_estimator_.getAcceptedEstimatedPose().inverse();
		tf2::toMsg(object_pose_in_camera_frame, result_.pose.pose);
		result_.pose.header.stamp = object_pose_estimator_.getLastAcceptedPoseTime();
		result_.pose.header.frame_id = object_pose_estimator_.getBaseLinkFrameId();
		publishGoalSucceeded();
	} else if (status == dynamic_robot_localization::Localization<DRLPointType>::SensorDataProcessingStatus::SuccessfulPreprocessing) {
		publishGoalSucceeded();
	} else {
		publishGoalAborted("Pose estimation failed with error [" + dynamic_robot_localization::Localization<DRLPointType>::s_sensorDataProcessingStatusToStr(status) + "]");
	}

	object_pose_estimator_.setReferencePointCloudRequired(referencePointCloudRequired);
}


void ObjectRecognitionSkillServer::publishGoalSucceeded() {
	result_.percentage = 100;
	result_.skillStatus = action_server_name_ + ": Succeeded";
	ROS_INFO_STREAM(result_.skillStatus);
	actionServer_->setSucceeded(result_, result_.skillStatus);
}


void ObjectRecognitionSkillServer::publishGoalAborted(const std::string& _message) {
	result_.percentage = 0;
	result_.skillStatus = action_server_name_ + ": Aborted | Reason: " + _message;
	ROS_ERROR_STREAM(result_.skillStatus);
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
		ROS_WARN_STREAM(result_.skillStatus);
		actionServer_->setPreempted(result_, result_.skillStatus);
		return true;
	} else {
		return false;
	}
}

}
