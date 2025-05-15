#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath> // For sqrt, pow, fabs, and M_PI

class WaypointStateMachine {
public:
    WaypointStateMachine(ros::NodeHandle& nh)
        : nh_(nh),
          current_index_(0),
          position_tolerance_(0.01),
          orientation_tolerance_(0.02),
          can_proceed_(false), // Initialize to false
		  action_performed_(false)
    {
        // Initialize waypoints
        initializeWaypoints();  

        // Publishers and subscribers
        waypoint_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/waypoint", 10);
        scoop_pub = nh_.advertise<std_msgs::Bool>("/OpenCR/scoop", 1000);
		
		scoop_tilt_pub = nh_.advertise<std_msgs::Bool>("/OpenCR/scoop_tilt", 1000);
		beacon_pub = nh_.advertise<std_msgs::Bool>("/OpenCR/beacon_place", 1000);
		dump_geo_pub = nh_.advertise<std_msgs::Bool>("/OpenCR/dump_geo", 1000);
		dump_neo_pub = nh_.advertise<std_msgs::Bool>("/OpenCR/dump_neo", 1000);
		grab_tilt_pub = nh_.advertise<std_msgs::Bool>("/OpenCR/tilt_grab_up", 1000);
		 
        robot_pose_sub_ = nh_.subscribe("/robot_pose", 10, &WaypointStateMachine::robotPoseCallback, this);
        proceed_signal_sub_ = nh_.subscribe("/OpenCR/scoop_done", 10, &WaypointStateMachine::proceedSignalCallback, this);
		action_done_ = nh_.subscribe("/OpenCR/action_done", 10, &WaypointStateMachine::proceedActionDone, this);
		
		beaconDown = false;
		bucket_up = false;
        ROS_INFO("Starting waypoint state machine.");
        publishCurrentWaypoint();
    }

    struct Waypoint {
    geometry_msgs::Pose2D pose;
    bool wait;
    int action; // Action identifier
	};
	enum ActionType {
    ACTION_NONE = 0,
    SCOOP_RQS,
    TILT_SCOOP_REQUEST,
    BEACON_ACTION,
	BUCKET_GRAB,
	DUMP_GEO,
	DUMP_NEO,
	};


	void publishWaypoint(const Waypoint& waypoint) {
        ROS_INFO("Publishing waypoint: x=%.2f, y=%.2f, theta=%.2f, wait=%s",
                 waypoint.pose.x, waypoint.pose.y, waypoint.pose.theta,
                 waypoint.wait ? "true" : "false");
        waypoint_pub_.publish(waypoint.pose);
    }
	void makeWaypoint(double x, double y, double theta, bool wait, int action) {
		Waypoint wp;
		wp.pose.x = x;
		wp.pose.y = y;
		wp.pose.theta = theta;
		wp.wait = wait;
		wp.action = action;
		waypoints_.push_back(wp);
	}
    void initializeWaypoints() {
		
		
		//Wait for led and go to beacon
		makeWaypoint(0.79375, 0.150, 0.0, true, ACTION_NONE); // Wait for led              
		makeWaypoint(0.79375, 0.60, 0.0, true, SCOOP_RQS);//move out of start and scoop
		makeWaypoint(0.79375, 0.5715, 1.57079632679, false, ACTION_NONE);//turn 90deg twords beacon  
		makeWaypoint(0.19, 0.6, 1.57079632679, true, TILT_SCOOP_REQUEST);//Hit beacon mast and tillt to clear
		makeWaypoint(0.32, 0.60, 1.57079632679, true, SCOOP_RQS);// backup and scoop


		//Beacon placment wiggle
		makeWaypoint(0.32, 0.60, -1.57079632679, false, ACTION_NONE);//turn 180deg to place beacon  
		makeWaypoint(0.165, 0.57, -1.57079632679, false, ACTION_NONE);//backup to placing location     
		makeWaypoint(0.165, 0.58, -1.57079632679, true, BEACON_ACTION);//Place beacon down
		makeWaypoint(0.165, 0.53, -1.57079632679, false, ACTION_NONE);//wiggle
		makeWaypoint(0.165, 0.61, -1.57079632679, false, ACTION_NONE);//wiggle
		makeWaypoint(0.165, 0.58, -1.57079632679, false, ACTION_NONE);//wiggle
		// makeWaypoint(0.18, 0.52, -1.57079632679, false, ACTION_NONE);//wiggle
		// makeWaypoint(0.18, 0.62, -1.57079632679, false, ACTION_NONE);//wiggle
		// makeWaypoint(0.19, 0.58, -1.57079632679, false, ACTION_NONE);//wiggle
		makeWaypoint(0.25, 0.58, -1.57079632679, false, BEACON_ACTION);//Lift beacon
		
		
		//slam into wall and clear next to neb bucket
		makeWaypoint(0.25, 0.58, 0.0, false, ACTION_NONE);//turn 90 
		makeWaypoint(0.16, 0.85, 0.0, false, ACTION_NONE);//slide into beacon wall
		makeWaypoint(0.1, 0.85, 0.0, false, ACTION_NONE);//hit beacon wall relocalize 
		makeWaypoint(0.36, 0.85, 0.0, false, ACTION_NONE);//move to next scoop lane near neb bucket
		makeWaypoint(0.36, 1.05, 0.0, true, TILT_SCOOP_REQUEST);//hit wall and tilt
		makeWaypoint(0.36, .88, 0.0, true, SCOOP_RQS);// backup and scoop
		
		
		//move to right of neb bucket and scoop pices
		makeWaypoint(0.77, 0.88, 0.0, false, ACTION_NONE);//move to the right and 
		makeWaypoint(0.77, 0.55, 0.0, false, ACTION_NONE);//move back for scoop movement
		makeWaypoint(0.89, 0.55, 0.0, false, ACTION_NONE);//move right to clear neb bucket
		makeWaypoint(0.89, 1.1, 0.0, true, TILT_SCOOP_REQUEST);//hit wall to the left of neb bucket and tilt
		makeWaypoint(0.89, 0.95, 0.0, true, SCOOP_RQS);// backup and scoop
		makeWaypoint(0.89, 1.15, 0.0, true, TILT_SCOOP_REQUEST);//redo wall tilt incase extra pices
		makeWaypoint(0.89, 0.95, 0.0, false, ACTION_NONE);//backup and scoop  *******
		
		
		//Relocalize and clear far right corner outside of cave
		makeWaypoint(0.89, 0.95, -1.57079632679, false, ACTION_NONE);// turn -90 to align with far wall ******************
		makeWaypoint(0.89, 1.2, -1.57079632679, true, TILT_SCOOP_REQUEST);//hit far wall 
		makeWaypoint(1.2, 1.2, -1.57079632679, true, TILT_SCOOP_REQUEST);//drive to corner and tilt 
		makeWaypoint(1.08, 1.2, -1.57079632679, true, SCOOP_RQS);//backup and scoop
		
		
		//localize in far right corner
		makeWaypoint(1.08, 1.05, -1.57079632679, false, ACTION_NONE);// move off of far wall 
		makeWaypoint(1.08, 1.05, -3.14, false, ACTION_NONE);//rotate -90 to face start
		makeWaypoint(1.08, 1.3, -3.14, false, ACTION_NONE);//hit far wall
		makeWaypoint(1.3, 1.3, -3.14, false, ACTION_NONE);//home cornner
		
		
		//move from far wall and clear geo bucket for pickup
		makeWaypoint(1.26, 1.3, -3.14, false, ACTION_NONE);//offset off of conner for crossing 
		makeWaypoint(1.26, 0.59, -3.14, false, ACTION_NONE);//get up to geo container
		makeWaypoint(1.08, 0.59, -3.14, false, ACTION_NONE);// move right from geo for clearing
		makeWaypoint(1.08, 0.25, -3.14, true, TILT_SCOOP_REQUEST);//hit close wall and tilt
		makeWaypoint(1.08, 0.39, -3.14, true, SCOOP_RQS);//backup and scoop
		
		
		// //Delay and go to cave
		// makeWaypoint(1.08, 0.65, -3.14, false, ACTION_NONE);//Backup to align with cave entrance
		// makeWaypoint(1.08, 0.65, 1.57079632679, false, ACTION_NONE);//turn 90 
		// makeWaypoint(1.4, 0.65, 1.57079632679, false, ACTION_NONE);//dip into cave
		// makeWaypoint(1.25, 0.65, 1.57079632679, false, ACTION_NONE); //move out of cave
		
		//**** testing going into cave
		makeWaypoint(1.08, 0.65, -3.14, false, ACTION_NONE);//Backup to align with cave entrance
		makeWaypoint(1.08, 0.65, -1.57079632679, false, ACTION_NONE);//turn 90
		makeWaypoint(2.25, 0.65, -1.57079632679, true, TILT_SCOOP_REQUEST);//hit far cave wall and tilt
		makeWaypoint(2.0, 0.65, -1.57079632679, true, SCOOP_RQS);//backup and scoop
		makeWaypoint(2.0, 0.65, 1.57079632679, false, ACTION_NONE);//rotate 180
		makeWaypoint(1.6, 0.65, 1.57079632679, false, ACTION_NONE);//
		makeWaypoint(1.3, 0.65, 1.57079632679, false, ACTION_NONE);//
		
		//Bucket grab and dump
		makeWaypoint(1.3, 0.4, 1.57079632679, false, BUCKET_GRAB);// lift grabber and hit right wall near geo
		makeWaypoint(1.35, 0.4, 1.57079632679, false, ACTION_NONE);//move twords bucket 
		makeWaypoint(1.35, 0.28, 1.57079632679, true, BUCKET_GRAB);// home on bucket and grab
		makeWaypoint(1.32, 0.28, 1.57079632679, true, DUMP_GEO);// dump geo 
		makeWaypoint(1.37, 0.28, 1.57079632679, false, ACTION_NONE);//move forward to help grab


		//Cross map and place geo container in landing pad 0
		makeWaypoint(1.25, 0.5, 1.57079632679, false, ACTION_NONE);//move to center 
		makeWaypoint(0.5, 0.5, 1.57079632679, false, ACTION_NONE);//cross with bucket twords beacon mast
		makeWaypoint(0.5, 0.5, -1.57079632679, false, ACTION_NONE);//turn 180 for bucket placement
		makeWaypoint(0.3, 0.5, -1.57079632679, false, ACTION_NONE);//move back twords beacon mast
		//makeWaypoint(0.5, 0.5, -1.57079632679, false, ACTION_NONE);//move forward to align bucket
		makeWaypoint(0.1, 0.5, -1.57079632679, false, ACTION_NONE);//backup to beacon wall
		makeWaypoint(0.1, 0.85, -1.57079632679, false, BUCKET_GRAB);//move to far left conner and relase bucket


		//move neb bucket to landing pad
		makeWaypoint(0.25, 0.75, -1.57079632679, false, ACTION_NONE);//move forward slightly *maybe redundant
		makeWaypoint(0.5, 0.75, -1.57079632679, false, BUCKET_GRAB);//move forward and right and drop bucket grabber
		makeWaypoint(0.95, 0.8, -1.57079632679, true, SCOOP_RQS);//move past neb bucket
		makeWaypoint(0.95, 1.1, -1.57079632679, false, ACTION_NONE);//hit far wall next to neb bucket
		makeWaypoint(0.3, 1.1, -1.57079632679, false, ACTION_NONE);//backup and push neb bucket into landing pad
		makeWaypoint(0.32, 1.1, -1.57079632679, false, ACTION_NONE);//move forward for clearance of neb bucket
		makeWaypoint(0.32, 0.7, -1.57079632679, false, BUCKET_GRAB);//move right away from far wall
		//makeWaypoint(0.32, 0.95, -1.57079632679, false, BUCKET_GRAB);// rise grabing arm
		makeWaypoint(0.1, 0.7, -1.57079632679, false, ACTION_NONE);//hit beacon mast wall 
		makeWaypoint(0.05, 1.0, -1.57079632679, false, ACTION_NONE);//hit buckets
		makeWaypoint(0.05, 1.0, -1.57079632679, true, DUMP_NEO);//dump neo
		makeWaypoint(0.05, 1.0, -1.57079632679, true, BUCKET_GRAB);// move out and drop grab arm
		
		
		//move bucke to middle
		makeWaypoint(0.2, 1.0, -1.57079632679, false, ACTION_NONE);//forward
		makeWaypoint(0.1, .55, -1.57079632679, false, ACTION_NONE);// move out and drop grab arm
		
		
		
		
		//go to cave
		// makeWaypoint(0.9, 0.95, -1.57079632679, false, ACTION_NONE);
		// makeWaypoint(0.9, 1.2, -1.57079632679, false, ACTION_NONE);
		// makeWaypoint(1.2, 1.2, -1.57079632679, false, ACTION_NONE);
		// makeWaypoint(1.2, .8, -1.57079632679, false, ACTION_NONE);
		// //makeWaypoint(1.4, .8, -1.57079632679, false, ACTION_NONE);
		// makeWaypoint(2.3, .8, -1.57079632679, true, TILT_SCOOP_REQUEST);
		// makeWaypoint(2.15, .8, -1.57079632679, true, SCOOP_RQS);
		// makeWaypoint(2.3, .8, -1.57079632679, true, TILT_SCOOP_REQUEST);
		// makeWaypoint(2.15, .8, -1.57079632679, true, SCOOP_RQS);
		
    }

    

    void publishCurrentWaypoint() {
		if (current_index_ < waypoints_.size()) {
			current_waypoint_ = waypoints_[current_index_];

			publishWaypoint(current_waypoint_);
		} else {
			ROS_INFO("All waypoints have been published and reached.");
			ros::shutdown();
		}
	}


    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
		double position_error = calculatePositionError(*msg, current_waypoint_.pose);
		double orientation_error = calculateOrientationError(*msg, current_waypoint_.pose);

		if (position_error <= position_tolerance_ && orientation_error <= orientation_tolerance_) {
			if (!action_performed_) {
				ROS_INFO("Reached waypoint %zu.", current_index_ + 1);

				// Perform the action associated with this waypoint
				performAction(current_waypoint_.action);
				action_performed_ = true; // Mark the action as performed
			}

			if (current_waypoint_.wait) {
				if (can_proceed_) {
					ROS_INFO("Proceeding to next waypoint after waiting.");
					can_proceed_ = false; // Reset for future waits
					action_performed_ = false; // Reset action flag for next waypoint
					current_index_++;
					publishCurrentWaypoint();
				} else {
					ROS_INFO("Waiting at waypoint %zu until proceed signal is received.", current_index_ + 1);
					// Robot remains at current position
				}
			} else {
				current_index_++;
				action_performed_ = false; // Reset action flag for next waypoint
				can_proceed_ = false;
				publishCurrentWaypoint();
			}
		}
}


    void proceedSignalCallback(const std_msgs::Bool::ConstPtr& msg) {
        can_proceed_ = msg->data;
        if (can_proceed_) {
            ROS_INFO("Received proceed signal.");
        }
    }
	void proceedActionDone(const std_msgs::Bool::ConstPtr& msg) {
        can_proceed_ = msg->data;
        if (can_proceed_) {
            ROS_INFO("Received proceed signal.");
        }
    }

    double calculatePositionError(const geometry_msgs::Pose2D& pose1, const geometry_msgs::Pose2D& pose2) {
        return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
    }

    double calculateOrientationError(const geometry_msgs::Pose2D& pose1, const geometry_msgs::Pose2D& pose2) {
        double error = pose1.theta - pose2.theta;
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        return fabs(error);
    }
	void performAction(int action) {
		switch (action) {
			case ACTION_NONE:
				// No action needed
				ROS_INFO("No action for this waypoint.");
				break;
			case SCOOP_RQS:
				actionScoop();
				break; 
			case TILT_SCOOP_REQUEST:
				scoopTilt();
				break;
			case BEACON_ACTION:
				beacon_toggle();
				break;
			case BUCKET_GRAB:
				grabBucketAction();
			break;
			case DUMP_GEO:
				dump(true);
			break;
			case DUMP_NEO:
				dump(false);
			break;
			default:
				ROS_WARN("Unknown action %d at waypoint %zu.", action, current_index_ + 1);
				break;
		}
	}
	void actionScoop() {
		std_msgs::Bool scoop_cmd;
		scoop_cmd.data = true;
		scoop_pub.publish(scoop_cmd);
		ROS_INFO("Scoop turned ON.");
	}

	void scoopTilt() {
		std_msgs::Bool scoop_tilt_cmd;
		scoop_tilt_cmd.data = true;
		scoop_tilt_pub.publish(scoop_tilt_cmd);
		ROS_INFO("Scoop turned OFF.");
	}

	void grabBucketAction() {
		std_msgs::Bool bucket_cmd;
		bucket_up = !bucket_up;
		bucket_cmd.data = bucket_up;
		grab_tilt_pub.publish(bucket_cmd);
	}
	void dump(bool dump_geo){
		std_msgs::Bool dump_cmd;
		dump_cmd.data = true;
		if(dump_geo){
			dump_geo_pub.publish(dump_cmd);
		}
		else{
			dump_neo_pub.publish(dump_cmd);
		}
	}
	
	void beacon_toggle(){
		std_msgs::Bool beacon_cmd;
		beaconDown = !beaconDown;
		beacon_cmd.data = beaconDown;
		beacon_pub.publish(beacon_cmd);
	}


private:
    ros::NodeHandle nh_;
    ros::Publisher waypoint_pub_;
    ros::Publisher scoop_pub;
	
	ros::Publisher scoop_tilt_pub;
	ros::Publisher beacon_pub;
	ros::Publisher dump_geo_pub;
	ros::Publisher dump_neo_pub;
	ros::Publisher grab_tilt_pub;
	
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber proceed_signal_sub_;
	ros::Subscriber action_done_;

    std::vector<Waypoint> waypoints_;
    Waypoint current_waypoint_;
    size_t current_index_;
    double position_tolerance_;
    double orientation_tolerance_;
	bool beaconDown;
	bool bucket_up;
    bool can_proceed_;
	bool action_performed_;

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_state_machine");
    ros::NodeHandle nh;
	
    WaypointStateMachine wsm(nh);

    ros::spin();
    return 0;
}
