#include "localizer2d.h"

#include "icp/eigen_icp_2d.h"

Localizer2D::Localizer2D()
    : _map(nullptr),
      _laser_in_world(Eigen::Isometry2f::Identity()),
      _obst_tree_ptr(nullptr) {}

/**
 * @brief Set the internal map reference and constructs the KD-Tree containing
 * obstacles coordinates for fast access.
 *
 * @param map_
 */
void Localizer2D::setMap(std::shared_ptr<Map> map_) {
  // Set the internal map pointer
  _map = map_;
  /**
   * If the map is initialized, fill the _obst_vect vector with world
   * coordinates of all cells representing obstacles.
   * Finally instantiate the KD-Tree (obst_tree_ptr) on the vector.
   */
  // TODO
  //Check if the map is initialized and inizialization of rows and cols
  if (_map && _map -> initialized()){ // Checks if the map has been inizialized
    int rows = _map -> rows(); // If so, the data (rows and cols) are here extracted
    int cols = _map -> cols();
    _obst_vect.clear(); // Empty the vector to avoid duplications

    for (size_t r = 0; r<rows; ++r){
      for (size_t c = 0; c<cols; ++c){

        //Check if it is an obstacle for every line and every col
        if ((*_map)(r,c) == CellType::Occupied){ // If it is an obstacle...
          Eigen::Vector2f wp = _map -> grid2world(cv::Point2i(r,c)); // Conversion from grid to world coords
          _obst_vect.push_back(wp); // I add those coordinates to the vector (in world coords)
        }
        else { // If it is not an obstacle, ...
          continue; // ... keep searching
        }
      }
    }
  }
  // Create KD-Tree
  // TODO
  // Creation of KD Tree from the vector _obst_vect
  _obst_tree_ptr = std::make_shared<TreeType>(_obst_vect.begin(), _obst_vect.end());
}

/**
 * @brief Set the current estimate for laser_in_world
 *
 * @param initial_pose_
 */
void Localizer2D::setInitialPose(const Eigen::Isometry2f& initial_pose_) {
  // TODO
  _laser_in_world = initial_pose_; // Setting the laser scan as the initial pose in world coords to start ICP
}

/**
 * @brief Process the input scan.
 * First creates a prediction using the current laser_in_world estimate
 *
 * @param scan_
 */
void Localizer2D::process(const ContainerType& scan_) {
  // Use initial pose to get a synthetic scan to compare with scan_
  // TODO
  ContainerType pred_scan;
  getPrediction(pred_scan); // This function generates a predicted scan from the estimated pose and the map

  /**
   * Align prediction and scan_ using ICP.
   * Set the current estimate of laser in world as initial guess (replace the
   * solver X before running ICP)
   */
  // TODO
  // ICP exection
  ICP icp(pred_scan, scan_, 10); // Creating the instance of the ICP class. 
                                // 10 is the MIN NUMBER OF REQUESTED POINTS FOR THE ICP ALIGNMENT
  icp.X() = _laser_in_world; // Setting the initial pose as initial guess for ICP
  icp.run(30); // Actual execution of the algo. 30 is the NUMBER OF ITERATIONS

  /**
   * Store the solver result (X) as the new laser_in_world estimate
   *
   */
  // TODO

  _laser_in_world = icp.X(); // Updating the estimated pose of the robot with the ICP result
  
}

/**
 * @brief Set the parameters of the laser scanner. Used to predict
 * measurements.
 * These parameters should be taken from the incoming sensor_msgs::LaserScan
 * message
 *
 * For further documentation, refer to:
 * http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html
 *
 *
 * @param range_min_
 * @param range_max_
 * @param angle_min_
 * @param angle_max_
 * @param angle_increment_
 */
void Localizer2D::setLaserParams(float range_min_, float range_max_,
                                 float angle_min_, float angle_max_,
                                 float angle_increment_) {
  _range_min = range_min_;
  _range_max = range_max_;
  _angle_min = angle_min_;
  _angle_max = angle_max_;
  _angle_increment = angle_increment_;
}

/**
 * @brief Computes the predicted scan at the current laser_in_world pose
 * estimate.
 *
 * @param dest_ Output predicted scan
 */
void Localizer2D::getPrediction(ContainerType& prediction_) {
  prediction_.clear();
  /**
   * To compute the prediction, query the KD-Tree and search for all points
   * around the current laser_in_world estimate.
   * You may use additional sensor's informations to refine the prediction.
   */
  // TODO
  float radius = 20;
  std::vector<PointType*> neighbors;

  // Extraction of x,y in the world coords
  float x_l = _laser_in_world.translation().x(); 
  float y_l = _laser_in_world.translation().y();

  // Searching all the obtacles around the current position within a radius
  // of 20 from the coordinates found
  _obst_tree_ptr->fullSearch(neighbors, Eigen::Vector2f(x_l, y_l), radius);


  // For each obstacle found, the eclidean distance and the angle wrt the c
  // current pose fo the robot are computed
  for(const auto& n:neighbors){
    float x_n = n->x();
    float y_n = n->y();
    float difference_x = x_l - x_n;
    float difference_y = y_l - y_n;
    float euc_dist = sqrt(std::pow(difference_x,2) + std::pow(difference_y,2));
    float alpha = std::atan2(difference_y, difference_x);

    // If the distance and the angle of the obstacle are within the laser params ... 
    if((_range_min <= euc_dist && euc_dist <= _range_max) && (alpha >= _angle_min && alpha <= _angle_max)){
      prediction_.push_back(*n); // It adds the obstacle in the predicted scan
    }
  }
}