#include <iostream>
#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const float CalculateNISValue(const VectorXd z_prediction, const VectorXd z_measurement, const MatrixXd covariance) {
  VectorXd difference{z_measurement - z_prediction};
  return difference.transpose() * covariance.inverse() *difference;
}

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.8;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */
  n_x_ = 5;
  n_aug_ = n_x_ +2;
  lambda_ = 3-n_aug_;

  
  P_.setIdentity();
  P_(3,3) = 0.0225;
  P_(4,4) = 0.0225;
  
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_ +1);
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for(int i=1; i < 2*n_aug_ +1;i++)
  {
    weights_(i) = 0.5/(lambda_ + n_aug_);
  }

  time_us_ = 0;
  
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  // * TODO: Complete this function! Make sure you switch between lidar and radar
  // * measurements.
  // */
  if(!is_initialized_)
  {
    VectorXd z = meas_package.raw_measurements_;
    if(use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      x_ << z(0), z(1), 0.0, 0.0, 0.0;
    }
    else if(use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      double rho = z(0);
      double phi = z(1);
      double rhodot = z(2);
      double v = sqrt((rhodot*sin(phi)*rhodot*sin(phi))+ (rhodot*cos(phi)*rhodot*cos(phi)));
      double px = z(0)*cos(phi);
      double py = z(0)*sin(phi);
      x_ << px, py, v,0, 0;
            
    }
    is_initialized_ = true;
    return;
    
       
  }

  double dt = (double)(meas_package.timestamp_ - time_us_)/1000000.0;
  
  Prediction(dt);
  
  time_us_ = meas_package.timestamp_;

  if(use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)
  {
    UpdateRadar(meas_package);
  }
  if(use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER)
  {
    UpdateLidar(meas_package);  
  }
  
}


void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */
  
  int n_sigma = 2*n_aug_ +1 ;
  MatrixXd Xsig = MatrixXd(n_aug_, n_sigma);
  VectorXd x_aug(7);
  x_aug.head(5) = x_;
  x_aug(5) = 0.0;
  x_aug(6)= 0.0;
  
  MatrixXd P_aug(n_aug_, n_aug_);
  P_aug.fill(0.0);
  P_aug.block<5,5>(0,0) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  MatrixXd A = P_aug.llt().matrixL();
  Xsig.col(0) = x_aug;
  
  for(int i=0; i< n_aug_; i++)
  {
    Xsig.col(1+i) = x_aug + (sqrt(lambda_+n_aug_)*A.col(i));
    Xsig.col(1+n_aug_+i) = x_aug - (sqrt(lambda_+n_aug_)*A.col(i));
  }
  
  
  for(int i=0; i<n_sigma; i++)
  {
    double px = Xsig(0,i);
    double py = Xsig(1,i);
    double v = Xsig(2,i);
    double yaw = Xsig(3,i);
    double yawd = Xsig(4,i);
    double n_va = Xsig(5,i);
    double n_vyawdd = Xsig(6,i);
    
    if(fabs(yawd)>0.001)
    {
      Xsig_pred_(0,i) = px + ((v/yawd)*(sin(yaw + (yawd*delta_t)) - sin(yaw))) + (0.5*delta_t*delta_t*n_va*cos(yaw));
      Xsig_pred_(1,i) = py + ((v/yawd)*(cos(yaw) - cos(yaw + (yawd*delta_t)))) + (0.5*delta_t*delta_t*n_va*sin(yaw));
    }
    else
    {
      Xsig_pred_(0,i) = px + (v*cos(yaw)*delta_t)+ (0.5*delta_t*delta_t*n_va*cos(yaw));
      Xsig_pred_(1,i) = py + (v*sin(yaw)*delta_t) + (0.5*delta_t*delta_t*n_va*sin(yaw));
    }
    
    Xsig_pred_(2,i) = v + (n_va*delta_t);
    Xsig_pred_(3,i) = yaw + (yawd*delta_t) + (0.5*delta_t*delta_t*n_vyawdd);
    Xsig_pred_(4,i) = yawd + (n_vyawdd*delta_t);
     
  }
  //Predict mean and covariance
  x_.fill(0.0);
  P_.fill(0.0);
  
  for(int i=0; i<n_sigma; i++)
  {
    x_ = x_ + (weights_(i)*Xsig_pred_.col(i));
  }
  
  for(int i=0; i<n_sigma; i++)
  {
    VectorXd xdiff = Xsig_pred_.col(i) - x_ ;
    
    while (xdiff(3)>M_PI)  xdiff(3) -= 2.*M_PI;
    while (xdiff(3)<-M_PI)  xdiff(3) += 2.*M_PI;
    
    P_ = P_ + (weights_(i)*xdiff*xdiff.transpose());
  }
  

}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  //Predict Measurement mean and covariance
  
  VectorXd z = meas_package.raw_measurements_;

  
  int n_sigma = 2*n_aug_ + 1;
  int n_z = 2;
  MatrixXd Z_sigma(n_z,n_sigma);

  //Predict Sigma Points
  for(int i=0; i<n_sigma; i++)
  {
    Z_sigma(0,i) = Xsig_pred_(0,i);
    Z_sigma(1,i) = Xsig_pred_(1,i);
     
  }
  

  //Predict mean and covariance
  VectorXd z_pred(n_z);
  MatrixXd S(n_z, n_z);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for(int i=0; i<n_sigma; i++)
  {
    z_pred = z_pred + (weights_(i)*Z_sigma.col(i));
  }

  for(int i=0; i<n_sigma; i++)
  {
    VectorXd zdiff = Z_sigma.col(i) - z_pred ;
    
    S = S + (weights_(i)*zdiff*zdiff.transpose());
  }
  MatrixXd R(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_laspx_*std_laspx_;
  R(1,1) = std_laspy_*std_laspy_;
  S += R;

  //Update State
  MatrixXd T(n_x_,n_z);
  T.fill(0.0);
  for(int i=0; i<n_sigma; i++)
  {
    VectorXd zdiff = Z_sigma.col(i) - z_pred ;
    
    VectorXd xdiff = Xsig_pred_.col(i) - x_ ;
    //while(xdiff(3)>M_PI)  xdiff(3) -= 2*M_PI;
    //while(xdiff(3)<-M_PI)  xdiff(3) += 2*M_PI;
    T = T + (weights_(i)*xdiff*zdiff.transpose());
  }
  
  MatrixXd K = T*S.inverse();
  VectorXd y = z - z_pred;
  x_ = x_+ K*y;
  P_ = P_ - (K*S*K.transpose());
  std::cout<<"LIDAR NIS: " <<CalculateNISValue(z_pred, z, S)<<std::endl;
  
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */
  //Predict Measurement mean and covariance
  VectorXd z = meas_package.raw_measurements_;

  
  int n_sigma = 2*n_aug_ + 1;
  int n_z =3;
  MatrixXd Z_sigma(n_z,n_sigma);
  
  //Predict Sigma Points
  for(int i=0; i<n_sigma; i++)
  {
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);
    double yawd = Xsig_pred_(4,i);

    double vx = cos(yaw)*v;
    double vy = sin(yaw)*v;
    double dist = sqrt((px*px)+(py*py));

    Z_sigma(0,i) = dist;
    Z_sigma(1,i) = atan2(py,px);
    Z_sigma(2,i) = (px*vx + py*vy)/dist;
    
  }
  

  //Predict mean and covariance
  VectorXd z_pred(n_z);
  MatrixXd S(n_z, n_z);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for(int i=0; i<n_sigma; i++)
  {
    z_pred = z_pred + (weights_(i)*Z_sigma.col(i));
  }
  
  for(int i=0; i<n_sigma; i++)
  {
    VectorXd zdiff = Z_sigma.col(i) - z_pred ;
    while (zdiff(1)>M_PI)  zdiff(1) -= 2*M_PI;
    while (zdiff(1)<-M_PI)  zdiff(1) += 2*M_PI;
    S = S + (weights_(i)*zdiff*zdiff.transpose());
  }
  MatrixXd R(n_z,n_z);
  R.fill(0.0);
  R(0,0) = std_radr_*std_radr_;
  R(1,1) = std_radphi_*std_radphi_;
  R(2,2) = std_radrd_*std_radrd_;
  S += R;

  //Update State
  MatrixXd T(n_x_, n_z);
  T.fill(0.0);
  for(int i=0; i<n_sigma; i++)
  {
    VectorXd zdiff = Z_sigma.col(i) - z_pred ;
    
    while (zdiff(1)>M_PI)  zdiff(1) -= 2*M_PI;
    while (zdiff(1)<-M_PI)  zdiff(1) += 2*M_PI;
    
    VectorXd xdiff = Xsig_pred_.col(i) - x_ ;
    
    while (xdiff(3)>M_PI)  xdiff(3) -= 2*M_PI;
    while (xdiff(3)<-M_PI)  xdiff(3) += 2*M_PI;
    
    T = T + (weights_(i)*xdiff*zdiff.transpose());
  }

  
  MatrixXd K = T*S.inverse();
  VectorXd y = z- z_pred;
  while(y(1)>M_PI)  y(1) -= 2*M_PI;
  while(y(1)<-M_PI)  y(1) += 2*M_PI;
  
  x_ = x_+ K*y;
  P_ = P_ - (K*S*K.transpose());
  std::cout<<"RADAR NIS: " <<CalculateNISValue(z_pred, z, S)<<std::endl;
   
}