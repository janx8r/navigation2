#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <Eigen/Dense>

#include <rviz_rendering/objects/shape.hpp>

#include <nav2_rviz_plugins/ndt_visual.hpp>

#include <iostream>


namespace nav2_rviz_plugins{
  NDTVisual::NDTVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ){
    scene_manager_ = scene_manager;
    frame_node_ = parent_node->createChildSceneNode();
    NDT_elipsoid_.reset(new rviz_rendering::Shape(rviz_rendering::Shape::Sphere,scene_manager_,frame_node_ ));
  }

  NDTVisual::~NDTVisual()
  {
    scene_manager_->destroySceneNode( frame_node_ );
  }

  void NDTVisual::setCell(nav2_msgs::msg::NDTCellMsg cell){
    Ogre::Vector3 position(cell.mean_x,cell.mean_y,cell.mean_z);
    //eigen values aka size of the elipsoid
    Eigen::Matrix3d cov;
    int m_itr=0;
    for(int i=0;i<3;i++){
      for(int j=0;j<3;j++){
        cov(i,j)=cell.cov_matrix[m_itr];
        m_itr++;
      }
    }
    Eigen::Matrix3d m_eigVec = Eigen::Matrix3d::Zero(3,3);
    Eigen::Matrix3d m_eigVal = Eigen::Matrix3d::Zero(3,3);
    Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    m_eigVal = es.pseudoEigenvalueMatrix();
    m_eigVec = es.pseudoEigenvectors();
    m_eigVal = m_eigVal.cwiseSqrt();

    // In m_eigVec the largest element is not always on the diagonal.
    // This leads to orientation problems. Therefore, the columns of eigenvalues and eigenvectors are reordered so that the largest values are on the diagonal.
    Eigen::Matrix3d m_eigVec_temp = Eigen::Matrix3d::Zero(3,3);
    Eigen::Matrix3d m_eigVal_temp = Eigen::Matrix3d::Zero(3,3);
    for(int s=0;s<3;s++){
      //search for the largest entry in this column
      double max_val = -1.0;
      int max_index = s;
      for(int z=0;z<3;z++){
        if(abs(m_eigVec(z,s)) > max_val){
          max_val = abs(m_eigVec(z,s));
          max_index = z;
        }
      }
      // Store the entire column at the correct position in the eigenvector matrix so that the max elements are on the diagonal
      for(int j=0;j<3;j++){
        m_eigVec_temp(j,max_index) = m_eigVec(j,s);
      }
      m_eigVal_temp(max_index,max_index) = m_eigVal(s,s);
    }
    m_eigVec = m_eigVec_temp;
    m_eigVal = m_eigVal_temp;


    Eigen::Quaternion<double> q(m_eigVec);
    Ogre::Vector3 scale(3*m_eigVal(0,0),3*m_eigVal(1,1),3*m_eigVal(2,2));
    Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());

    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);
    // Eigen::Matrix3d evecs;
    // Eigen::Vector3d evals;
    // evecs = Sol.eigenvectors().real();
    // evals = Sol.eigenvalues().real();
    // Eigen::Quaternion<double> q(evecs);
    // Ogre::Vector3 scale(30*evals(0),30*evals(1),30*evals(2));
    // Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());



    NDT_elipsoid_->setScale(scale);
    NDT_elipsoid_->setPosition(position);
    NDT_elipsoid_->setOrientation(orient);
   
  }

  void NDTVisual::setCell2D(nav2_msgs::msg::NDTCellMsg cell){
    Ogre::Vector3 position(cell.mean_x,cell.mean_y,cell.mean_z);
    //eigen values aka size of the elipsoid
    Eigen::Matrix3d cov;
    int m_itr=0;
    for(int i=0;i<2;i++){
      for(int j=0;j<2;j++){
        cov(i,j)=cell.cov_matrix[m_itr];
        m_itr++;
      }
    }
    cov(0,2) = 0.0;
    cov(1,2) = 0.0;
    cov(2,2) = 1.0;
    cov(2,0) = 0.0;
    cov(2,1) = 0.0;

    //std::cout << cov << std::endl;


    Eigen::Matrix3d m_eigVec = Eigen::Matrix3d::Zero(3,3);
    Eigen::Matrix3d m_eigVal = Eigen::Matrix3d::Zero(3,3);
    Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    m_eigVal = es.pseudoEigenvalueMatrix();
    m_eigVec = es.pseudoEigenvectors();
    m_eigVal = m_eigVal.cwiseSqrt();

    // In m_eigVec the largest element is not always on the diagonal.
    // This leads to orientation problems. Therefore, the columns of eigenvalues and eigenvectors are reordered so that the largest values are on the diagonal.
    Eigen::Matrix3d m_eigVec_temp = Eigen::Matrix3d::Zero(3,3);
    Eigen::Matrix3d m_eigVal_temp = Eigen::Matrix3d::Zero(3,3);
    for(int s=0;s<3;s++){
      //search for the largest entry in this column
      double max_val = -1.0;
      int max_index = s;
      for(int z=0;z<3;z++){
        if(abs(m_eigVec(z,s)) > max_val){
          max_val = abs(m_eigVec(z,s));
          max_index = z;
        }
      }
      // Store the entire column at the correct position in the eigenvector matrix so that the max elements are on the diagonal
      for(int j=0;j<3;j++){
        m_eigVec_temp(j,max_index) = m_eigVec(j,s);
      }
      m_eigVal_temp(max_index,max_index) = m_eigVal(s,s);
    }
    m_eigVec = m_eigVec_temp;
    m_eigVal = m_eigVal_temp;


    Eigen::Quaternion<double> q(m_eigVec);
    Ogre::Vector3 scale(3*m_eigVal(0,0),3*m_eigVal(1,1),0.01);
    Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());

    // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> Sol (cov);
    // Eigen::Matrix3d evecs;
    // Eigen::Vector3d evals;
    // evecs = Sol.eigenvectors().real();
    // evals = Sol.eigenvalues().real();
    // Eigen::Quaternion<double> q(evecs);
    // Ogre::Vector3 scale(30*evals(0),30*evals(1),30*evals(2));
    // Ogre::Quaternion orient(q.w(),q.x(),q.y(),q.z());


    NDT_elipsoid_->setScale(scale);
    NDT_elipsoid_->setPosition(position);
    NDT_elipsoid_->setOrientation(orient);
   
  }

  void NDTVisual::setFramePosition( const Ogre::Vector3& position ){
    frame_node_->setPosition( position );
  }

  void NDTVisual::setFrameOrientation( const Ogre::Quaternion& orientation ){
    frame_node_->setOrientation( orientation );
  }

  void NDTVisual::setColor( float r, float g, float b, float a ){
    NDT_elipsoid_->setColor( r, g, b, a );
  }
} 

