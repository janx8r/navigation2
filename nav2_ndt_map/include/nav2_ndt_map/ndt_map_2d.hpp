
#ifndef NAV2_AMCL__MAP__NDT_MAP_2D_HPP_
#define NAV2_AMCL__MAP__NDT_MAP_2D_HPP_

#include <stdint.h>
#include <math.h>
#include <Eigen/Dense>
#include "nav2_msgs/msg/ndt_map_msg.hpp"
#include "nav2_msgs/msg/ndt_cell_msg.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>

#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp> 
#include <cereal/archives/binary.hpp>
//#include <cereal/archives/xml.hpp>

#include <fstream>


#define DIM 2
#define LOC_NEIGH 0

// Description for a single map cell.
class NdtCell {
    public:
        NdtCell(nav2_msgs::msg::NDTCellMsg cell_msg);
        NdtCell();

        nav2_msgs::msg::NDTCellMsg to_message();

        void reset();

        // Tell if the cell was only initialized and has no gaussian
        bool has_gaussian;

        // coordinates of center in m used for uninitilaised cells
        double center[DIM];

        // coordinates of mean in m      
        Eigen::Vector2f mean;

        // occupancy of the cell
        double occupancy;

        // Number of points considered in the normal distribution of the cell
        uint64_t n;

        // set the covariance matrix and calculate all variables dependent on it
        void set_cov_matrix(const std::vector<double> cov_mat);

        std::vector<double> get_cov_matrix();

        // Calculates the probability of a point in the cell
        double get_probability_cell(Eigen::Vector2f position);

        std::vector<pcl::PointXYZ> point_buffer;
        void update_cell();

        void calc_inv_cov_matrix();


    private:
        // covariance matrix        
        Eigen::Matrix2f cov_matrix;

        // inverse covariance matrix 
        Eigen::Matrix2f inv_cov_matrix;

        

        // Prefactor for calculation of probability
        double det;

        friend class cereal::access;
        // template <class Archive>
        // void serialize( Archive & ar )
        // {
        //     ar(has_gaussian, center[0], center[1], mean(0), mean(1), occupancy, n, cov_matrix(0,0),cov_matrix(0,1),cov_matrix(1,0),cov_matrix(1,1));
        //     //this->calc_inv_cov_matrix();
        // }


        template<class Archive>
        void save(Archive & output) const
        {
            output(has_gaussian);
            if(has_gaussian)
                output(center[0], center[1], mean(0), mean(1), occupancy, n, cov_matrix(0,0),cov_matrix(0,1),cov_matrix(1,0),cov_matrix(1,1));
        }

        template<class Archive>
        void load(Archive & input)
        {
            input(has_gaussian);
            if(has_gaussian){
                input(center[0], center[1], mean(0), mean(1), occupancy, n, cov_matrix(0,0),cov_matrix(0,1),cov_matrix(1,0),cov_matrix(1,1));
                calc_inv_cov_matrix();
            }
            else{
                reset();
            }   
        }
        


};


// Description for a single map cell.
class NdtMap {
    public:
        NdtMap(std::string frame_id, rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock);
        ~NdtMap();

        void init();
        void init(double x_min, double x_max, double y_min, double y_max, double delta);
        // size of the map in m
        double size[DIM];

        // origin of the map in m
        double origin[DIM];

        // cell size in m
        double cell_size[DIM];

        // Number of cells per axis
        int32_t cell_count[DIM];

        std::string frame_id;
        builtin_interfaces::msg::Time stamp;
        rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock;

        // map cells
        //NdtCell* cells;
        std::vector<NdtCell> cells;

        void from_message(const nav2_msgs::msg::NDTMapMsg& map_msg);

        void to_message(nav2_msgs::msg::NDTMapMsg* ndt_map_msg);

        double get_probability(Eigen::Vector2f position);

        double get_probability(double x, double y);

        // load new points in NDT map
        void update(pcl::PointCloud<pcl::PointXYZ> points);

        bool load_from_file(std::string filename);

        bool save_to_file(std::string filename);



    private:
        uint64_t get_cell_flattend_index(uint16_t i, uint16_t j);

        bool is_valid(int32_t i, int32_t j);


        // Convert from world coords to map coords
        // x,y,z: point in 3D space
        // return: index of the cell in which the point is located
        int32_t get_i(double x){return floor((x - this->origin[0]) / this->cell_size[0]);}
        int32_t get_j(double y){return floor((y - this->origin[1]) / this->cell_size[1]);}
        //uint16_t get_k(double z) return floor((z - this->origin[2]) / this->cell_size[2]);


        // Convert from map index to world coords
        // i,j,k: index of cell
        // return: origin of cell in m
        double get_x_origin(uint16_t i){return this->origin[0] + i * this->cell_size[0];}
        double get_y_origin(uint16_t j){return this->origin[1] + j * this->cell_size[1];}
        //double get_z_origin(uint16_t k) return this->origin[2] + k * this->cell_size[2];


        // Convert from map index to world coords
        // i,j,k: index of cell
        // return: center of cell in m
        double get_x_center(uint16_t i){return this->origin[0] + (i + 0.5) * this->cell_size[0];}
        double get_y_center(uint16_t j){return this->origin[1] + (j + 0.5) * this->cell_size[1];}
        //double get_z_center(uint16_t k) return this->origin[2] + (k + 0.5) * this->cell_size[2];


        friend class cereal::access;

        template <class Archive>
        void serialize( Archive & ar )
        {
            ar(frame_id, size[0], size[1], origin[0], origin[1], cell_size[0], cell_size[1], cell_count[0], cell_count[1], cells);
        }

        // template<class Archive>
        // void save(Archive & output) const
        // {
        //     output(frame_id, size[0], size[1], origin[0], origin[1], cell_size[0], cell_size[1], cell_count[0], cell_count[1], cells);
        //     // uint64_t cells_count = this->cell_count[0] * this->cell_count[1];
        //     // for(uint64_t i = 0; i<cells_count; i++){
        //     //     output(cells[i]);
        //     // }
        //     // for (NdtCell cell : cells){
        //     //     output(cell);
        //     // }
        // }

        // template<class Archive>
        // void load(Archive & input)
        // {
        //     input(frame_id, size[0], size[1], origin[0], origin[1], cell_size[0], cell_size[1], cell_count[0], cell_count[1], cells);
        //     // uint64_t cells_count = this->cell_count[0] * this->cell_count[1];
        //     // this->cells = new NdtCell[cells_count];

        //     // for(uint64_t i = 0; i<cells_count; i++){
        //     //     input(cells[i]);
        //     //     cells[i].calc_inv_cov_matrix();
        //     // }
        //     // for (NdtCell cell : cells){
        //     //     cell.calc_inv_cov_matrix();
        //     // }
        // }

};

#endif  // NAV2_AMCL__MAP__NDT_MAP_2D_HPP_
