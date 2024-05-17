#include "nav2_ndt_map/ndt_map_2d.hpp"

NdtCell::NdtCell(){
    reset();
}

NdtCell::NdtCell(nav2_msgs::msg::NDTCellMsg cell_msg){
    has_gaussian = cell_msg.has_gaussian;
    center[0] = cell_msg.center_x;
    center[1] = cell_msg.center_y;
    mean << cell_msg.mean_x, cell_msg.mean_y;
    occupancy = cell_msg.occupancy;
    n = cell_msg.n;
    if(cell_msg.cov_matrix[0] != cell_msg.cov_matrix[0]){
        std::cout << "NAN in cells cov-matrix, resetting it..."<< std::endl;
        reset();
    }
    set_cov_matrix(cell_msg.cov_matrix);
}

void NdtCell::reset(){
    has_gaussian = false;
    center[0] = 0.0;
    center[1] = 0.0;
    mean << 0.0, 0.0;
    occupancy = 0.0;
    n = 0;
    cov_matrix << 0.0, 0.0, 0.0, 0.0;
    calc_inv_cov_matrix();
}

nav2_msgs::msg::NDTCellMsg NdtCell::to_message(){
    nav2_msgs::msg::NDTCellMsg cell_msg;
    cell_msg.has_gaussian = has_gaussian;
    cell_msg.center_x = center[0];
    cell_msg.center_y = center[1];
    cell_msg.mean_x = mean(0);
    cell_msg.mean_y = mean(1);
    cell_msg.occupancy = occupancy;
    cell_msg.n = n;
    cell_msg.cov_matrix = get_cov_matrix();
    return cell_msg;
}

void NdtCell::set_cov_matrix(const std::vector<double> cov_mat){
    this->cov_matrix << cov_mat[0],cov_mat[1],cov_mat[2],cov_mat[3];
    this->calc_inv_cov_matrix();
}

std::vector<double> NdtCell::get_cov_matrix(){
    std::vector<double> cov{this->cov_matrix(0),this->cov_matrix(1),this->cov_matrix(2),this->cov_matrix(3)};
    return cov;
}

void NdtCell::calc_inv_cov_matrix(){
    this->inv_cov_matrix = this->cov_matrix.inverse();
    this->det = 1.0 / sqrt(pow(2 * M_PI, DIM) * this->cov_matrix.determinant());
}

double NdtCell::get_probability_cell(Eigen::Vector2f position){
    if(this->has_gaussian){
        Eigen::Vector2f xm = (position - this->mean);

        if(det != det) std::cout << "det is nan " << std::endl;
        if(xm != xm) std::cout << "xm is nan: " << xm << std::endl;
        if(this->inv_cov_matrix != this->inv_cov_matrix) std::cout << "this->inv_cov_matrix is nan: " << this->inv_cov_matrix << std::endl;

        return this->det * exp(-0.5 * xm.transpose() * this->inv_cov_matrix * xm);
    }
    else
        return 0.0;
}

// aktualisiert mean und cov anhand der Punkte im point_buffer
void NdtCell::update_cell(){
    Eigen::Vector2f mean_old = this->mean;
    uint64_t n_old = this->n;

    // calc mean of new points
    Eigen::Vector2f sum_new(0.0, 0.0);
    uint64_t n_new = this->point_buffer.size();
    for (auto & point : this->point_buffer){
        sum_new(0) += point.x;
        sum_new(1) += point.y;
    }

    // avoid dividing by zero
    if(n_new == 0 || (n_old + n_new - 1) == 0 || (n_old + n_new) == 0) return;

    Eigen::Vector2f mean_new = sum_new / n_new;

    // update old mean with mean of added points
    // from eq. 2, Saarinen et al. "Normal Distributions Transform Occupancy Maps: Application to Large-Scale Online 3D Mapping"
    this->mean = (n_old * mean_old + sum_new) / (n_old + n_new);

    // update n
    this->n = n_old + n_new;
    
    // calc updated cov matrix
    // from eq. 11, Saarinen et al. "Normal Distributions Transform Occupancy Maps: Application to Large-Scale Online 3D Mapping"
    Eigen::Matrix2f S_new = Eigen::Matrix2f::Zero();

    for (auto & point : this->point_buffer){
        Eigen::Vector2f pt(point.x, point.y);
        S_new += (pt - mean_new) * (pt - mean_new).transpose(); 
    }
    Eigen::Matrix2f S_old = this->cov_matrix * (n_old - 1);
    Eigen::Matrix2f S_update = S_old  +  S_new  +  ((n_old * n_new)/(n_old + n_new)) * ((mean_old - mean_new) * (mean_old - mean_new).transpose());

    //update cov
    // from eq. 3, Saarinen et al. "Normal Distributions Transform Occupancy Maps: Application to Large-Scale Online 3D Mapping"
    
    this->cov_matrix = S_update / (n_old + n_new - 1);
    this->calc_inv_cov_matrix();

    // clear buffer
    this->point_buffer.clear();

    this->has_gaussian = true;
}





NdtMap::NdtMap(std::string frame_id, rclcpp::node_interfaces::NodeClockInterface::SharedPtr clock){
    this->frame_id = frame_id;
    this->clock = clock;
}

NdtMap::~NdtMap(){
}


void NdtMap::init(){
    double x_min = -15.0;
    double x_max = 15.0;
    double y_min = -15.0;
    double y_max = 15.0;
    double delta = 0.3;
    this->init(x_min, x_max, y_min, y_max, delta);
}

void NdtMap::init(double x_min, double x_max, double y_min, double y_max, double delta){
    this->size[0] = x_max - x_min;
    this->size[1] = y_max - y_min;

    this->origin[0] = x_min;
    this->origin[1] = y_min;

    this->cell_size[0] = delta;
    this->cell_size[1] = delta;

    this->cell_count[0] = ceil(this->size[0] / this->cell_size[0]);
    this->cell_count[1] = ceil(this->size[1] / this->cell_size[1]);

    uint64_t cells_count = this->cell_count[0] * this->cell_count[1];
    // this->cells = new NdtCell[cells_count];

    cells.clear();
    NdtCell cell;
    for (uint64_t i = 0; i < cells_count; i++) {
        //this->cells[i].has_gaussian = false;
        
        cells.push_back(cell);
    }
}

bool NdtMap::load_from_file(std::string filename){
    std::cout << "Loading " << filename << "..." << std::endl;
    {
      std::ifstream input_stream(filename, std::ofstream::binary);
      if(input_stream.is_open()){
        cereal::BinaryInputArchive iarchive(input_stream);
        iarchive(*this);
      }
      else{
        std::cout << "Failed to open " << filename << ". Loading dummy-map instead."  << std::endl;
        this->init();
        return false;
      }
    }
    std::cout << "Loading done."  << std::endl;
    return true;
}


bool NdtMap::save_to_file(std::string filename){
    std::cout << "Saving " << filename << "..." << std::endl;
    {
      std::ofstream output_stream(filename, std::ios::binary);
      if(output_stream.is_open()){
        cereal::BinaryOutputArchive oarchive(output_stream);
        oarchive(*this);
      }
      else{
        std::cout << "Failed to open " << filename << "."  << std::endl;
        return false;
      }
    }
    std::cout << "Saving done."  << std::endl;
    return true;
}


void NdtMap::from_message(const nav2_msgs::msg::NDTMapMsg&  ndt_map_msg){
    std::cout << "map from_message "<< std::endl;nav2_msgs::msg::NDTCellMsg to_message();
    this->size[1] = ndt_map_msg.y_size;
    this->cell_size[0] = ndt_map_msg.x_cell_size;
    this->cell_size[1] = ndt_map_msg.y_cell_size;
    this->cell_count[0] = ceil(ndt_map_msg.x_size / ndt_map_msg.x_cell_size);
    this->cell_count[1] = ceil(ndt_map_msg.y_size / ndt_map_msg.y_cell_size);
    this->origin[0] = ndt_map_msg.x_cen - ndt_map_msg.x_size / 2;
    this->origin[1] = ndt_map_msg.y_cen - ndt_map_msg.y_size / 2;

    cells.clear();
    for (nav2_msgs::msg::NDTCellMsg cell_msg: ndt_map_msg.cells){
        NdtCell cell(cell_msg);
        cells.push_back(cell);
    }

    std::cout << "map from_message finished "<< std::endl;nav2_msgs::msg::NDTCellMsg to_message();
            


    //this->cells = new NdtCell[cells_count];
    // for (uint64_t i = 0; i < cells_count; i++) {
    //     this->cells[i].has_gaussian = ndt_map_msg.cells[i].has_gaussian;
    //     this->cells[i].center[0] = ndt_map_msg.cells[i].center_x;
    //     this->cells[i].center[1] = ndt_map_msg.cells[i].center_y;
    //     this->cells[i].mean << ndt_map_msg.cells[i].mean_x, ndt_map_msg.cells[i].mean_y;
    //     this->cells[i].occupancy = ndt_map_msg.cells[i].occupancy;
    //     this->cells[i].n = ndt_map_msg.cells[i].n;
    //     if(ndt_map_msg.cells[i].cov_matrix[0] != ndt_map_msg.cells[i].cov_matrix[0]){
    //         std::cout << "cov is nan in cell i = " << i << ", disableing it (TODO)"<< std::endl;
    //         this->cells[i].has_gaussian = false;
    //     }
    //     this->cells[i].set_cov_matrix(ndt_map_msg.cells[i].cov_matrix);
    // }
    // std::cout << "map from_message after loop "<< std::endl;
}

void NdtMap::to_message(nav2_msgs::msg::NDTMapMsg* ndt_map_msg){
    ndt_map_msg->header.frame_id = this->frame_id;
    ndt_map_msg->header.stamp = this->stamp;
    ndt_map_msg->x_size = this->size[0];
    ndt_map_msg->y_size = this->size[1];
    ndt_map_msg->z_size = 0.0;

    ndt_map_msg->x_cen = this->origin[0] + ndt_map_msg->x_size / 2;
    ndt_map_msg->y_cen = this->origin[1] + ndt_map_msg->y_size / 2;
    ndt_map_msg->z_cen = ndt_map_msg->z_size / 2;

    ndt_map_msg->x_cell_size = this->cell_size[0];
    ndt_map_msg->y_cell_size = this->cell_size[1];
    ndt_map_msg->z_cell_size = 0.0;

    for (NdtCell cell: cells){
        ndt_map_msg->cells.push_back(cell.to_message());
    }

    // uint64_t cells_count = this->cell_count[0] * this->cell_count[1];
    // // für alle Zellen
    // for (uint64_t i = 0; i < cells_count; i++) {
    //     nav2_msgs::msg::NDTCellMsg cell;
    //     cell.has_gaussian = this->cells[i].has_gaussian;
    //     cell.center_x = this->cells[i].center[0];
    //     cell.center_y = this->cells[i].center[1];
    //     cell.mean_x = this->cells[i].mean(0);
    //     cell.mean_y = this->cells[i].mean(1);
    //     cell.occupancy = this->cells[i].occupancy;
    //     cell.n = this->cells[i].n;
    //     cell.cov_matrix = this->cells[i].get_cov_matrix();

    //     ndt_map_msg->cells.push_back(cell);
    // }
}

double NdtMap::get_probability(Eigen::Vector2f position){
    // in welche Zelle fällt dieser Punkt?
    int32_t i = (int32_t)this->get_i(position(0));
    int32_t j = (int32_t)this->get_j(position(1));
    double p = 0.0;

    // für lokale Umgebung
    for(int32_t i_ = i-LOC_NEIGH; i_ <= i+LOC_NEIGH; i_++){
        for(int32_t j_ = j-LOC_NEIGH; j_ <= j+LOC_NEIGH; j_++){
            if(this->is_valid(i_, j_)){
                // berechne Wahrscheinlichkeit des Punktes in der Zelle
                uint64_t cell_index = this->get_cell_flattend_index(i_, j_);
                p += this->cells[cell_index].get_probability_cell(position);
            }
        }
    }
    return p;
}

double NdtMap::get_probability(double x, double y){
    Eigen::Vector2f position;
    position << x, y;
    return this->get_probability(position);
}


uint64_t NdtMap::get_cell_flattend_index(uint16_t i, uint16_t j){
    return  i + \
            j * this->cell_count[0];
}

bool NdtMap::is_valid(int32_t i, int32_t j){
    return  (i >= 0) && (i < this->cell_count[0]) &&\
            (j >= 0) && (j < this->cell_count[1]);
}

void NdtMap::update(pcl::PointCloud<pcl::PointXYZ> points){
    // fill points in the buffer of the corresponding cells

    //std::cout << 'a' << std::endl;
    std::set<uint64_t> idxs;
    for (const auto& point: points.points) {

        //std::cout << 'b' << std::endl;
        int32_t i = this->get_i(point.x);
        int32_t j = this->get_j(point.y);

        //std::cout << "i=" << i << " j=" << j << std::endl;
        //std::cout << 'c' << std::endl;
        if(this->is_valid(i, j)){
            //std::cout << 'd' << std::endl;
            uint64_t idx = this->get_cell_flattend_index(i, j);
            //std::cout << idx << std::endl;
            this->cells[idx].point_buffer.push_back(point);
            //std::cout << 'f' << std::endl;
            idxs.insert(idx);
            //std::cout << 'g' << std::endl;
        }
        else{
            std::cout << "not valid: " << i << ", " << j << " point " << point.x << ", " << point.y << std::endl;
        }
    }
    
    //std::cout << 'c' << std::endl;

    // for all cells with new points
    for (auto idx : idxs){
        this->cells[idx].update_cell();
    }

    //std::cout << 'd' << std::endl;

    // update timestamp to now
    this->stamp = this->clock->get_clock()->now();
}



