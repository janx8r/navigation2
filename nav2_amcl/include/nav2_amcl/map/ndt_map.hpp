/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/**************************************************************************
 * Desc: Normal Distribution Transform (NDT) map
 * Author: Jan Weber
 * Date: 22 Sep 2023
 **************************************************************************/

#ifndef NAV2_AMCL__MAP__NDT_MAP_HPP_
#define NAV2_AMCL__MAP__NDT_MAP_HPP_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// Description for a single map cell.
typedef struct
{
    // Tell if the cell was only initialized and has no gaussian
    bool has_gaussian;

    // coordinates of center in m used for uninitilaised cells
    double center[3];

    // coordinates of mean in m      
    double mean[3];
    
    // covariance matrix        
    double cov_matrix[9];

    // occupancy of the cell
    double occupancy;

    // Number of points considered in the normal distribution of the cell
    uint64_t n;
} ndt_cell_t;


// Description for a map
typedef struct
{
    // size of the map in m
    double size[3];

    // origin of the map in m
    double origin[3];

    // cell size in m
    double cell_size[3];

    // Number of cells per axis
    uint64_t cell_count[3];

    // map cells
    ndt_cell_t * cells; 
} ndt_map_t;


/**************************************************************************
 * Basic map functions
 **************************************************************************/

// Create a new (empty) map
ndt_map_t * ndt_map_alloc(void);

// Destroy a map
void ndt_map_free(ndt_map_t * map);




/**************************************************************************
 * Map manipulation macros
 **************************************************************************/

// Convert from map index to world coords
// i,j,k: index of cell
// return: center of cell in m
#define NDT_MAP_I_TO_X_CENTER(map, i) (map->origin[0] + (i + 0.5) * map->cell_size[0])
#define NDT_MAP_J_TO_Y_CENTER(map, j) (map->origin[1] + (j + 0.5) * map->cell_size[1])
#define NDT_MAP_K_TO_Z_CENTER(map, k) (map->origin[2] + (k + 0.5) * map->cell_size[2])

// Convert from map index to world coords
// i,j,k: index of cell
// return: origin of cell in m
#define NDT_MAP_I_TO_X_ORIGIN(map, i) (map->origin[0] + i * map->cell_size[0])
#define NDT_MAP_J_TO_Y_ORIGIN(map, j) (map->origin[1] + j * map->cell_size[1])
#define NDT_MAP_K_TO_Z_ORIGIN(map, k) (map->origin[2] + k * map->cell_size[2])

// Convert from world coords to map coords
// x,y,z: point in 3D space
// return: index of the cell in which the point is located
#define NDT_MAP_X_TO_I(map, x) (floor((x - map->origin[0]) / map->cell_size[0]))
#define NDT_MAP_Y_TO_J(map, y) (floor((y - map->origin[1]) / map->cell_size[1]))
#define NDT_MAP_Z_TO_K(map, z) (floor((z - map->origin[2]) / map->cell_size[2]))

// Test to see if the given map coords lie within the absolute map bounds.
#define NDT_MAP_VALID(map, i, j, k) (\
    (i >= 0) && (i < map->cell_count[0]) &&\
    (j >= 0) && (j < map->cell_count[1]) &&\
    (k >= 0) && (k < map->cell_count[2]))

// Compute the cell index for the given map coords.
#define NDT_MAP_INDEX(map, i, j, k) (i + \
                                     j * map->cell_count[0] + \
                                     k * map->cell_count[0] * map->cell_count[1])

#ifdef __cplusplus
}
#endif

#endif  // NAV2_AMCL__MAP__NDT_MAP_HPP_
