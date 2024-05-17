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

#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "nav2_amcl/map/ndt_map.hpp"


// Create a new map
ndt_map_t * ndt_map_alloc(void)
{
  ndt_map_t * map;

  map = (ndt_map_t *) malloc(sizeof(ndt_map_t));

  // Assume we start at (0, 0)
  map->origin[0] = 0.0;
  map->origin[1] = 0.0;
  map->origin[2] = 0.0;

  // Make the size odd
  map->size[0] = 0.0;
  map->size[1] = 0.0;
  map->size[2] = 0.0;

  map->cell_size[0] = 0.0;
  map->cell_size[1] = 0.0;
  map->cell_size[2] = 0.0;

  map->cell_count[0] = 0;
  map->cell_count[1] = 0;
  map->cell_count[2] = 0;


  // Allocate storage for main map
  map->cells = (ndt_cell_t *) NULL;

  return map;
}


// Destroy a map
void ndt_map_free(ndt_map_t * map)
{
  free(map->cells);
  free(map);
}
