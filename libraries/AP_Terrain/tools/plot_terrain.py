import math, struct, os, sys
import crc16, time, struct, gzip
import matplotlib.pyplot as plt
import numpy as np

# MAVLink sends 4x4 grids
TERRAIN_GRID_MAVLINK_SIZE = 4

# a 2k grid_block on disk contains 8x7 of the mavlink grids.  Each
# grid block overlaps by one with its neighbour. This ensures that
# the altitude at any point can be calculated from a single grid
# block
TERRAIN_GRID_BLOCK_MUL_X = 7
TERRAIN_GRID_BLOCK_MUL_Y = 8

# this is the spacing between 32x28 grid blocks, in grid_spacing units =  (28x24)
TERRAIN_GRID_BLOCK_SPACING_X = ((TERRAIN_GRID_BLOCK_MUL_X-1)*TERRAIN_GRID_MAVLINK_SIZE)
TERRAIN_GRID_BLOCK_SPACING_Y = ((TERRAIN_GRID_BLOCK_MUL_Y-1)*TERRAIN_GRID_MAVLINK_SIZE)

# giving a total grid size of a disk grid_block of 32x28
TERRAIN_GRID_BLOCK_SIZE_X = (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_X)
TERRAIN_GRID_BLOCK_SIZE_Y = (TERRAIN_GRID_MAVLINK_SIZE*TERRAIN_GRID_BLOCK_MUL_Y)
IO_BLOCK_SIZE = 2048

# Return the bytes from a file
def read_dat_file(file):
    if file.endswith("DAT.gz"):
        with gzip.open(file, 'rb') as f:
            tile = f.read()
    elif file.endswith("DAT"):
        with open(file, 'rb') as f:
            tile = f.read()
    return tile


# Extract all the data from a block
def unpack_block(block):
    (bitmap, lat, lon, crc, version, spacing) = struct.unpack("<QiiHHH", block[:22])
    height_len = TERRAIN_GRID_BLOCK_SIZE_X*TERRAIN_GRID_BLOCK_SIZE_Y
    height = struct.unpack("<%uh" % height_len, block[22:1814])
    height = np.array(height).reshape((TERRAIN_GRID_BLOCK_SIZE_X, TERRAIN_GRID_BLOCK_SIZE_Y))
    (grid_idx_x, grid_idx_y, lon_degrees, lat_degrees) = struct.unpack("<HHhb", block[1814:1821])
    return bitmap, lat, lon, crc, version, spacing, height, grid_idx_x, grid_idx_y, lon_degrees, lat_degrees


# Find the largest grid index in north and east direction in order
# to determine the block dimensions for the tile.
def get_max_grid_idx(data):
    grid_idx_x_max = 0
    grid_idx_y_max = 0
    # iterate through each 2048 byte segment in the data corresponding to a block
    for i in range(int(len(data)/IO_BLOCK_SIZE)):
        start_idx = IO_BLOCK_SIZE*i
        end_idx = start_idx+IO_BLOCK_SIZE
        # extract the grid index from the block
        (*_, grid_idx_x, grid_idx_y, _, _) = unpack_block(data[start_idx:end_idx])
        # check min/max and update if necessary
        if grid_idx_x>grid_idx_x_max:
            grid_idx_x_max=grid_idx_x
        if grid_idx_y>grid_idx_y_max:
            grid_idx_y_max=grid_idx_y
    return grid_idx_x_max, grid_idx_y_max


# Plot a graph of the tile.
def display(file):
    data = read_dat_file(file)
    # Find out how many blocks are in the tile and allocate space to store the block information
    grid_idx_x_max, grid_idx_y_max = get_max_grid_idx(data)
    n_grid_blocks_x = grid_idx_x_max+1
    n_grid_blocks_y = grid_idx_y_max+1
    heightmap = np.zeros((TERRAIN_GRID_BLOCK_SIZE_X*n_grid_blocks_x, TERRAIN_GRID_BLOCK_SIZE_Y*n_grid_blocks_y))
    print(f".DAT file contains ({n_grid_blocks_x} x {n_grid_blocks_y}) grid blocks.")

    for i in range(int(len(data)/IO_BLOCK_SIZE)):
        start_idx = IO_BLOCK_SIZE*i
        end_idx = start_idx+IO_BLOCK_SIZE
        (*_, height, grid_idx_x, grid_idx_y, _, _) = unpack_block(data[slice(start_idx, end_idx)])
        if height.shape == (TERRAIN_GRID_BLOCK_SIZE_X, TERRAIN_GRID_BLOCK_SIZE_Y):
            heightmap[
                grid_idx_x*TERRAIN_GRID_BLOCK_SIZE_X : (grid_idx_x+1)*TERRAIN_GRID_BLOCK_SIZE_X, 
                grid_idx_y*TERRAIN_GRID_BLOCK_SIZE_Y : (grid_idx_y+1)*TERRAIN_GRID_BLOCK_SIZE_Y] = height
        else:
            print("invalid height data")
        
   #Display 
    size = 15
    plt.figure(figsize=(size, size*n_grid_blocks_y/n_grid_blocks_x))
    plt.imshow(heightmap, cmap='gray')
    plt.show()


#Main
if __name__=='__main__':
    from argparse import ArgumentParser
    parser = ArgumentParser(description='terrain data viewer')

    parser.add_argument("--file", type=str, default=None, help="file for a tile in format NxxSxxx.DAT or NxxSxxx.DAT.gz")
    args = parser.parse_args()

    if args.file is not None:
        print(display(args.file))
    else:
        print("Please specify a file to plot")
