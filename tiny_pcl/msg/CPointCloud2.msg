# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the "fields" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
sensor_msgs/PointField[] fields

CPointField[] cfields    # description of used diff encoding
bool zlib_compressed     # zlib compression enabled
uint64 bit_size		 # size of diff encoded data in bits
uint64 aligned_data_size # size of diff endoded data in bytes aligned to 64bit
uint8[] data             # Actual compressed point data


bool is_dense            # True if there are no invalid points

