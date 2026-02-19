# Chunk Payload Format

This document defines the binary payload layout for point cloud chunks sent by processor_node.

## TCP Packet Layout

The TCP stream sends a header followed by a payload.

Header (network byte order, big-endian):

- uint32 magic ("LDAR" = 0x4C444152)
- uint32 payload_size (bytes of the payload only)
- uint32 lidar_id
- uint32 sequence

Payload (native little-endian floats on current platforms):

- uint32 cloud_count
- repeat cloud_count times:
  - uint32 point_count
  - repeat point_count times:
    - float32 x
    - float32 y
    - float32 z

## Notes

- cloud_count typically equals chunk_size on the sender, unless a partial chunk is sent.
- lidar_id identifies the source sensor (e.g., 1, 2, 3).
- sequence increments by 1 for each payload sent per lidar_id.
- float32 values are written in the host native endianness; current deployment is little-endian.
