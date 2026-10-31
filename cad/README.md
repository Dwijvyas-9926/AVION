# CAD

Mechanical parts for the AVION airframe, designed in-house. The airframe uses carbon-fibre tubes joined by 3D-printed nodes, so any broken joint can be re-printed from these files in the field.

- **`stl/`** contains print-ready meshes.
- **`step/`** contains editable solid models.
- **`3mf/`** contains slicer projects.

The full airframe assembly (`AVION ASSEMBLY.step`, 43 MB) is kept in the team's Google Drive (see the main [README](../README.md#files-hosted-outside-this-repository)). The ground station's 3D digital twin (`flight_software/gui_assets/avion_twin.glb`) was exported from that assembly.

The part descriptions below are inferred from file names and geometry. Sizes are bounding boxes in millimetres.

## Payload system

| File | Size (mm) | Description |
|---|---|---|
| `stl/payload_hook.stl` | 80 × 42 × 49 | Servo-actuated release hook (Pixhawk AUX1) |
| `stl/hook_upper_cap.stl` | 50 × 21 × 7 | Cap that closes the top of the hook mechanism |

## Electronics and sensor mounts

| File | Size (mm) | Description |
|---|---|---|
| `stl/Ras_pix_top.stl` | 89 × 12 × 61 | Raspberry Pi / Pixhawk stack mount, upper half |
| `stl/Ras_pix_Bottom.stl` | 65 × 10 × 52 | Raspberry Pi / Pixhawk stack mount, lower half |
| `stl/rpi_pixhawk_mount.stl` | 64 × 55 × 2 * | Flat Raspberry Pi / Pixhawk mounting plate |
| `step/rs3_cradle_v2.step` | – | Cradle for the RealSense camera (original name `RS3_cradle (2) v2.step`) |
| `stl/plate_lidar.stl` | 80 × 110 × 5 | Plate for a rangefinder / LiDAR module |
| `stl/rail_plate.stl` | 78 × 50 × 16 | Rail plate for mounting components |

\* `rpi_pixhawk_mount.stl` was exported at 1000× scale. Its raw bounding box is 64000 × 55000 × 2000 units, so scale it by 0.001 in the slicer.

## Frame and joints

| File | Size (mm) | Description |
|---|---|---|
| `stl/bottom_plate_updated.stl` | 134 × 149 × 12 | Central bottom plate |
| `stl/Body6.stl` / `3mf/Body6.3mf` | 132 × 222 × 6 | Body plate (STL and slicer project) |
| `stl/final_L_mount_updated.stl` | 59 × 59 × 37 | L-shaped tube joint |
| `stl/final_updated_T_mount.stl` | 30 × 52 × 17 | T-shaped tube joint |
| `stl/ET3_tray_arm_aligned.stl` | 181 × 181 × 21 | Tray aligned to the arms |
| `stl/PG1_guard_ring.stl` | 181 × 181 × 29 | Guard ring |
| `stl/PG2_collar_body.stl` | 51 × 51 × 31 | Collar body, paired with PG3 |
| `stl/PG3_collar_cap.stl` | 29 × 29 × 12 | Collar cap |
| `step/clamp_arm.step` | – | Tube clamp, arm |
| `step/clamp_cap.step` | – | Tube clamp, cap |
| `stl/Untitled.stl` | 30 × 43 × 60 | Unnamed part, kept as delivered |

Spaces in file names were replaced with underscores.
