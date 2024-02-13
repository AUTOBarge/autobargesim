# Guidance

## About
The objective of this class is to employ a guidance controller to generate reference course angles and speed commands for a vessel.

## Dependencies
N/A

## Features
The following methods are avialble for the user:
- #### [chi_d, U_d] = compute_LOSRef(wp_pos, wp_speed, x, wp_idx):
  This method is used to calculate reference course angles and speed commands.
- #### wp_idx = find_active_wp_segment(wp_pos, x, wp_idx):
  This method is used to find the active waypoint segment depending on the current position of the vessel.

## Contact
Dhanika Mahipala: dhanika.mahipala@km.kongsberg.com
Hoang Anh Tran: hoang.a.tran@ntnu.no
