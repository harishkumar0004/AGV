"""
Continuous Path Execution with Dynamic Distance & Trajectory Correction

This module handles:
1. Dynamic total distance calculation based on start/end tags
2. Single continuous trapezoidal motion for entire path
3. AprilTag detections for position confirmation & trajectory correction
4. Path distance calculation for each of k-shortest paths

Key Concept:
- Arduino executes ONE long motion profile for total distance
- Intermediate tags are reference points for correction, NOT stopping points
- If robot detects tag at wrong angle/offset, sends correction command
- No acceleration/deceleration between cells - continuous smooth motion
"""

import json
import math
import os
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass


@dataclass
class PathInfo:
    """Information about a single path."""
    coordinates: List[Tuple[int, int]]  # (row, col) coordinates
    node_ids: List[int]                 # 1-based node IDs
    num_cells: int                       # Number of cells
    total_distance_mm: float             # Total distance in mm
    waypoints: List[Tuple[int, int]]    # Expected waypoint IDs from tags
    
    def __repr__(self):
        return f"Path({self.num_cells}cells, {self.total_distance_mm:.0f}mm, nodes={self.node_ids})"


class PathDistanceCalculator:
    """Calculate distances for paths based on grid layout."""
    
    def __init__(self, distance_per_cell: float = 500.0):
        """
        Args:
            distance_per_cell: Distance between adjacent grid cells (mm)
        """
        self.distance_per_cell = distance_per_cell
    
    def calculate_path_distance(self, coords: List[Tuple[int, int]]) -> float:
        """
        Calculate total distance for a path.
        
        Args:
            coords: List of (row, col) grid coordinates
        
        Returns:
            Total distance in mm = (number of cells - 1) × distance_per_cell
        """
        if len(coords) < 2:
            return 0.0
        
        # Distance = number of moves × distance_per_move
        # For path [A, B, C, D], there are 3 moves
        num_moves = len(coords) - 1
        return num_moves * self.distance_per_cell
    
    def get_cell_count(self, coords: List[Tuple[int, int]]) -> int:
        """Get number of cells (edges) in path."""
        return len(coords) - 1


class MotionProfile:
    """Calculates trapezoidal motion profile for total distance."""
    
    def __init__(self, total_distance_mm: float, max_rpm: float = 60, 
                 initial_rpm: float = 2, acc_rpm_per_sec: float = 25):
        """
        Args:
            total_distance_mm: Total distance to travel (mm)
            max_rpm: Maximum RPM
            initial_rpm: Starting RPM (to avoid jerk)
            acc_rpm_per_sec: Acceleration rate (RPM/second)
        """
        self.total_distance_mm = total_distance_mm
        self.max_rpm = max_rpm
        self.initial_rpm = initial_rpm
        self.acc_rpm_per_sec = acc_rpm_per_sec
        
        # Will be calculated
        self.accel_time_sec = 0.0
        self.cruise_time_sec = 0.0
        self.decel_time_sec = 0.0
        self.accel_distance_mm = 0.0
        self.cruise_distance_mm = 0.0
        self.decel_distance_mm = 0.0
        self.total_time_sec = 0.0
        
        self._calculate_profile()
    
    def _calculate_profile(self):
        """Calculate trapezoidal profile for total distance."""
        # Using kinematic equations for trapezoidal motion
        # v = v0 + a*t
        # d = v0*t + 0.5*a*t²
        
        # Acceleration phase
        rpm_change = self.max_rpm - self.initial_rpm
        self.accel_time_sec = rpm_change / self.acc_rpm_per_sec
        avg_accel_rpm = (self.initial_rpm + self.max_rpm) / 2.0
        self.accel_distance_mm = avg_accel_rpm * self.accel_time_sec
        
        # Deceleration phase (symmetric)
        self.decel_time_sec = self.accel_time_sec
        self.decel_distance_mm = self.accel_distance_mm
        
        # Cruise phase
        self.cruise_distance_mm = self.total_distance_mm - self.accel_distance_mm - self.decel_distance_mm
        
        if self.cruise_distance_mm > 0:
            # Can reach max RPM
            self.cruise_time_sec = self.cruise_distance_mm / self.max_rpm
        else:
            # Distance too short - use triangular profile
            self._calculate_triangular_profile()
    
    def _calculate_triangular_profile(self):
        """Calculate triangular profile for short distances."""
        # Peak RPM achieved based on distance constraint
        # Using: d = 0.5*a*(t_accel + t_decel)² where t_accel = t_decel
        peak_rpm_squared = (self.initial_rpm ** 2) + \
                          (self.total_distance_mm * self.acc_rpm_per_sec * 60.0)
        peak_rpm = math.sqrt(peak_rpm_squared)
        
        # Recalculate with new peak
        self.accel_time_sec = (peak_rpm - self.initial_rpm) / self.acc_rpm_per_sec
        avg_rpm = (self.initial_rpm + peak_rpm) / 2.0
        self.accel_distance_mm = avg_rpm * self.accel_time_sec
        
        self.decel_time_sec = self.accel_time_sec
        self.decel_distance_mm = self.accel_distance_mm
        self.cruise_distance_mm = 0.0
        self.cruise_time_sec = 0.0
    
    def get_speed_at_time(self, elapsed_time_sec: float) -> float:
        """Get RPM at any point in motion."""
        if elapsed_time_sec <= self.accel_time_sec:
            return self.initial_rpm + (self.acc_rpm_per_sec * elapsed_time_sec)
        elif elapsed_time_sec <= (self.accel_time_sec + self.cruise_time_sec):
            return self.max_rpm
        else:
            decel_time = elapsed_time_sec - (self.accel_time_sec + self.cruise_time_sec)
            return max(0, self.max_rpm - (self.acc_rpm_per_sec * decel_time))
    
    def get_profile_summary(self) -> Dict:
        """Get profile summary for logging."""
        return {
            'total_distance_mm': self.total_distance_mm,
            'total_time_sec': self.accel_time_sec + self.cruise_time_sec + self.decel_time_sec,
            'accel_phase': {
                'distance_mm': self.accel_distance_mm,
                'time_sec': self.accel_time_sec
            },
            'cruise_phase': {
                'distance_mm': self.cruise_distance_mm,
                'time_sec': self.cruise_time_sec,
                'speed_rpm': self.max_rpm
            },
            'decel_phase': {
                'distance_mm': self.decel_distance_mm,
                'time_sec': self.decel_time_sec
            }
        }


class TrajectoryCorrection:
    """Handles position/angle corrections based on AprilTag detection."""
    
    def __init__(self, tag_layout_file: str = 'apriltag_layout.json'):
        """
        Args:
            tag_layout_file: Path to apriltag_layout.json
        """
        self.tag_layout = {}
        self.expected_waypoints = {}
        self._load_layout(tag_layout_file)
    
    def _load_layout(self, layout_file: str):
        """Load tag layout from JSON."""
        try:
            with open(layout_file, 'r') as f:
                entries = json.load(f)
            
            for entry in entries:
                tag_id = int(entry['id'])
                row = int(entry['row'])
                col = int(entry['col'])
                self.tag_layout[tag_id] = (row, col)
        except Exception as e:
            print(f"[TrajectoryCorrection] Failed to load layout: {e}")
    
    def analyze_tag_detection(self, tag_id: int, detected_angle: float, 
                             expected_position: Tuple[int, int]) -> Dict:
        """
        Analyze if detected tag is at correct position and angle.
        
        Args:
            tag_id: AprilTag ID
            detected_angle: Angle offset of detected tag (degrees)
            expected_position: Expected (row, col) for this tag
        
        Returns:
            Dict with correction recommendation
        """
        expected_pos = self.tag_layout.get(tag_id)
        
        if expected_pos is None:
            return {'status': 'unknown_tag', 'correction_needed': False}
        
        pos_match = (expected_pos == expected_position)
        angle_error = abs(detected_angle)  # 0° = straight-on
        
        correction = {
            'tag_id': tag_id,
            'position_match': pos_match,
            'angle_error_deg': angle_error,
            'correction_needed': False,
            'correction_type': None,
            'correction_magnitude': 0
        }
        
        # Check for misalignment
        if not pos_match:
            correction['correction_needed'] = True
            correction['correction_type'] = 'position_drift'
            print(f"[TrajectoryCorrection] Position drift: Tag {tag_id} at {expected_pos}, expected {expected_position}")
        
        # Check for angle offset (threshold: ±10°)
        if angle_error > 10:
            correction['correction_needed'] = True
            correction['correction_type'] = 'angle_correction'
            # Map angle error to steering correction (-1 to +1)
            correction['correction_magnitude'] = max(-1, min(1, detected_angle / 45.0))
        
        return correction
    
    def get_waypoint_for_path(self, path_coords: List[Tuple[int, int]]) -> List[int]:
        """
        Get expected tag IDs for each waypoint in path.
        
        Args:
            path_coords: List of (row, col) coordinates
        
        Returns:
            List of tag IDs corresponding to each waypoint
        """
        waypoint_tags = []
        for pos in path_coords:
            for tag_id, tag_pos in self.tag_layout.items():
                if tag_pos == pos:
                    waypoint_tags.append(tag_id)
                    break
        return waypoint_tags


class ContinuousPathExecutor:
    """
    Executes continuous motion for entire path with dynamic distance.
    Intermediate AprilTags used for trajectory verification only.
    """
    
    def __init__(self, distance_per_cell: float = 500.0,
                 max_rpm: float = 60, initial_rpm: float = 2,
                 acc_rpm_per_sec: float = 25):
        """
        Args:
            distance_per_cell: Distance between adjacent cells (mm)
            max_rpm: Maximum motor RPM
            initial_rpm: Starting RPM
            acc_rpm_per_sec: Acceleration rate
        """
        self.distance_calculator = PathDistanceCalculator(distance_per_cell)
        self.max_rpm = max_rpm
        self.initial_rpm = initial_rpm
        self.acc_rpm_per_sec = acc_rpm_per_sec
        self.trajectory_corrector = TrajectoryCorrection()
    
    def prepare_execution(self, path_coords: List[Tuple[int, int]],
                         path_node_ids: List[int]) -> Dict:
        """
        Prepare execution plan for a path.
        
        Args:
            path_coords: List of (row, col) grid coordinates
            path_node_ids: List of 1-based node IDs
        
        Returns:
            Dict with execution parameters for Arduino
        """
        # Calculate total distance
        total_distance_mm = self.distance_calculator.calculate_path_distance(path_coords)
        num_cells = self.distance_calculator.get_cell_count(path_coords)
        
        # Get waypoint tags
        waypoint_tags = self.trajectory_corrector.get_waypoint_for_path(path_coords)
        
        # Calculate motion profile
        motion_profile = MotionProfile(
            total_distance_mm=total_distance_mm,
            max_rpm=self.max_rpm,
            initial_rpm=self.initial_rpm,
            acc_rpm_per_sec=self.acc_rpm_per_sec
        )
        
        profile_summary = motion_profile.get_profile_summary()
        
        exec_plan = {
            'path_coords': path_coords,
            'path_node_ids': path_node_ids,
            'total_distance_mm': total_distance_mm,
            'num_cells': num_cells,
            'waypoint_tags': waypoint_tags,
            'motion_profile': profile_summary,
            'total_execution_time_sec': profile_summary['total_time_sec'],
            'arduino_params': {
                'total_distance_mm': int(total_distance_mm),
                'distance_per_cell_mm': int(self.distance_calculator.distance_per_cell),
                'max_rpm': int(self.max_rpm),
                'initial_rpm': int(self.initial_rpm),
                'acc_rpm_per_sec': int(self.acc_rpm_per_sec)
            }
        }
        
        return exec_plan

    def print_execution_plan(self, exec_plan: Dict):
        """Pretty print execution plan."""
        print("\n" + "="*80)
        print("EXECUTION PLAN - CONTINUOUS MOTION WITH TRAJECTORY CORRECTION")
        print("="*80)
        
        print(f"\nPath Information:")
        print(f"  Coordinates: {exec_plan['path_coords']}")
        print(f"  Node IDs: {exec_plan['path_node_ids']}")
        print(f"  Number of cells: {exec_plan['num_cells']}")
        print(f"  Total distance: {exec_plan['total_distance_mm']:.1f} mm")
        
        print(f"\nWaypoint Tags for Position Verification:")
        for i, (coord, tag_id) in enumerate(zip(exec_plan['path_coords'], 
                                                 exec_plan['waypoint_tags'])):
            print(f"  [{i}] Coordinate {coord} → Tag {tag_id}")
        
        print(f"\nMotion Profile (SINGLE continuous motion):")
        mp = exec_plan['motion_profile']
        print(f"  Total time: {mp['total_time_sec']:.2f} seconds")
        print(f"  \nAcceleration Phase:")
        print(f"    Distance: {mp['accel_phase']['distance_mm']:.1f} mm")
        print(f"    Time: {mp['accel_phase']['time_sec']:.2f} sec")
        print(f"    RPM: {self.initial_rpm} → {self.max_rpm}")
        
        if mp['cruise_phase']['distance_mm'] > 0:
            print(f"  \nCruise Phase:")
            print(f"    Distance: {mp['cruise_phase']['distance_mm']:.1f} mm")
            print(f"    Time: {mp['cruise_phase']['time_sec']:.2f} sec")
            print(f"    Speed: {mp['cruise_phase']['speed_rpm']} RPM (constant)")
        
        print(f"  \nDeceleration Phase:")
        print(f"    Distance: {mp['decel_phase']['distance_mm']:.1f} mm")
        print(f"    Time: {mp['decel_phase']['time_sec']:.2f} sec")
        print(f"    RPM: {self.max_rpm} → {self.initial_rpm}")
        
        print(f"\nArduino Parameters (send before starting motion):")
        ap = exec_plan['arduino_params']
        print(f"  TOTAL_DIST_MM = {ap['total_distance_mm']}")
        print(f"  DISTANCE_PER_CELL_MM = {ap['distance_per_cell_mm']}")
        print(f"  MAX_RPM = {ap['max_rpm']}")
        print(f"  INITIAL_RPM = {ap['initial_rpm']}")
        print(f"  ACC_RPM_PER_SEC = {ap['acc_rpm_per_sec']}")
        
        print("\n" + "="*80)


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    """
    Example: Execute path from Tag 1 to Tag 5 with continuous motion
    """
    
    # Example paths (from A* pathfinding)
    paths = [
        [(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)],  # 2000 mm (4 cells)
        [(0, 0), (1, 0), (1, 1), (1, 2), (1, 3), (0, 3), (0, 4)],  # 3000 mm (6 cells)
        [(0, 0), (0, 1), (1, 1), (1, 2), (1, 3), (0, 3), (0, 4)],  # 3000 mm
        # ... more paths
    ]
    
    executor = ContinuousPathExecutor(
        distance_per_cell=500.0,
        max_rpm=60,
        initial_rpm=2,
        acc_rpm_per_sec=25
    )
    
    print("\nANALYZING ALL PATHS:")
    print("="*80)
    
    for i, path in enumerate(paths, 1):
        distance = executor.distance_calculator.calculate_path_distance(path)
        num_cells = executor.distance_calculator.get_cell_count(path)
        print(f"\nPath {i}: {num_cells} cells × 500mm/cell = {distance:.0f} mm total")
        print(f"  Coordinates: {path}")
    
    # Pick one path to execute
    selected_path = paths[0]
    path_node_ids = list(range(1, len(selected_path) + 1))  # Dummy node IDs
    
    print("\n\n" + "="*80)
    print(f"SELECTED PATH {1}: Preparing for execution...")
    print("="*80)
    
    exec_plan = executor.prepare_execution(selected_path, path_node_ids)
    executor.print_execution_plan(exec_plan)
