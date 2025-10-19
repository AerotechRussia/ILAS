#!/usr/bin/env python3
"""
ILAS - Intelligent Landing and Avoidance System
Main entry point
"""

import argparse
import sys
import numpy as np

from ilas.core import ILASCore
from ilas.utils.config import load_config, get_default_config


def main():
    """Main entry point for ILAS"""
    parser = argparse.ArgumentParser(
        description='ILAS - Intelligent Landing and Avoidance System for VTOL Drones'
    )
    parser.add_argument(
        '--config',
        type=str,
        help='Path to configuration file (YAML or JSON)'
    )
    parser.add_argument(
        '--mode',
        type=str,
        choices=['mission', 'test', 'interactive'],
        default='interactive',
        help='Operation mode'
    )
    parser.add_argument(
        '--mission',
        type=str,
        help='Path to mission file (for mission mode)'
    )
    
    args = parser.parse_args()
    
    # Load configuration
    if args.config:
        try:
            config = load_config(args.config)
            print(f"Loaded configuration from {args.config}")
        except Exception as e:
            print(f"Error loading config: {e}")
            print("Using default configuration")
            config = get_default_config()
    else:
        print("Using default configuration")
        config = get_default_config()
    
    # Initialize ILAS
    print("\n" + "="*60)
    print("ILAS - Intelligent Landing and Avoidance System")
    print("Copyright (c) 2025 AerotechRussia")
    print("="*60 + "\n")
    
    ilas = ILASCore(config)
    
    # Start system
    if not ilas.start():
        print("Failed to start ILAS system")
        sys.exit(1)
    
    try:
        if args.mode == 'mission':
            # Run mission from file
            if not args.mission:
                print("Mission file required for mission mode")
                sys.exit(1)
            
            mission_config = load_config(args.mission)
            print(f"Running mission: {args.mission}")
            ilas.run_mission(mission_config)
            
        elif args.mode == 'test':
            # Run test scenario
            print("Running test scenario...")
            run_test_scenario(ilas)
            
        else:  # interactive
            # Interactive mode
            print("Entering interactive mode...")
            print("Commands: status, navigate, land, emergency, quit")
            interactive_mode(ilas)
            
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        ilas.stop()
        print("ILAS system shutdown complete")


def run_test_scenario(ilas: ILASCore):
    """Run a simple test scenario"""
    import time
    
    print("\n=== Test Scenario: Obstacle Avoidance ===")
    
    # Simulate sensor data with obstacles
    sensor_data = {
        'lidar': np.array([
            [5.0, 0.0, 0.0],   # Obstacle ahead
            [5.1, 0.1, 0.0],
            [4.9, -0.1, 0.0],
        ])
    }
    
    # Target position
    target = np.array([10.0, 0.0, 5.0])
    
    print(f"Target position: {target}")
    print("Detecting obstacles and calculating avoidance...")
    
    # Navigate with avoidance
    nav_command, is_safe = ilas.navigate_to_target(target, sensor_data)
    
    print(f"Navigation command: {nav_command}")
    print(f"Safety status: {'SAFE' if is_safe else 'UNSAFE'}")
    
    time.sleep(2)
    
    print("\n=== Test Scenario: Landing Site Selection ===")
    
    # Test landing
    landing_center = np.array([10.0, 0.0, 0.0])
    print(f"Landing area center: {landing_center}")
    print("Analyzing landing sites...")
    
    success = ilas.execute_landing(landing_center, 10.0, sensor_data)
    
    if success:
        print("✓ Landing executed successfully")
    else:
        print("✗ Landing failed")
    
    time.sleep(1)
    
    # Print system status
    status = ilas.get_system_status()
    print("\n=== System Status ===")
    print(f"Running: {status['is_running']}")
    print(f"Controller connected: {status['controller_connected']}")
    print(f"Position: {status['telemetry']['position']}")


def interactive_mode(ilas: ILASCore):
    """Interactive command mode"""
    while True:
        try:
            cmd = input("\nILAS> ").strip().lower()
            
            if cmd == 'quit' or cmd == 'exit':
                break
            elif cmd == 'status':
                status = ilas.get_system_status()
                print("\n=== System Status ===")
                print(f"Running: {status['is_running']}")
                print(f"Controller: {status['controller_connected']}")
                print(f"Position: {status['telemetry']['position']}")
                print(f"Velocity: {status['telemetry']['velocity']}")
                
            elif cmd == 'navigate':
                try:
                    x = float(input("Target X: "))
                    y = float(input("Target Y: "))
                    z = float(input("Target Z: "))
                    target = np.array([x, y, z])
                    
                    sensor_data = {}  # Placeholder
                    nav_cmd, is_safe = ilas.navigate_to_target(target, sensor_data)
                    
                    print(f"Navigation: {nav_cmd}")
                    print(f"Safety: {'SAFE' if is_safe else 'UNSAFE'}")
                except ValueError:
                    print("Invalid coordinates")
                    
            elif cmd == 'land':
                try:
                    x = float(input("Landing area X: "))
                    y = float(input("Landing area Y: "))
                    z = float(input("Landing area Z: "))
                    radius = float(input("Search radius: "))
                    
                    center = np.array([x, y, z])
                    sensor_data = {}
                    
                    success = ilas.execute_landing(center, radius, sensor_data)
                    print(f"Landing: {'SUCCESS' if success else 'FAILED'}")
                except ValueError:
                    print("Invalid input")
                    
            elif cmd == 'emergency':
                print("Executing emergency landing...")
                ilas.emergency_land({})
                print("Emergency landing initiated")
                
            elif cmd == 'help':
                print("\nAvailable commands:")
                print("  status    - Show system status")
                print("  navigate  - Navigate to target position")
                print("  land      - Execute intelligent landing")
                print("  emergency - Emergency landing")
                print("  help      - Show this help")
                print("  quit      - Exit ILAS")
                
            else:
                print(f"Unknown command: {cmd}")
                print("Type 'help' for available commands")
                
        except EOFError:
            break


if __name__ == '__main__':
    main()
