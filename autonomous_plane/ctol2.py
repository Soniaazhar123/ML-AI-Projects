# # # import time
# # # import json
# # # import numpy as np
# # # from pymavlink import mavutil

# # # # Function to read coordinates from a file
# # # def read_coordinates_from_file(filename):
# # #     with open(filename, 'r') as file:
# # #         data = json.load(file)
# # #     return data

# # # # Connecting the drone
# # # master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

# # # try:
# # #     # Wait for the heartbeat message to find the system ID
# # #     master.wait_heartbeat()

# # #     # Function to send commands to the drone
# # #     def send_command_long(command, param1, param2, param3, param4, param5, param6, param7):
# # #         master.mav.command_long_send(
# # #             master.target_system,
# # #             master.target_component,
# # #             command,
# # #             0,
# # #             param1, param2, param3, param4, param5, param6, param7
# # #         )

# # #     # Arming the drone
# # #     send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
# # #     print("Arming the drone...")
# # #     time.sleep(2)

# # #     # Set mode to TAKEOFF
# # #     mode = 'TAKEOFF'
# # #     mode_id = master.mode_mapping()[mode]
# # #     master.set_mode(mode_id)
# # #     print("Setting mode to TAKEOFF...")
# # #     time.sleep(10)  # Time for the fixed-wing to climb to a safe altitude

# # #     # Load coordinates from the file
# # #     coordinates_file = "coordinates.txt"
# # #     coordinates = read_coordinates_from_file(coordinates_file)

# # #     # Extract latitudes and longitudes from the file
# # #     boundary_lat = [point['latitude'] for point in coordinates]
# # #     boundary_long = [point['longitude'] for point in coordinates]

# # #     # Close the boundary (if needed)
# # #     boundary_lat.append(boundary_lat[0])
# # #     boundary_long.append(boundary_long[0])

# # #     # Calculate zigzag waypoints, adapted for smoother turns
# # #     lat_min, lat_max = min(boundary_lat), max(boundary_lat)
# # #     long_min, long_max = min(boundary_long), max(boundary_long)

# # #     num_lines = 10  # Number of zigzag lines
# # #     lat_points = np.linspace(lat_min, lat_max, num_lines)
# # #     waypoints = []

# # #     for i, lat in enumerate(lat_points):
# # #         if i % 2 == 0:
# # #             waypoints.append((lat, long_min))  # Start of zigzag line
# # #             waypoints.append((lat, long_max))  # End of zigzag line
# # #         else:
# # #             waypoints.append((lat, long_max))
# # #             waypoints.append((lat, long_min))

# # #     # Send mission count
# # #     master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
# # #     print(f"Sending mission count: {len(waypoints)}")

# # #     # Send waypoints
# # #     for i, waypoint in enumerate(waypoints):
# # #         lat, lon = waypoint
# # #         alt = 100  # MUST be adjusted accordingly
# # #         master.mav.mission_item_send(
# # #             master.target_system,
# # #             master.target_component,
# # #             i,
# # #             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
# # #             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
# # #             0, 1, 0, 0, 0, 0,
# # #             lat, lon, alt
# # #         )
# # #         print(f"Sending waypoint {i}: lat={lat}, lon={lon}, alt={alt}")
# # #         time.sleep(1)

# # #     # Set the first waypoint as current
# # #     master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
# # #     print("Setting first waypoint as current...")

# # #     # Give some time for the drone to process the mission items
# # #     time.sleep(5)

# # #     # Set the mode to AUTO
# # #     mode = 'AUTO'
# # #     mode_id = master.mode_mapping()[mode]
# # #     master.set_mode(mode_id)
# # #     print("Setting mode to AUTO...")
# # #     time.sleep(2)

# # #     # Wait for the drone to complete the mission
# # #     time.sleep(150)  # Calculate this delay based on mission size and distance

# # #     # Set mode to RTL
# # #     mode = 'RTL'
# # #     mode_id = master.mode_mapping()[mode]
# # #     master.set_mode(mode_id)
# # #     print("Setting mode to RTL...")
# # #     time.sleep(40)  # Adjust this delay as needed

# # #     # Land the drone
# # #     send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
# # #     print("Landing the drone...")

# # # except Exception as e:
# # #     print("Error:", e)

# # # finally:
# # #     # Close the connection
# # #     master.close()

# # import time
# # import json
# # import numpy as np
# # from pymavlink import mavutil

# # # Function to read coordinates from a file
# # def read_coordinates_from_file(filename):
# #     with open(filename, 'r') as file:
# #         data = json.load(file)
# #     return data

# # # Connecting to the drone
# # master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

# # try:
# #     # Wait for heartbeat to find system ID
# #     master.wait_heartbeat()

# #     # Function to send MAVLink commands
# #     def send_command_long(command, param1, param2, param3, param4, param5, param6, param7):
# #         master.mav.command_long_send(
# #             master.target_system,
# #             master.target_component,
# #             command,
# #             0,
# #             param1, param2, param3, param4, param5, param6, param7
# #         )

# #     # **Arming the drone**
# #     send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
# #     print("Arming the drone...")
# #     time.sleep(2)

# #     # **Set mode to TAKEOFF**
# #     mode = 'TAKEOFF'
# #     mode_id = master.mode_mapping()[mode]
# #     master.set_mode(mode_id)
# #     print("Setting mode to TAKEOFF...")
# #     time.sleep(10)  # Allow time for climb

# #     # Load coordinates
# #     coordinates_file = "coordinates.txt"
# #     coordinates = read_coordinates_from_file(coordinates_file)

# #     # Extract boundary lat/lon values
# #     boundary_lat = [point['latitude'] for point in coordinates]
# #     boundary_long = [point['longitude'] for point in coordinates]

# #     # Define **zigzag parameters**
# #     lat_min, lat_max = min(boundary_lat), max(boundary_lat)
# #     long_min, long_max = min(boundary_long), max(boundary_long)
# #     num_lines = 10  # Number of zigzag lines
# #     altitude = 100  # Set fixed altitude for waypoints
# #     lat_points = np.linspace(lat_min, lat_max, num_lines)
# #     waypoints = []

# #     # **Generate zigzag pattern with loiter turns**
# #     for i, lat in enumerate(lat_points):
# #         if i % 2 == 0:
# #             waypoints.append((lat, long_min, altitude))  # Start of zigzag
# #             waypoints.append((lat, long_max, altitude))  # End of zigzag
# #             waypoints.append((lat, long_max, altitude, "loiter"))  # Loiter to turn
# #         else:
# #             waypoints.append((lat, long_max, altitude))
# #             waypoints.append((lat, long_min, altitude))
# #             waypoints.append((lat, long_min, altitude, "loiter"))  # Loiter to turn

# #     # **Send mission count**
# #     master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
# #     print(f"Sending mission count: {len(waypoints)}")

# #     # **Send waypoints**
# #     for i, waypoint in enumerate(waypoints):
# #         lat, lon, alt = waypoint[:3]
# #         is_loiter = len(waypoint) > 3  # Check if loiter is needed

# #         command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS if is_loiter else mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

# #         master.mav.mission_item_send(
# #             master.target_system,
# #             master.target_component,
# #             i,
# #             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
# #             command,
# #             0, 1, 0, 0, 0, 0,
# #             lat, lon, alt
# #         )

# #         print(f"Sending waypoint {i}: lat={lat}, lon={lon}, alt={alt} {'(LOITER TURN)' if is_loiter else ''}")
# #         time.sleep(1)

# #     # **Set the first waypoint as current**
# #     master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
# #     print("Setting first waypoint as current...")

# #     # **Set mode to AUTO**
# #     time.sleep(5)
# #     mode = 'AUTO'
# #     mode_id = master.mode_mapping()[mode]
# #     master.set_mode(mode_id)
# #     print("Setting mode to AUTO...")

# #     # **Wait for mission completion**
# #     time.sleep(150)  # Adjust based on mission distance

# #     # **Return to Launch (RTL)**
# #     mode = 'RTL'
# #     mode_id = master.mode_mapping()[mode]
# #     master.set_mode(mode_id)
# #     print("Setting mode to RTL...")
# #     time.sleep(40)

# #     # **Land the drone**
# #     send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
# #     print("Landing the drone...")

# # except Exception as e:
# #     print("Error:", e)

# # finally:
# #     # **Close MAVLink connection**
# #     master.close()



# import time
# import json
# import numpy as np
# from pymavlink import mavutil

# # Function to read coordinates from a file
# def read_coordinates_from_file(filename):
#     with open(filename, 'r') as file:
#         data = json.load(file)
#     return data

# # Connecting the drone
# master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

# try:
#     # Wait for the heartbeat message to find the system ID
#     master.wait_heartbeat()

#     # Function to send commands to the drone
#     def send_command_long(command, param1, param2, param3, param4, param5, param6, param7):
#         master.mav.command_long_send(
#             master.target_system,
#             master.target_component,
#             command,
#             0,
#             param1, param2, param3, param4, param5, param6, param7
#         )

#     # Arming the drone
#     send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
#     print("Arming the drone...")
#     time.sleep(2)

#     # Set mode to TAKEOFF
#     mode = 'TAKEOFF'
#     mode_id = master.mode_mapping()[mode]
#     master.set_mode(mode_id)
#     print("Setting mode to TAKEOFF...")
#     time.sleep(10)  # Time for the fixed-wing to climb to a safe altitude

#     # Load coordinates from the file (boundary coordinates)
#     coordinates_file = "coordinates.txt"
#     coordinates = read_coordinates_from_file(coordinates_file)

#     # Extract latitudes and longitudes from the file
#     boundary_lat = [point['latitude'] for point in coordinates]
#     boundary_long = [point['longitude'] for point in coordinates]

#     # Close the boundary (if needed)
#     boundary_lat.append(boundary_lat[0])
#     boundary_long.append(boundary_long[0])

#     # Define the boundary coordinates for the 120m x 120m area (adjust based on location)
#     lat_min = boundary_lat[0] - 0.00054  # 60 meters south
#     lat_max = boundary_lat[0] + 0.00054  # 60 meters north
#     long_min = boundary_long[0] - 0.00108  # 60 meters west
#     long_max = boundary_long[0] + 0.00108  # 60 meters east

#     # Calculate zigzag waypoints, adapted for smoother turns
#     num_lines = 10  # Number of zigzag lines
#     lat_points = np.linspace(lat_min, lat_max, num_lines)
#     waypoints = []

#     for i, lat in enumerate(lat_points):
#         if i % 2 == 0:
#             waypoints.append((lat, long_min))  # Start of zigzag line
#             waypoints.append((lat, long_max))  # End of zigzag line
#             waypoints.append((lat, long_max, 'loiter'))  # Loiter turn at end of zigzag line
#         else:
#             waypoints.append((lat, long_max))  # Start of reverse zigzag line
#             waypoints.append((lat, long_min))  # End of reverse zigzag line
#             waypoints.append((lat, long_min, 'loiter'))  # Loiter turn at end of zigzag line

#     # Send mission count
#     master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
#     print(f"Sending mission count: {len(waypoints)}")

#     # Send waypoints
#     for i, waypoint in enumerate(waypoints):
#         lat, lon = waypoint[:2]
#         alt = 100  # Adjust this altitude if necessary
#         command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

#         if len(waypoint) > 2 and waypoint[2] == 'loiter':
#             command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS  # Set loiter command for smooth turns

#         master.mav.mission_item_send(
#             master.target_system,
#             master.target_component,
#             i,
#             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
#             command,
#             0, 1, 0, 0, 0, 0,
#             lat, lon, alt
#         )
#         print(f"Sending waypoint {i}: lat={lat}, lon={lon}, alt={alt}")
#         time.sleep(1)

#     # Set the first waypoint as current
#     master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
#     print("Setting first waypoint as current...")

#     # Give some time for the drone to process the mission items
#     time.sleep(5)

#     # Set the mode to AUTO
#     mode = 'AUTO'
#     mode_id = master.mode_mapping()[mode]
#     master.set_mode(mode_id)
#     print("Setting mode to AUTO...")
#     time.sleep(2)

#     # Wait for the drone to complete the mission
#     time.sleep(150)  # Calculate this delay based on mission size and distance

#     # Set mode to RTL
#     mode = 'RTL'
#     mode_id = master.mode_mapping()[mode]
#     master.set_mode(mode_id)
#     print("Setting mode to RTL...")
#     time.sleep(40)  # Adjust this delay as needed

#     # Land the drone
#     send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
#     print("Landing the drone...")

# except Exception as e:
#     print("Error:", e)

# finally:
#     # Close the connection
#     master.close()




import time
import json
import numpy as np
from pymavlink import mavutil

# Function to read coordinates from a file
def read_coordinates_from_file(filename):
    with open(filename, 'r') as file:
        data = json.load(file)
    return data

# Connecting the drone
master = mavutil.mavlink_connection('tcp:127.0.0.1:14550')

try:
    # Wait for the heartbeat message to find the system ID
    master.wait_heartbeat()

    # Function to send commands to the drone
    def send_command_long(command, param1, param2, param3, param4, param5, param6, param7):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            command,
            0,
            param1, param2, param3, param4, param5, param6, param7
        )

    # Arming the drone
    send_command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1, 0, 0, 0, 0, 0, 0)
    print("Arming the drone...")
    time.sleep(2)

    # Set mode to TAKEOFF
    mode = 'TAKEOFF'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Setting mode to TAKEOFF...")
    time.sleep(10)  # Time for the fixed-wing to climb to a safe altitude

    # Load coordinates from the file
    coordinates_file = "coordinates.txt"
    coordinates = read_coordinates_from_file(coordinates_file)

    # Extract latitudes and longitudes from the file
    boundary_lat = [point['latitude'] for point in coordinates]
    boundary_long = [point['longitude'] for point in coordinates]

    # Close the boundary (if needed)
    boundary_lat.append(boundary_lat[0])
    boundary_long.append(boundary_long[0])

    # Calculate zigzag waypoints, adapted for smoother turns
    lat_min, lat_max = min(boundary_lat), max(boundary_lat)
    long_min, long_max = min(boundary_long), max(boundary_long)

    num_lines = 10  # Number of zigzag lines
    lat_points = np.linspace(lat_min, lat_max, num_lines)
    waypoints = []

    for i, lat in enumerate(lat_points):
        if i % 2 == 0:
            waypoints.append((lat, long_min))  # Start of zigzag line
            waypoints.append((lat, long_max))  # End of zigzag line
            waypoints.append((lat, long_max, 'loiter'))  # Loiter turn at end of zigzag line
        else:
            waypoints.append((lat, long_max))  # Start of reverse zigzag line
            waypoints.append((lat, long_min))  # End of reverse zigzag line
            waypoints.append((lat, long_min, 'loiter'))  # Loiter turn at end of zigzag line

    # Send mission count
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
    print(f"Sending mission count: {len(waypoints)}")

    # Send waypoints
    for i, waypoint in enumerate(waypoints):
        lat, lon = waypoint[:2]
        alt = 100  # Adjust this altitude if necessary
        command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT

        if len(waypoint) > 2 and waypoint[2] == 'loiter':
            command = mavutil.mavlink.MAV_CMD_NAV_LOITER_TURNS  # Set loiter command for smooth turns

        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            i,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            command,
            0, 1, 0, 0, 0, 0,
            lat, lon, alt
        )
        print(f"Sending waypoint {i}: lat={lat}, lon={lon}, alt={alt}")
        time.sleep(1)

    # Set the first waypoint as current
    master.mav.mission_set_current_send(master.target_system, master.target_component, 0)
    print("Setting first waypoint as current...")

    # Give some time for the drone to process the mission items
    time.sleep(5)

    # Set the mode to AUTO
    mode = 'AUTO'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Setting mode to AUTO...")
    time.sleep(2)

    # Wait for the drone to complete the mission
    time.sleep(150)  # Calculate this delay based on mission size and distance

    # Set mode to RTL
    mode = 'RTL'
    mode_id = master.mode_mapping()[mode]
    master.set_mode(mode_id)
    print("Setting mode to RTL...")
    time.sleep(40)  # Adjust this delay as needed

    # Land the drone
    send_command_long(mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
    print("Landing the drone...")

except Exception as e:
    print("Error:", e)

# # finally:
# #     # Close the connection
# #     master.close()




