
import cosysairsim as airsim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(False)

# client.simPause(True)


# client.simSetVehiclePose(airsim.Pose(airsim.Vector3r(0,0,-2), airsim.to_quaternion(0,0,0)), True)
# for i in range(100):
#     data = client.getLidarData("lidar1")
#     print(data.time_stamp)


mesh = client.simGetMeshPositionVertexBuffers()
    