import logging
from flask import Flask, jsonify, request
import docker
import os

# Configuration for the web server
WEBSERVER_IP = '0.0.0.0'
WEBSERVER_PORT = 9999

# Initialize Flask app
app = Flask(__name__)

# Setup logging
logging.basicConfig(level=logging.INFO)

# Initialize Docker clients for edge and robot environments
DOCKER_CLIENTS = {
    'edge': docker.from_env(),
    'robot': docker.DockerClient(base_url='tcp://192.168.40.70:2375')
}


@app.route("/deploy/", methods=['POST'])
def deploy_post():
    """
    Endpoint to deploy services. It automatically deploys predefined Docker containers
    on edge and robot clients.
    """
    logging.info("DEPLOYING SERVICE")
    try:
        create_virtual_instance('go1-roscore', 'roscore-edge', 'edge', None, None, 'ENABLE_STATISTICS=true')

        # create_digital_twin_app_instance('isaac-sim:2023.1.0-ubuntu22.04','digital-twin-app','edge')

        # create_gesture_control_app_instance('go1-gesture-control','gesture-control-app','edge')

        create_virtual_instance('go1-base', 'go1-base', 'robot', 'TARGET_IP=192.168.123.161', {'8082/udp': 8082, '8090/udp': 8090, '8091/udp': 8091, '8007/udp': 8007, '8080/tcp': 8080, '8081/tcp': 8081})
        
        # create_rviz_vnc_instance('go1-rviz-vnc', 'rviz-vnc', 'edge', {'80/tcp': 6080})
        
        # create_sensor_instance('rplidar-lidar', 'lidar','robot', '/dev/rplidar:/dev/rplidar:rwm', False) 

        # create_sensor_instance('astra-camera', 'camera','robot', '/dev/astra:/dev/astra', True) 
        
        # create_virtual_instance('go1-navigation', 'go1-navigation', 'edge')

        
        return jsonify({'success': True}), 200
    except Exception as e:
        logging.error(f"Deployment failed: {e}")
        return jsonify({'error': str(e)}), 500


@app.route("/terminate/", methods=['POST'])
def terminate_post():
    """
    Endpoint to terminate all running services. It stops and removes all containers
    managed by the defined Docker clients.
    """
    logging.info("TERMINATING SERVICE")
    try:
        for client in DOCKER_CLIENTS.values():
            for container in client.containers.list():
                logging.info(f"Stopping and removing {container.name}")
                container.stop()
                container.remove(force=True)
        return jsonify({'success': True}), 200
    except Exception as e:
        logging.error(f"Termination failed: {e}")
        return jsonify({'error': str(e)}), 500


@app.route("/stop/", methods=['POST'])
def stop_post():
    """
    Endpoint to stop specific containers based on the names provided in the request.
    """
    data = request.json
    names = data.get('containerName', [])
    logging.info(f"Stopping containers: {names}")
    try:
        for client in DOCKER_CLIENTS.values():
            for container in client.containers.list():
                if container.name in names:
                    logging.info(f"Stopping {container.name}")
                    container.stop()
        return jsonify({'success': True}), 200
    except Exception as e:
        logging.error(f"Stop operation failed: {e}")
        return jsonify({'error': str(e)}), 500


@app.route("/start/", methods=['POST'])
def start_post():
    """
    Endpoint to start specific containers based on the names provided in the request.
    """
    data = request.json
    names = data.get('containerName', [])
    logging.info(f"Starting containers: {names}")
    try:
        for client in DOCKER_CLIENTS.values():
            for container in client.containers.list(all=True):
                if container.name in names:
                    logging.info(f"Starting {container.name}")
                    container.start()
        return jsonify({'success': True}), 200
    except Exception as e:
        logging.error(f"Start operation failed: {e}")
        return jsonify({'error': str(e)}), 500

def create_virtual_instance(docker_image, instance_name, constrain, robot_target_ip=None, ports=None, enable_statistice=None):
    """
    Helper function to create a Docker container instance.
    """
    client = DOCKER_CLIENTS.get(constrain)
    if client:
        env_vars = ['ROS_MASTER_URI=http://roscore-edge:11311']

        # Add robot_target_ip to environment variables if it's provided
        if robot_target_ip:
            env_vars.append(robot_target_ip)
        
        if enable_statistice:
            env_vars.append(enable_statistice)

        # Initialize additional options
        container_options = {
            "image": docker_image,
            "hostname": instance_name,
            "name": instance_name,
            "network": 'digital-twin-service',
            "environment": env_vars,
            "detach": True,
            "privileged": True,
        }
        
        # Add ports if specified (vnc container)
        if ports:
            container_options["ports"] = ports

        try:
            client.containers.run(**container_options)
            logging.info(f"{instance_name} instance created successfully.")
        except docker.errors.DockerException as e:
            logging.error(f"Failed to create {instance_name}: {e}")

def create_rviz_vnc_instance(docker_image, instance_name, constrain, ports={'80/tcp': 6080}):
    """
    Helper function to create a Docker container instance.
    """
    client = DOCKER_CLIENTS.get(constrain)
    if client:
        env_vars = ['ROS_MASTER_URI=http://roscore-edge:11311']
        
        # Initialize additional options
        container_options = {
            "image": docker_image,
            "hostname": instance_name,
            "name": instance_name,
            "network": 'digital-twin-service',
            "environment": env_vars,
            "detach": True,
            "ports": ports,
            "shm_size": '1g'
        }

        try:
            client.containers.run(**container_options)
            logging.info(f"{instance_name} instance created successfully.")
        except docker.errors.DockerException as e:
            logging.error(f"Failed to create {instance_name}: {e}")


def create_sensor_instance(docker_image, instance_name, constrain, device, privilege):
    """
    Helper function to create a Docker container for sensor instances.
    """
    client = DOCKER_CLIENTS.get(constrain)
    if client:
        env_vars = ['ROS_MASTER_URI=http://roscore-edge:11311']

        try:
            client.containers.run(
                image=docker_image,
                hostname=instance_name,
                name=instance_name,
                network='digital-twin-service',
                environment=env_vars,
                devices=[device],
                privileged=privilege,
                detach=True
            )
            logging.info(f"{instance_name} sensor instance created successfully.")
        except docker.errors.DockerException as e:
            logging.error(f"Failed to create sensor {instance_name}: {e}")

def create_gesture_control_app_instance(docker_image, instance_name, constrain):
    """
    Helper function to create a Docker container for gesture control app GUI instance.
    """
    client = DOCKER_CLIENTS.get(constrain)
    if client:

        # Set ROS_MASTER_URI 
        ros_master_uri = "http://roscore-edge:11311" 
        web_server = "yes"
        camera_type = "webcam_ip"
        control_loop_rate = "50"
        cmd_vel = "go1_controller/cmd_vel"
        stamped = "true"


        env_vars = {
            "CAMERA_TYPE": camera_type,
            "CONTROL_LOOP_RATE": control_loop_rate, 
            "WEB_SERVER": web_server,
            "ROS_MASTER_URI": ros_master_uri, 
            "CMD_VEL": cmd_vel,
            "STAMPED": stamped
        }

        # Ports to expose and map for the Flask app
        ports = {
            "8888/tcp": 8888,
        }

        try:
            client.containers.run(
                image=docker_image,
                hostname=instance_name,
                name=instance_name,
                network='digital-twin-service',
                ports=ports,
                environment=env_vars,
                privileged=True,
                group_add=["video"],
                detach=True
            )
            logging.info(f"{instance_name} instance created successfully.")
        except docker.errors.DockerException as e:
            logging.error(f"Failed to create {instance_name}: {e}")


def create_digital_twin_app_instance(docker_image, instance_name, constrain):
    """
    Helper function to create a Docker container for DT app (Isaac Sim) instance.
    """
    client = DOCKER_CLIENTS.get(constrain)
    if client:

        # Set ROS_MASTER_URI 
        ros_master_uri = "http://roscore-edge:11311" 

        relative_path = "../../digital-twin-service/digital-replica/my-environments"
        host_dir = os.path.abspath(os.path.join(os.getcwd(), relative_path))

        # Environment variables
        env_vars = {
            "ACCEPT_EULA": "Y",
            "PRIVACY_CONSENT": "Y",
            "ROS_MASTER_URI": ros_master_uri,
        }

        # Container entrypoint
        # command=["./runheadless.webrtc.sh", "-v"]
        command=["./runheadless.native.sh", "-v"]

        # Ports to expose and map
        ports = {
            # TCP and UDP ports for 47995-48012
            **{f"{port}/tcp": port for port in range(47995, 48013)},
            **{f"{port}/udp": port for port in range(47995, 48013)},
            # TCP and UDP ports for 49000-49007
            **{f"{port}/tcp": port for port in range(49000, 49008)},
            **{f"{port}/udp": port for port in range(49000, 49008)},
            # TCP ports for 49100
            "49100/tcp": 49100,
            # Exposing ports for web streaming
            "8211/tcp": 8211,
            "8211/udp": 8211,
            "8011/tcp": 8011,
            "8011/udp": 8011,
            "8111/tcp": 8111,
            "8111/udp": 8111,
            "8211/tcp": 8211,
            "8211/udp": 8211,
            "8311/tcp": 8311,
            "8311/udp": 8311,
        }

        # Volume mappings
        volumes = {
            os.path.expanduser("~/docker/isaac-sim/cache/kit"): {"bind": "/isaac-sim/kit/cache", "mode": "rw"},
            os.path.expanduser("~/docker/isaac-sim/cache/ov"): {"bind": "/root/.cache/ov", "mode": "rw"},
            os.path.expanduser("~/docker/isaac-sim/cache/pip"): {"bind": "/root/.cache/pip", "mode": "rw"},
            os.path.expanduser("~/docker/isaac-sim/cache/glcache"): {"bind": "/root/.cache/nvidia/GLCache", "mode": "rw"},
            os.path.expanduser("~/docker/isaac-sim/cache/computecache"): {"bind": "/root/.nv/ComputeCache", "mode": "rw"},
            os.path.expanduser("~/docker/isaac-sim/logs"): {"bind": "/root/.nvidia-omniverse/logs", "mode": "rw"},
            os.path.expanduser("~/docker/isaac-sim/data"): {"bind": "/root/.local/share/ov/data", "mode": "rw"},
            os.path.expanduser("~/docker/isaac-sim/documents"): {"bind": "/root/Documents", "mode": "rw"},
            host_dir: {"bind": "/isaac-sim/my-environments", "mode": "rw"},
        }
        try:
            client.containers.run(
                image=docker_image,
                hostname=instance_name,
                name=instance_name,
                entrypoint="bash",
                command=command,
                environment=env_vars,
                volumes=volumes,
                network='digital-twin-service',
                ports=ports,
                runtime="nvidia",
                device_requests=[docker.types.DeviceRequest(count=-1, capabilities=[['gpu']])],
                detach=True,
            )
            logging.info(f"{instance_name} instance created successfully.")
        except docker.errors.DockerException as e:
            logging.error(f"Failed to create {instance_name}: {e}")

# Main entry point of the application
if __name__ == "__main__":
    app.run()
