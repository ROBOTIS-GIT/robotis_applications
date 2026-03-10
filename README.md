# robotis_applications

ROS Packages for Robotis Applications

This repository contains the official ROS 2 packages for the ROBOTIS applications. For detailed usage instructions, please refer to the documentation below.
  - [Documentation for Robotis Applications](https://ai.robotis.com/)

For usage instructions and demonstrations of the ROBOTIS Hand, check out:
  - [Tutorial Videos](https://www.youtube.com/@ROBOTISOpenSourceTeam)

To use the Docker image for running ROS packages with the ROBOTIS Hand, visit:
  - [Docker Images](https://hub.docker.com/r/robotis/robotis-applications/tags)

## Docker Usage

The main entrypoint is `docker/container.sh`.
Docker builds for both `amd64` and `arm64` use the shared `docker/Dockerfile`.

Build and start the container:

```bash
./docker/container.sh start
```

On the first run, `start` also generates `robotis_vuer` certificates automatically
if `cert.pem` and `key.pem` are missing, using the host IP detected on the host
machine.

Enter the running container:

```bash
./docker/container.sh enter
```

Stop the container:

```bash
./docker/container.sh stop
```

## Launch VR Publisher

The `robotis_vuer` package provides one launch file with a `model` argument.

Run SH5: (default)

```bash
ros2 launch robotis_vuer vr.launch.py model:=sh5
```

Run SG2:

```bash
ros2 launch robotis_vuer vr.launch.py model:=sg2
```

## Certificates

Generate certificates inside the container:

```bash
/root/gen_cert.sh
```

`gen_cert.sh` auto-detects the host IP by default. You can still override it:

```bash
/root/gen_cert.sh 192.168.0.10
```
