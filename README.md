# Fault Tolerance for MAVLink-based Mission Programs Using Checkpointing and Logging

This artifact accompanies the paper **"Fault Tolerance for MAVLink-based Mission Programs Using Checkpointing and Logging"**, submitted to the **26th International Conference on Distributed Applications and Interoperable Systems (DAIS 26)**.

It contains the full implementation of the proposed fault-tolerance mechanism, enabling reproducible evaluation of crash–restart behavior, including checkpoint-based recovery and log-based replay. For ease of evaluation, the artifact provides a container-only setup that does not require Kubernetes or external storage services. This simplified deployment preserves the functional behavior necessary to validate the correctness and performance claims presented in the paper.

The included mission program implements the patrol workload described in the paper: two drones patrol predefined waypoints and perform a battery-based handover when the first drone's battery level drops below a configured threshold. Crash scenarios can be introduced either manually or via the provided automation script, allowing systematic validation of recovery correctness and replay behavior.

Clone the code from the repo:

```bash
git clone https://github.com/AnTzikas/mavlink-logbased-ft
cd mavlink-logbased-ft/
```

> [NOTE]
> If you are working from a provided archive instead of cloning from GitHub, unzip the contents into a clean directory and navigate into it before proceeding.

You should see the following directory structure / files:

```text
.
|-- README.md
|-- LICENSE
|-- run_crash_scenarios.py
|-- cleanup.py
|-- experiment_logs/
|   |-- checkpoints/  
|   |-- logfiles/       
|   |-- wrapper_logs/   
|   |-- container_logs/ 
|-- src/                
|   |-- supervisor.py
|   |-- wrapper/
|       |-- __init__.py
|       |-- constants.py
|       |-- interaction_journal.py
|       |-- ipc_sem.py
|       |-- replay_buffer.py
|       |-- wrapper.py
|-- docker/             
|   |-- Dockerfile.mission
|   |-- Dockerfile.sitl
|   |-- compose/
|       |-- compose.mission.yaml
|       |-- compose.sitl-drones.yaml
|-- mission/        
    |-- mission.env
    |-- patrol_mission.py
    |-- patrol.waypoints
```

Before proceeding, ensure (i) the `Docker` engine is installed on the host machine, and that (ii) the `Docker Compose v2` plugin is available. You may then build the container for the mission program and the simulated drone:

```bash
docker build -t mission-container -f docker/Dockerfile.mission .
docker build -t ardupilot-sitl -f docker/Dockerfile.sitl .
```

Before running the checkpoint-based experiments, verify that checkpoint/restore via CRIU is supported by the host kernel:

```bash
docker run --rm --privileged mission-container criu check
```

If this check fails, the experiments can still be executed by disabling checkpoints via `CHECKPOINT=0` in `mission/mission.env`. This way, after a restart, the mission program will run the mission from the beginning in replay mode until it reaches the end of the input/output logs and then will resume interaction with the drones.

---

## Running experiments

To start an experiment, first launch two instances of the drone container to emulate the two drones used in the patrol program:

```bash
docker compose -f docker/compose/compose.sitl-drones.yaml up
```

Once the drones are ready (the autopilot is waiting for commands), you can start the mission container. This is configured to run with elevated privileges as CRIU requires additional kernel capabilities for process checkpoint/restore:

```bash
docker compose -f docker/compose/compose.mission.yaml up
```

The `mission/mission.env` file contains the environment variables required by the mission program and the supervisor, including experiment parameters such as the total number of laps and battery threshold. These can be adjusted as desired before starting the mission container.

Mission status/progress can be tracked by observing the console output of the drone container (SITL/Ardupilot output) and the mission container (mission program & wrapper output). For easier inspection, we recommend running each command in separate terminals. In addition, after each run, information about the execution is stored under the `experiment_logs/` directory: CRIU checkpoint images (`checkpoints/`), raw byte-level wrapper interaction logs (`logfiles/`), human-readable versions of the wrapper logs (`wrapper_logs/`) and mission/drone container stdout/stderr (`container_logs/`) which are produced only when running experiments using `run_crash_scenarios.py`.

Failures and restarts can be introduced at any point in time, by stopping and then restarting the mission container:

```bash
docker compose -f docker/compose/compose.mission.yaml kill
docker compose -f docker/compose/compose.mission.yaml up
```

Before running an experiment manually, ensure that logs from any previous run (checkpoint and interaction logs) have been removed:

```bash
python3 cleanup.py
```

To run tests in an automated way, use the `run_crash_scenarios.py` script, providing as arguments the points in time during mission execution where failures/restarts should be introduced (in seconds after the start of mission execution). For example, to run a test where failures occur at the 1st, 2nd and 3rd minute of execution:

```bash
python3 run_crash_scenarios.py 60 120 180
```

Upon completion, the script prints a summary of the run, including metrics such as total messages processed and messages replayed during recovery. The generated logs from the drone containers, the mission program and the wrapper are stored under the `experiment_logs/` directory.

To establish a reference, observe a failure-free execution (e.g., by running the script without any arguments), and then run scenarios that include failures. Also, you can then reproduce specific failure scenarios by running the script with the respective semantic trigger as a first argument:

```bash
python3 run_crash_scenarios.py after_send <lap> <waypoint>
python3 run_crash_scenarios.py after_rtl 
```

In the first case, the lap (1-3) and waypoint (1-6) where the failure should be triggered after sending the corresponding goto command to the drone, must be provided as additional arguments. In the second case, no additional arguments are required; the failure will be triggered after the mission program instructs the first drone to land and before the second drone is engaged to continue the patrol.

## License

Our full code (supervisor, wrapper, and mission program) is released as free open-source under the **Apache 2.0** license. Anyone can download, use, and change the code for both non-commercial and commercial purposes. The only requirement is to properly acknowledge the authors of the original code.

For the full legal text, please see the [LICENSE](LICENSE) file in the root directory.
