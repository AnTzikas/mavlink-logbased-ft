import subprocess

def run_cmd(cmd, detach=False, log_file=None):
    if detach:
        # Open the file in write mode
        out = open(log_file, 'a') if log_file else None
        # Start the process and send output to the file
        return subprocess.Popen(cmd, shell=True, stdout=out, stderr=out)
    return subprocess.run(cmd, shell=True, check=True)


def main():
    
    #Cleanup old logfiles
    run_cmd("rm -rf experiment_logs/container_logs/*")
    run_cmd("rm -rf experiment_logs/checkpoint/*")
    run_cmd("rm -rf experiment_logs/logfiles/*")
    run_cmd("rm -rf experiment_logs/mission_logs/*")
    run_cmd("rm -rf experiment_logs/wrapper_logs/*")


if __name__ == "__main__":
    main()