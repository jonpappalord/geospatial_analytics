import traci
import sys
import os


def init_traci(config_file_path):
    
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")

    #Configuration
    sumo_binary = os.environ['SUMO_HOME']+"/bin/sumo"

    sumo_cmd = [sumo_binary, "-c", config_file_path]

    traci.start(sumo_cmd)
    traci.simulationStep()