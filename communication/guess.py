import rospy
from ruamel.yaml import YAML


MAIN_YAML = '/root/rm/radar-detect/configs/main_config.yaml'
GUESS_YAML = '/root/rm/radar-detect/configs/guess.yaml'
        
class Guess:
    def __init__(self):
        with open(GUESS_YAML, 'r') as file:
            self.guess_yaml = self.yaml.load(file)
        with open(MAIN_YAML, 'r') as file:
            self.main_yaml = self.yaml.load(file)

        self.color = self.main_yaml['global']['my_color']
        self.point = self.guess_yaml[self.color]
    


























