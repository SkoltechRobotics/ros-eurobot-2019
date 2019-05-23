import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--config",
                    help="path to config yaml file",
                   default="/home/alexey/CatkinWorkspace/src/ros-eurobot-2019/eurobot_camera/configs/calibration.yaml")
parser.add_argument("-t", "--template",
                    help="path to field template file",
                    default="/home/alexey/Desktop/field.png")
_args = parser.parse_args()
args = _args
print args.config
