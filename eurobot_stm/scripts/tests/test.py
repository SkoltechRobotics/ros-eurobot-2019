import sys
import argparse
from manipulator import Manipulator


if __name__== '__main__':

    man = Manipulator()

    parser = argparse.ArgumentParser()
    parser.add_argument("-s", "--size",
                        help="small or big")
    parser.add_argument("-a", "--action",
                        help="action to make",)
    args = parser.parse_args()

    if args.size == 0:
        if args.action == 0:
            man.calibrate_small()
        elif args.action == 1:
            man.collect_small()
        elif args.action == 2:
            man.release_small()
        else:
            exit("WRONG COMMAND")
    elif args.size == 1:
        if args.action == 0:
            man.calibrate_big()
        elif args.action == 1:
            man.collect_big()
        elif args.action == 2:
            man.release_big()
        else:
            exit("WRONG COMMAND")
    else:
        exit("WRONG COMMAND")



