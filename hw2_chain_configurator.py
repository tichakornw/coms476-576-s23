#!/usr/bin/env python
import sys, argparse
import rospy


def publish(config, W, L, D):
    """Publish the configuration of a 2D kinematic chain A_1, ..., A_m

    @type config: a list [theta_1, ..., theta_m] where theta_1 represents the angle between A_1 and the x-axis,
        and for each i such that 1 < i <= m, \theta_i represents the angle between A_i and A_{i-1}.
    @type W: float, representing the width of each link
    @type L: float, representing the length of each link
    @type D: float, the distance between the two points of attachment on each link
    """
    # TODO: Implement this function
    raise NotImplementedError


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description="Display the arrangement of 2D kinematic chain"
    )
    parser.add_argument(
        "config",
        metavar="config",
        type=float,
        nargs="+",
        help="chain configuration theta_1, ..., theta_m",
    )
    parser.add_argument(
        "-W", type=float, required=True, dest="W", help="the width of each link"
    )
    parser.add_argument(
        "-L", type=float, required=True, dest="L", help="the length of each link"
    )
    parser.add_argument(
        "-D",
        type=float,
        required=True,
        dest="D",
        help="the distance between the two points of attachment",
    )

    args = parser.parse_args(sys.argv[1:])

    return args


if __name__ == "__main__":
    args = parse_args()

    try:
        publish(args.config, args.W, args.L, args.D)
    except rospy.ROSInterruptException:
        pass
