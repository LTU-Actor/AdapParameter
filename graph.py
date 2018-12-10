#! /usr/bin/env python3

# from this repo
import build

# external
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import time
import argparse


if (__name__ == '__main__'):
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '-r',
        '--run',
        dest='run',
        type=str,
        required=True,
        help='Automatically start a client')
    args = argparser.parse_args()

    build.build()
    sns.set(style='darkgrid')
    try:
        server = build.launch_launchfile('server')
        time.sleep(2) # TODO: remove once client is able to wait
        node = build.launch_direct(args.run)
        build.waiton(node)
        df = pd.read_csv(node.stdout, sep=',')
        cols = df.columns
        df['point'] = df.index
        df = df.melt(value_vars=cols, id_vars=['point'])
        sns.lineplot(x="point", y="value", hue='variable', data=df)
        plt.show()
    finally:
        build.kill_node(server)
        build.kill_node(node)
