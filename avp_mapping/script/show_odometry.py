import argparse
import numpy as np
import matplotlib.pyplot as plt
import os



def drawOdometry(odometry_file):
    fp = open(odometry_file,'r')
    odometry_content =  [l.split(',') for l in fp.readlines()]
    x = []
    y = []
    for row in  odometry_content :
        x.append( float(row[4]))
        y.append(float(row[5]))
    x = np.array(x)
    y = np.array(y)
    plt.plot ( x,y,'-',alpha=1.0,color=(0, 0.9490,0),linewidth=1.2 , label = 'odometry')

def main():


    parser = argparse.ArgumentParser(description='Show Odoemtry')
    parser.add_argument('-t', dest='odometry',required=False,default='./odometry.txt',help='odometry file')
    args = parser.parse_args()

    plt.rcParams['axes.facecolor'] = (0.0156, 0.1019, 0.231)
    plt.figure()

    if os.path.exists(args.odometry):
        drawOdometry( args.odometry)

    plt.axis('equal')
    leg = plt.legend()
    for text in leg.get_texts():
        plt.setp(text, color=(0.9490, 0.9490, 0.9490))

    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
