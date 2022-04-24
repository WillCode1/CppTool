import argparse
import numpy as np
import matplotlib.pyplot as plt
import os



def drawOdometry(odometry_file):
    data = np.genfromtxt(odometry_file,usecols=(0,1,2))
    plt.plot ( data[:,0],data[:,1],'-',alpha=1.0,color=(0, 0.9490,0),linewidth=1.2 , label = 'odometry')

def drawTimeDuration(log_file):
    data = np.genfromtxt(log_file,usecols=(0,1,2))
    index = np.linspace(0,len( data) ,num= len(data))
    plt.plot ( index ,data[:,2],'-',alpha=1.0,color=(0, 0.9490,0),linewidth=1.2 , label = 'time duration/frame')



def main():


    parser = argparse.ArgumentParser(description='Show Odoemtry')
    parser.add_argument('-t', dest='result',required=True,default='./odometry.txt',help='result file of hpa mapping ')
    parser.add_argument('-a',dest='show_time',required=False,default=False,help='')
    args = parser.parse_args()

    color = (0,0,0)

    if not args.show_time:
        plt.rcParams['axes.facecolor'] = (0.0156, 0.1019, 0.231)
        color = (0.9490, 0.9490, 0.9490)

    plt.figure()

    if not args.show_time and os.path.exists(args.result):
        drawOdometry( args.result)
        plt.axis('equal')

    if args.show_time and os.path.exists(args.result):
        drawTimeDuration(args.result)


    leg = plt.legend()
    for text in leg.get_texts():
        plt.setp(text, color=color)

    plt.grid()
    plt.show()


if __name__ == '__main__':
    main()
