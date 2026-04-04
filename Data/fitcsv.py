import matplotlib.pylab as plt
import numpy as np
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--plot', action=argparse.BooleanOptionalAction, default=True)
parser.add_argument('-f', '--filename', help="Name of the CSV file to process", default="TrajectoryData.csv" )
args = parser.parse_args()

filename = args.filename

first_loop = True

while(True):
    print("Enter -1 to exit this program. Changes to data can be made while running")
    new_range = float(input("Input a new range to predict angle and speed [inches]: "))

    if new_range == -1:
        break

    data = np.loadtxt(filename, delimiter=',', skiprows=1)  # skip the first row which holds metadata

    range_data = data[:, 0]
    angle_data = data[:, 1]
    speed_data = data[:, 2]

    poly_angle = 6  # degree of polynomial for the angle (0 = constant, 1 = linear, 2 = parabola, 3 = cubic)
    poly_speed = 5  # degree of polynomial for the speed (0 = constant, 1 = linear, 2 = parabola, 3 = cubic)

    angle_coeff = np.polyfit(range_data, angle_data, poly_angle)
    speed_coeff = np.polyfit(range_data, speed_data, poly_speed)  # fit a polynomial to the available data


    new_angle = np.polyval(angle_coeff, new_range)  # evaluate the polynomial at the requested range point
    new_speed = np.polyval(speed_coeff, new_range)

    print("Predicted Range, Angle, Speed")
    print(f"{new_range:0.1f}, {new_angle:0.1f}, {new_speed:0.1f}\n")
    print(f"Angle polynomial coefficients {angle_coeff}\n")
    print(f"Speed polynomial coefficients {speed_coeff}\n")

    plot_range = np.linspace(np.min(range_data), np.max(range_data))  # make a scale of ranges to create a plot
    plot_angle = np.polyval(angle_coeff, plot_range)
    plot_speed = np.polyval(speed_coeff, plot_range)

    if args.plot:
        if first_loop:
            fig, ax = plt.subplots(nrows=1, ncols=2, figsize=(9,5), tight_layout=True)
        else:
            ax[0].clear()
            ax[1].clear()
        first_loop = False
        plt.sca(ax[0])
        plt.plot(plot_range, plot_angle, ':', label='Polynomial Fit')
        plt.plot(range_data, angle_data, 'k.', label="Test Data")
        plt.plot(new_range, new_angle, 'r*', label="Requested Point")
        plt.grid(True)
        plt.xlabel("Range [inches]")
        plt.ylabel("Angle [degrees]")
        plt.legend()

        plt.sca(ax[1])
        plt.plot(plot_range, plot_speed, ':', label='Polynomial Fit')
        plt.plot(range_data, speed_data, 'k.', label="Test Data")
        plt.plot(new_range, new_speed, 'r*', label="Requested Point")
        plt.grid(True)
        plt.xlabel("Range [inches]")
        plt.ylabel("Speed [ticks/sec]")
        plt.legend()

        plt.show(block=False)