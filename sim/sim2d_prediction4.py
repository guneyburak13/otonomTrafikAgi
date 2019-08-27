import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
import time
import random


def sim_run4(options, KalmanFilter, light_state):
    start = time.clock()
    # Simulator Options
    FIG_SIZE = options['FIG_SIZE'] # [Width, Height]
    ALLOW_SPEEDING = options['ALLOW_SPEEDING']


    kalman_filter = KalmanFilter()

    def a():
        a.first_prediction = True
        a.time = 0
        a.u_pedal = 5
        a.u_pedal2 = 5
        a.u_pedal3 = 0
        a.change = 0
        a.u_steer2 = 0
        a.line = False
        a.signal = None
        a.x01 = 0
        a.x02 = 0
    a()




    def physics(t0, dt, state, u_pedal):

        a.u_pedal = u_pedal
        if len(state) == 0:
            x0 = 55
            y0 = 3
            v0 = 5
            theta0 = 0
            theta_dot0 = 0
        else:
            x0 = state[-1][0]
            y0 = state[-1][1]
            v0 = state[-1][2]
            theta0 = state[-1][3]
            theta_dot0 = state[-1][4]

        a.u_steer = 0
        a.x01 = x0
        if x0 >= 86 and a.signal is False:
            a.u_pedal3 = 1

        x1 = v0*np.cos(theta0)*dt + x0
        y1 = v0*np.sin(theta0)*dt + y0
        v1 = (-v0 + 1.0*u_pedal)/0.8*dt + v0
        theta1 = theta_dot0*dt + theta0
        theta_dot1 = a.u_steer

        return [x1, y1, v1, theta1, theta_dot1]

    def physics2(t0, dt, state2, u_pedal2):

        a.u_pedal2 = u_pedal2

        if len(state2) == 0:
            x0 = 55
            y0 = 7
            v0 = 5
            theta0 = 0
            theta_dot0 = 0
        else:
            x0 = state2[-1][0]
            y0 = state2[-1][1]
            v0 = state2[-1][2]
            theta0 = state2[-1][3]
            theta_dot0 = state2[-1][4]

        a.u_steer2 = 0
        a.x02 = x0
        if x0 >= 84:
            a.line = True

        if x0 >= 86 and a.signal is False:
            a.u_pedal3 = 1


        x1 = v0*np.cos(theta0)*dt + x0
        y1 = v0*np.sin(theta0)*dt + y0
        v1 = (-v0 + 1.0*u_pedal2)/0.8*dt + v0
        theta1 = theta_dot0*dt + theta0
        theta_dot1 = a.u_steer2

        return [x1, y1, v1, theta1, theta_dot1]

    def physics3(t0, dt, state3, u_pedal3):

        a.u_pedal3 = u_pedal3

        if len(state3) == 0:
            x0 = 86
            y0 = -2
            v0 = 1
            theta0 = 1.57
            theta_dot0 = 0
        else:
            x0 = state3[-1][0]
            y0 = state3[-1][1]
            v0 = state3[-1][2]
            theta0 = state3[-1][3]
            theta_dot0 = state3[-1][4]

        a.u_steer3 = 0
        if y0 >= 4 and a.signal is True:
            a.u_pedal = 5
        if y0 >= 8 and a.signal is True:
            a.u_pedal2 = 5

        x1 = v0 * np.cos(theta0) * dt + x0
        y1 = v0 * np.sin(theta0) * dt + y0
        v1 = (-v0 + 1.0 * u_pedal3) / 0.8 * dt + v0
        theta1 = theta_dot0 * dt + theta0
        theta_dot1 = a.u_steer3

        return [x1, y1, v1, theta1, theta_dot1]

    state = []
    state2 = []
    state3 = []
    est_data_t = []
    x_est_data = []
    noise_data = []
    noise_data2 = []
    noise_data3 = []
    predict_x_loc = []
    light_time = [4.5, 4.7, 4.8, 5.0, 5.2]
    light_time = light_time[light_state]


    def update_plot(num):

        a()
        a.time = int(t[num])
        # Car.
        car_loc = [state[num][0], state[num][1]]
        car_ang = state[num][3]
        car_cos = np.cos(car_ang)
        car_sin = np.sin(car_ang)
        car.set_data([car_loc[0], car_loc[0] + 2 * car_cos], [car_loc[1], car_loc[1] + 2 * car_sin])
        est.set_data([x_est_data[num][0]], [x_est_data[num][1]])

        car2_loc = [state2[num][0], state2[num][1]]
        car2_ang = state2[num][3]
        car2_cos = np.cos(car2_ang)
        car2_sin = np.sin(car2_ang)
        car2.set_data([car2_loc[0], car2_loc[0] + 2 * car2_cos], [car2_loc[1], car2_loc[1] + 2 * car2_sin])
        est.set_data([x_est_data[num][0]], [x_est_data[num][1]])

        car3_loc = [state3[num][0], state3[num][1]]
        car3_ang = state3[num][3]
        car3_cos = np.cos(car3_ang)
        car3_sin = np.sin(car3_ang)
        car3.set_data([car3_loc[0], car3_loc[0] + 2 * car3_cos], [car3_loc[1], car3_loc[1] + 2 * car3_sin])
        est.set_data([x_est_data[num][0]], [x_est_data[num][1]])

        return car, car2, car3

    t = np.linspace(0.0, 100, 1001)
    dt = 0.1
    a()
    b = random.randint(0, 1)
    if b == 0:
        a.signal = False
        a.u_pedal3 = 0
    else:
        a.signal = True
        a.u_pedal3 = 1

    print("signal: ", a.signal)
    for t0 in t:

        state += [physics(t0, dt, state, a.u_pedal)]
        state2 += [physics2(t0, dt, state2, a.u_pedal2)]
        state3 += [physics3(t0, dt, state3, a.u_pedal3)]
        if True:  # t0%1.0 == 0.0:
            est_data_t += [t0]
            # Measure car location.
            state_with_noise = []
            state_with_noise2 = []
            state_with_noise3 = []
            state_with_noise += [state[-1][0] + (np.random.rand(1)[0] - 0.5) * 0.5]
            state_with_noise += [state[-1][1] + (np.random.rand(1)[0] - 0.5) * 0.5]
            state_with_noise2 += [state2[-1][0] + (np.random.rand(1)[0] - 0.5) * 0.5]
            state_with_noise2 += [state2[-1][1] + (np.random.rand(1)[0] - 0.5) * 0.5]
            state_with_noise3 += [state3[-1][0] + (np.random.rand(1)[0] - 0.5) * 0.5]
            state_with_noise3 += [state3[-1][1] + (np.random.rand(1)[0] - 0.5) * 0.5]
            noise_data += [state_with_noise]
            noise_data2 += [state_with_noise2]
            noise_data3 += [state_with_noise3]
            if t0 == 0.0:
                x_est_data += [[0, 0]]
                continue
            kalman_filter.predict(dt)
            x_est_data += [kalman_filter.measure_and_update(state_with_noise, dt)]
            x_est_data += [kalman_filter.measure_and_update(state_with_noise2, dt)]
            x_est_data += [kalman_filter.measure_and_update(state_with_noise3, dt)]
            if t0 >= light_time and a.first_prediction:
                if not ALLOW_SPEEDING:
                    predict_x_loc = kalman_filter.predict_red_light(95)

                else:
                    predict_x_loc = kalman_filter.predict_red_light_speed(95)

                """
                if predict_x_loc[0] is False:
                    a.u_pedal = 0
                    a.u_pedal2 = 0
                elif ALLOW_SPEEDING:
                    a.u_pedal = 10
                """
                if a.signal is True and a.x01 >= 75 and a.x02 >= 75:
                    a.u_pedal = 0
                    a.u_pedal2 = 0
                    print("x01: ", a.x02)
                    print("x02: ", a.x02)

                elif a.signal is False and a.x01 >= 75 and a.x02 >= 75:
                    a.u_pedal = 10
                    a.u_pedal2 = 10

                a.first_prediction = False







    ###################
    # SIMULATOR DISPLAY

    # Total Figure
    fig = plt.figure(figsize=(FIG_SIZE[0], FIG_SIZE[1]))
    gs = gridspec.GridSpec(10, 10)

    # Elevator plot settings.
    ax = fig.add_subplot(gs[:10, :10])

    plt.xlim(60, 110)
    ax.set_ylim([-20, 30])

    plt.xticks([])
    plt.yticks([])
    plt.title('Traffic Simulation with Autonomous Cars')

    # Main plot info.
    car, = ax.plot([15, 12], [5, 3], 'g-', linewidth=5)
    car2, = ax.plot([15, 12], [5, 3], 'r-', linewidth=5)
    car3, = ax.plot([15, 12], [5, 3], 'b-', linewidth=5)
    #light, = ax.plot([94, 94], [4, 2], 'g-', linewidth=3)
    #light2, = ax.plot([94, 94], [8, 6], 'g-', linewidth=3)
    est, = ax.plot([], [], 'ks', markersize=5, fillstyle='full', linewidth=4)
    est_light, = ax.plot([10, 10], [17, 18], 'g--', linewidth=1)
    meas, = ax.plot([], [], 'gs', markersize=30, fillstyle='none', linewidth=4)

    # First section.
    ax.plot([1, 1], [9, 1], 'k-')
    ax.plot([1, 87], [9, 9], 'k-')
    ax.plot([1, 83], [5, 5], 'k--')
    ax.plot([1, 95], [1, 1], 'k-')
    ax.plot([87, 87], [9, 1], 'k--')
    ax.plot([85, 86], [2, 2], 'k-')
    ax.plot([85, 86], [3, 3], 'k-')
    ax.plot([85, 86], [4, 4], 'k-')
    ax.plot([85, 86], [5, 5], 'k-')
    ax.plot([85, 86], [6, 6], 'k-')
    ax.plot([85, 86], [7, 7], 'k-')
    ax.plot([85, 86], [8, 8], 'k-')

    # First intersection.
    ax.plot([95, 110], [9, 9], 'k-')
    ax.plot([97, 110], [5, 5], 'k--')
    ax.plot([95, 110], [1, 1], 'k-')
    ax.plot([95, 95], [9, 1], 'k--')

    # Second section.
    ax.plot([87, 87], [9, 87], 'k-')
    ax.plot([91, 91], [11, 85], 'k--')
    ax.plot([95, 95], [9, 87], 'k-')
    ax.plot([87, 95], [9, 9], 'k--')

    # second intersection.
    ax.plot([87, 95], [87, 87], 'k--')
    ax.plot([87, 87], [87, 95], 'k--')
    ax.plot([87, 95], [95, 95], 'k--')

    ax.plot([87, 87], [95, 110], 'k-')
    ax.plot([91, 91], [97, 110], 'k--')
    ax.plot([95, 95], [87, 110], 'k-')
    ax.plot([92, 94], [94, 94], 'g-', linewidth=3)

    # Final Section.
    ax.plot([87, 2], [87, 87], 'k-')
    ax.plot([87, 2], [91, 91], 'k--')
    ax.plot([87, 2], [95, 95], 'k-')
    ax.plot([2, 2], [95, 87], 'k-')

    #-----------------------------------------------




    print("Compute Time: ", round(time.clock() - start, 3), "seconds.")
    # Animation.
    car_ani = animation.FuncAnimation(fig, update_plot, frames=range(1,len(t)),
                                    interval=100, repeat=False, blit=False)
    #car_ani.save('lines.mp4')

    plt.show()
