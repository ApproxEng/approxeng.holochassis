from math import sin, cos
from threading import Thread

import matplotlib.patches as patches
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def show_plot(simulation, x1=-1000, y1=-1000, x2=1000, y2=1000, update_interval=10):
    """
    Show a plot bound to the specified simulation. Install dependencies with 'pip install matplotlib'
    
    :param simulation: 
        A :class:`approxeng.holochassis.simulation.Simulation` to watch for pose changes
    :param x1: 
        Lower bound on x axis in mm, defaults to -1000
    :param y1: 
        Lower bound on y axis in mm, defaults to -1000
    :param x2: 
        Upper bound on x axis in mm, defaults to 1000
    :param y2: 
        Upper bound on y axis in mm, defaults to 1000
    :param update_interval: 
        Update interval for the plot animation
    """

    class PlotThread(Thread):
        def __init__(self):
            super(PlotThread, self).__init__(name='PlotThread')
            self.arrow = None
            self.circle = None

        def run(self):
            fig = plt.figure(figsize=(7, 7))
            ax = fig.add_axes([0.05, 0.05, 0.9, 0.9], aspect='equal')
            ax.set_xlim(x1, x2)
            # ax.set_xticks([])
            ax.set_ylim(y1, y2)
            # ax.set_yticks([])
            ax.set_frame_on(True)
            plt.rc('grid', linestyle="-", color='black')
            plt.grid(True)
            colour = (0.4, 1.0, 0.2, 0.5)
            edge_colour = (0.3, 0.7, 0.1, 1.0)

            arrow_params = {'length_includes_head': True,
                            'head_starts_at_zero': False,
                            'width': 1,
                            'head_width': 40,
                            'head_length': 20,
                            'fc': colour,
                            'ec': edge_colour}
            circle = None

            def update(frame_number):
                pose = simulation.pose
                if self.arrow is not None:
                    self.arrow.remove()
                    self.circle.remove()
                # ax.clear()
                # plt.arrow(pose.position.x, pose.position.y, 100 * sin(pose.orientation), 100 * cos(pose.orientation),
                #         width=40, head_width=100, head_length=40, fc="none", ec="r", alpha=0.5, **arrow_params)
                unit_offset_x = sin(pose.orientation)
                unit_offset_y = cos(pose.orientation)
                self.arrow = ax.add_patch(
                    patches.FancyArrow(pose.position.x + unit_offset_x * 50, pose.position.y + unit_offset_y * 50,
                                       25 * sin(pose.orientation),
                                       25 * cos(pose.orientation),
                                       **arrow_params)
                )
                ax.add_artist(
                    plt.Circle((pose.position.x, pose.position.y), 5, color=(0.4, 1.0, 0.2, 0.5), ec=edge_colour))

                self.circle = plt.Circle((pose.position.x, pose.position.y), 50, color=(0.4, 1.0, 0.2, 0.5),
                                         ec=edge_colour)
                ax.add_artist(self.circle)

            animation = FuncAnimation(fig, update, interval=update_interval)
            plt.show()

    return PlotThread()
