"""
Utility functions
"""
import math
import numpy as np
import pickle

"""
Unit convertions
"""
def rad_of_deg(d): return d/180.*math.pi

def deg_of_rad(r): return r*180./math.pi


"""
Misc
"""
def save_trajectory(time, X, U, desc, filename):
    with open(filename, "wb") as f:
        pickle.dump([time, X, U, desc], f)

def load_trajectory(filename):
    with open(filename, "rb") as f:
        time, X, U, desc = pickle.load(f)
    return time, X, U, desc


''' input test vectors '''

def make_random_pulses(dt, size, min_nperiod=1, max_nperiod=10, min_intensity=-1, max_intensity=1.):
    ''' make a vector of pulses of randon duration and intensities '''
    npulses = size/max_nperiod*2
    durations = np.random.random_integers(low=min_nperiod, high=max_nperiod, size=npulses)
    intensities =  np.random.uniform(low=min_intensity, high=max_intensity, size=npulses)
    pulses = []
    for duration, intensitie in zip(durations, intensities):
        pulses += [intensitie for i in range(duration)]
    pulses = np.array(pulses)
    time = np.linspace(0, dt*len(pulses), len(pulses))
    return time, pulses

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1
def sine_sweep(t, omega=2, domega=0.5, domega1=0.5): return math.sin(omega*(1-domega1*math.sin(domega*t))*t)


def random_input_vec(time): return np.random.uniform(low=-1.0, high=1.0, size=len(time))
def step_input_vec(time, a0=-1, a1=1, dt=4, t0=0): return [step(t, a0, a1, dt, t0) for t in time]
def sine_input_vec(time): return np.sin(time)
def sawtooth_input_vec(time): return scipy.signal.sawtooth(time)
def sine_swipe_input_vec(time): return [sine_sweep(t) for t in time]



"""
Plotting
"""
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

my_title_spec = {'color'    : 'k', 'fontsize'   : 20 }

def save_if(filename, dpi=80):
    if filename: matplotlib.pyplot.savefig(filename, dpi=dpi)

def prepare_fig(fig=None, window_title=None, figsize=(20.48, 10.24), margins=None):
    if fig == None:
        fig = plt.figure(figsize=figsize)
    #else:
    #    plt.figure(fig.number)
    if margins:
        left, bottom, right, top, wspace, hspace = margins
        fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top,
                            hspace=hspace, wspace=wspace)
    if window_title:
         fig.canvas.set_window_title(window_title)
    return fig

def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None):
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab:
        ax.xaxis.set_label_text(xlab)
    if ylab:
        ax.yaxis.set_label_text(ylab)
    if title:
        ax.set_title(title, my_title_spec)
    if legend <> None:
        ax.legend(legend, loc='best')
    if xlim <> None:
        ax.set_xlim(xlim[0], xlim[1])
    if ylim <> None:
        ax.set_ylim(ylim[0], ylim[1])

def plot_in_grid(time, plots, ncol, figure=None, window_title="None", legend=None, filename=None,
                 margins=(0.04, 0.08, 0.93, 0.96, 0.20, 0.34)):
    nrow = math.ceil(len(plots)/float(ncol))
    figsize = (10.24*ncol, 2.56*nrow)
    figure = prepare_fig(figure, window_title, figsize=figsize, margins=margins) 
#    pdb.set_trace()
    for i, (title, ylab, data) in enumerate(plots):
        ax = figure.add_subplot(nrow, ncol, i+1)
        ax.plot(time, data)
        decorate(ax, title=title, ylab=ylab)
    if legend<>None:
        ax.legend(legend, loc='best')
    save_if(filename)
    return figure
