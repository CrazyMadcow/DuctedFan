from Model.model import Model
from Simulation.simulation import Sim
from Simulation.visualization import Plot
import torch
import numpy as np
import matplotlib.pyplot as plt
import time


if __name__ == '__main__':

    model       =   Model()

    sim         =   Sim(model)

    plot        =   Plot(object='ductedfan')
    for i in range(len(sim.t)):

        sim.XHist[:, i] = sim.X.T
        sim.UHist[:, i] = sim.U.T

        TCmd = 0.5*( -2.5*( -10.0 -sim.X[2,0]) + 9.81)
        delCmd = 0.5*(0 - sim.X[11,0])

        sim.U[0,0]  =   TCmd
        sim.U[1,0]  =   TCmd
        sim.U[2,0]  =   delCmd
        sim.U[3,0]  =   delCmd
        sim.U[4,0]  =   delCmd
        sim.U[5,0]  =   delCmd

        Xdot = model.Dynamics(sim.X, sim.U)

        sim.X = sim.X + Xdot * sim.dt

        plot.UpdatePlot(sim.X[0,0].cpu().numpy(),sim.X[1,0].cpu().numpy(),sim.X[2,0].cpu().numpy(),sim.X[6,0].cpu().numpy(),sim.X[7,0].cpu().numpy(),sim.X[8,0].cpu().numpy())

    plot.EndPlot()

    # plt.plot(sim.t.cpu(), sim.XHist[2,:].cpu())
    # plt.plot(sim.t.cpu(), sim.XHist[0, :].cpu())
    # plt.plot(sim.t.cpu(), sim.XHist[1, :].cpu())
    # plt.show()