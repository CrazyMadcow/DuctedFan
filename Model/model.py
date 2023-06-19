import torch
from Model import cartpole
from Model import ductedfan
import numpy as np


class Model():

    def __init__(self, platform="DuctedFan"):
        try:
            eval(platform.lower() + '.Init' + platform + '(self)')
            print("Model Initialized! - " + platform)
        except:
            print("Wrong!")
            pass

    def Dynamics(self, X, U):
        # XDot        =   torch.zeros((X.shape[0],X.shape[1]), device = X.device)

        XDot = self.f(self, X) + self.G(self,X,U)

        return XDot

