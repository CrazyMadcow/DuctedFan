import torch
import math

def InitDuctedFan(model):
    model.nx        =   12
    model.nu        =   6
    model.m         =   1
    model.g         =   9.81
    model.Ix        =   0.01
    model.Iy        =   0.01
    model.Iz        =   0.02
    model.f         =   StateDynamicsDuctedFan
    model.G         =   ControlDynamicsDuctedFan
    model.platform  =   "DuctedFan"


def StateDynamicsDuctedFan(model, X):
    f           =   torch.zeros((X.shape[0],X.shape[1]), device = X.device)

    m           =   model.m
    g           =   model.g
    Ix          =   model.Ix
    Iy          =   model.Iy
    Iz          =   model.Iz

    N           =   X[0, :]
    E           =   X[1, :]
    D           =   X[2, :]
    u           =   X[3, :]
    v           =   X[4, :]
    w           =   X[5, :]


    roll        =   X[6, :]
    pitch       =   X[7, :]
    yaw         =   X[8, :]
    p           =   X[9, :]
    q           =   X[10, :]
    r           =   X[11, :]

    uvw         =   torch.tensor([[u],[v],[w]])
    pqr         =   torch.tensor([[p],[q],[r]])

    velNed      =   torch.matmul(EulerDCM(roll, pitch, yaw),uvw)
    eulerDot    =   torch.matmul(PqrDCM(p, q, r), pqr)

    f[0,:]      =   velNed[0,0]
    f[1,:]      =   velNed[1,0]
    f[2,:]      =   velNed[2,0]

    f[3,:]      =   v*r - w*q
    f[4,:]      =   w*p - u*r
    f[5,:]      =   u*q - v*p

    f[6,:]      =   eulerDot[0, 0]
    f[7,:]      =   eulerDot[1, 0]
    f[8,:]      =   eulerDot[2, 0]

    f[9,:]      =   ((Iy - Iz) / Ix) * q * r
    f[10,:]     =   ((Iz - Ix) / Iy) * r * p
    f[11,:]     =   ((Ix - Iy) / Iz) * p * q

    return f


def ControlDynamicsDuctedFan(model, X, U):
    G           =   torch.zeros((X.shape[0],X.shape[1]), device = X.device)

    m           =   model.m
    g           =   model.g
    Ix          =   model.Ix
    Iy          =   model.Iy
    Iz          =   model.Iz

    roll        =   X[6, :]
    pitch       =   X[7, :]
    yaw         =   X[8, :]

    Fx, Fy, Fz, L, M, N     =   QuadrotorForceMomentGen(model, X, U)

    G[3,:]      =   Fx / m
    G[4,:]      =   Fy / m
    G[5,:]      =   Fz / m

    G[9,:]      =   L / Ix
    G[10,:]     =   M / Iy
    G[11,:]     =   N / Iz

    return G

def QuadrotorForceMomentGen(model, X, U):
    m           =   model.m
    g           =   model.g
    Ix          =   model.Ix
    Iy          =   model.Iy
    Iz          =   model.Iz

    N           =   X[0, :]
    E           =   X[1, :]
    D           =   X[2, :]
    u           =   X[3, :]
    v           =   X[4, :]
    w           =   X[5, :]

    roll        =   X[6, :]
    pitch       =   X[7, :]
    yaw         =   X[8, :]
    p           =   X[9, :]
    q           =   X[10, :]
    r           =   X[11, :]

    Fcmd        =   U[0,:]
    Lcmd        =   U[1, :]
    Mcmd        =   U[2, :]
    Ncmd        =   U[3, :]

    # gravity zone
    Grav        =   torch.tensor([[0],[0],[g]])
    R           =   EulerDCM(roll, pitch, yaw)
    FGrav       =   torch.matmul(R.T, m*Grav)
    FxGrav      =   FGrav[0, 0]
    FyGrav      =   FGrav[1, 0]
    FzGrav      =   FGrav[2, 0]
    LGrav       =   0
    MGrav       =   0
    NGrav       =   0

    # Aerodynamic zone
    CD          =   0
    rho         =   1.25
    Sx          =   0.1
    Sy          =   0.1
    Sz          =   0.2
    V           =   torch.norm(X[3:5,:]) + 0.001
    Drag        =   0.5*CD*rho*torch.pow(V,2)
    FxAero      =   (u/V)*Drag*Sx
    FyAero      =   (v / V) * Drag * Sx
    FzAero      =   (w / V) * Drag * Sx
    LAero       =   0
    MAero       =   0
    NAero       =   0

    # Thrust zone
    FxThrust    =   0
    FyThrust    =   0
    FzThrust    =   Fcmd
    LThrust     =   Lcmd
    MThrust     =   Mcmd
    NThrust     =   Ncmd

    Fx          =   FxGrav + FxAero + FxThrust
    Fy          =   FyGrav + FyAero + FyThrust
    Fz          =   FzGrav + FzAero + FzThrust

    L           =   LGrav + LAero + LThrust
    M           =   MGrav + MAero + MThrust
    N           =   NGrav + NAero + NThrust

    return Fx, Fy, Fz, L, M, N

def EulerDCM(roll, pitch, yaw):
    R       =   torch.zeros((3,3))

    cr      =   torch.cos(roll)
    sr      =   torch.sin(roll)
    cp      =   torch.cos(pitch)
    sp      =   torch.sin(pitch)
    cy      =   torch.cos(yaw)
    sy      =   torch.sin(yaw)

    R[0, 0] = cy*cp
    R[0, 1] = -sy*cr + cy*sp*sr
    R[0, 2] = sy*sr + cy*sp*cr

    R[1, 0] = sy*cp
    R[1, 1] = cy*cr + sy*sp*sr
    R[1, 2] = -cy*sr + sy*sp*cr

    R[2, 0] = -sp
    R[2, 1] = cp*sr
    R[2, 2] = cp*cr

    return R


def PqrDCM(roll, pitch, yaw):
    W       =   torch.zeros((3,3))

    W[0,0]  =   1
    W[0,1]  =   torch.sin(roll)*torch.tan(pitch)
    W[0,2]  =   torch.cos(roll)*torch.tan(pitch)

    W[1,0]  =   0
    W[1,1]  =   torch.cos(roll)
    W[1,2]  =   -torch.sin(roll)

    W[2,0]  =   0
    W[2,1]  =   torch.sin(roll)/torch.cos(pitch)
    W[2,2]  =   torch.cos(roll)/torch.cos(pitch)

    return W