import numpy as np
from cmaes import CMA
import math
from scipy.stats import uniform
import matplotlib.pyplot as plt

import matplotlib.patches as patches
import time
from matplotlib.transforms import Affine2D

from PIL import Image
import os
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C


def HipPosition(X_body,Y_body): 
    N = int(X_body.size) + 1
    X_body = np.insert(X_body, 0, x_start)
    Y_body = np.insert(Y_body, 0, y_start)
    body_orientation = np.zeros((2,N))
    Rotation = [None]*N
    Rotation[0] = np.eye(2)
    LF = np.zeros((2,N))
    LH = np.zeros((2,N))
    RH = np.zeros((2,N))
    RF = np.zeros((2,N))
    LF[:, 0] = np.array([x_start+halfbody_x, y_start+halfbody_y])
    LH[:, 0] = np.array([x_start-halfbody_x, y_start+halfbody_y])
    RH[:, 0] = np.array([x_start-halfbody_x, y_start-halfbody_y])
    RF[:, 0] = np.array([x_start+halfbody_x, y_start-halfbody_y])

    for i in range(1,N):
        length = np.sqrt((X_body[i]-X_body[i-1])**2 + (Y_body[i]-Y_body[i-1])**2)
        body_orientation[0,i] = np.array([X_body[i]-X_body[i-1]])/length  # cos
        body_orientation[1,i] = np.array([Y_body[i]-Y_body[i-1]])/length  # sin
        body_orientation[:,i] = Rotation[i-1].T @ body_orientation[:,i]
        Rotation[i] = np.array([[body_orientation[0,i], -body_orientation[1,i]], [body_orientation[1,i], body_orientation[0,i]]]) @ Rotation[i-1]
        LF[:,i] = Rotation[i] @ np.array([+halfbody_x, +halfbody_y]) + np.array([X_body[i], Y_body[i]])
        LH[:,i] = Rotation[i] @ np.array([-halfbody_x, +halfbody_y]) + np.array([X_body[i], Y_body[i]])
        RH[:,i] = Rotation[i] @ np.array([-halfbody_x, -halfbody_y]) + np.array([X_body[i], Y_body[i]])
        RF[:,i] = Rotation[i] @ np.array([+halfbody_x, -halfbody_y]) + np.array([X_body[i], Y_body[i]])

    return LF,LH,RH,RF,body_orientation,Rotation

class pointclouds:

    def __init__(self, data=np.array([]), boundx=np.array([]), boundy=np.array([]), length=0, width=0, resolution=0.01):
        self.data = data
        self.boundx = boundx
        self.boundy = boundy
        self.length = length
        self.width = width
        self.resolution = resolution

def DepImage2PC(image_path, scale = 0.35, resolution = 0.01):

    depth_array = np.array(Image.open(image_path))
    [length, width] = depth_array.shape
    data = np.zeros((length*width,3))

    
    grayValue = depth_array / 255.0

    x = -((np.arange(length) - 1) - (length - 1) / 2)
    y = -((np.arange(width) - 1) - (width - 1) / 2)

    z = scale * grayValue


    x = x * resolution
    y = y * resolution

    print(x.shape)
    print(y.shape)

    Y_grid, X_grid = np.meshgrid(y, x)
    x = X_grid.flatten()
    y = Y_grid.flatten()
    z = z.flatten()
    print(x)
    print(y.shape)
    print(z.shape)

    data = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    
    boundx = np.array([data[:,0].min(), data[:,0].max()])
    boundy = np.array([data[:,1].min(), data[:,1].max()])
    print(boundx, boundy)
    print("Point cloud data shape:", data.shape)
    
    PC = pointclouds(data, boundx, boundy, length, width, resolution)
    
    return PC

def fit_gaussian_process(PC):
    # Extract x, y, z coordinates from the point cloud
    X_train = PC.data[:, :2]  # Use x and y as input features
    y_train = PC.data[:, 2]   # Use z as the target value

    # Define your custom kernel here
    kernel = C(0.1, (0.1, 1.0)) * RBF(length_scale=1.0, length_scale_bounds=(0.5, 2.0))

    # Fit the Gaussian Process model
    gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)
    print("Fitting Gaussian Process model...")
    try:
        gp.fit(X_train, y_train)
    except Exception as e:
        print(f"An error occurred while fitting the Gaussian Process: {e}")
        raise
    print("Gaussian Process model fitted.")

    return gp


def terrain_GS(X, Y, PC):

    # Fit a Gaussian Process model using the point cloud data
    GP_model = fit_gaussian_process(PC)
    print("Gaussian Process model fitted.")

    # Combine X and Y into a single array for prediction
    XY = np.vstack((X, Y)).T

    # Predict Z values using the Gaussian Process model
    Z = GP_model.predict(XY)
    print("Z values predicted using Gaussian Process.")

    return Z


def ine2cost(x):
    
    N = int(x.size)
    y_ = np.zeros(N)

    for i in range(0,N):
        x_ = x[i]
        if x_ >= cbf_delta:
            y_[i] = -cbf_mu*math.log(x[i])
        else:
            y_[i] = 0.5*cbf_mu*(((x[i]-2*cbf_delta)/cbf_delta)**2-1) - cbf_mu*math.log(cbf_delta)

    return np.sum(y_)  

def cost_y1(Z, weight):
    weight_y = weight.y
    weight_x = weight.x
    N = int(Z.size/2)
    X = Z[:N]
    Y = Z[N:]
    select_Y = np.zeros((1,N))
    select_Y[-1,-1] = 1
    select_X = select_Y.copy()
    
    return np.sum(weight_y*(select_Y@Y - y_end)**2) + np.sum(weight_x*(select_X@X - x_end)**2)


def cost_y2_GS(Z, weight, scenario):
    weight_terrain = weight.terrain
    N = int(Z.size / 2)
    X = Z[:N]
    Y = Z[N:]

    [LF, LH, RH, RF, _, _] = HipPosition(X, Y)
    LF_ = LF[:,0::2]
    RH_ = RH[:,0::2]
    LH_ = LH[:,1::2]
    RF_ = RF[:,1::2]
    X_ = np.concatenate((LF_[0,:], LH_[0,:], RH_[0,:], RF_[0,:]), axis=0)
    Y_ = np.concatenate((LF_[1,:], LH_[1,:], RH_[1,:], RF_[1,:]), axis=0)
    y2 = 0
    print("Loading scenario:", scenario)
    PC = DepImage2PC(scenario+".png", scale = 0.35, resolution = 0.1)
    print("Point cloud generated from depth image.")



    result = (terrain_GS(X_, Y_, PC) - 0)


    y2 = sum(result ** 2)

    print(f"cost_terrain: {weight_terrain * y2}")
    print(f"cost_terrain / weight: {y2}")

    return weight_terrain * y2

def cost_y3(Z, weight):
    N = int(Z.size/2)
    X = Z[:N]
    X = np.concatenate((np.array([x_start]), X), axis=0)
    Y = Z[N:]
    Y = np.concatenate((np.array([y_start]), Y), axis=0)

    delta_X = delta_x * np.ones(N)
    delta_Y = delta_y * np.ones(N)
    A = np.zeros((N,N+1))


    A[np.arange(N), np.arange(N)] = -1  
    A[np.arange(N), np.arange(N) + 1] = 1 

    y3_x = ine2cost(A@X + 0) + ine2cost(-A@X + delta_X + 0)
    y3_y = ine2cost(A@Y + delta_Y + 0) + ine2cost(-A@Y + delta_Y + 0)

    y3 = weight.X_seq*y3_x + weight.Y_seq*y3_y

    return y3

def cost_y4(Z, weight, obstacles):
    weight_slope = weight.slope
    N = int(Z.size / 2)
    X = Z[:N]
    Y = Z[N:]
    

    [LF, LH, RH, RF, _, _] = HipPosition(X, Y)
    X_H = np.concatenate((LH[0,:], RH[0,:]), axis=0)
    X_F = np.concatenate((RF[0,:], LF[0,:]), axis=0)
    Y_H = np.concatenate((LH[1,:], RH[1,:]), axis=0)
    Y_F = np.concatenate((RF[1,:], LF[1,:]), axis=0)

    height_H = terrain_GS(X_H, Y_H, obstacles)
    height_F = terrain_GS(X_F, Y_F, obstacles)
    
    y4 = sum((height_H - height_F) ** 2)

    return weight_slope*y4

def cost_y5(Z, weight, obstacles):
    weight_LR = weight.LR
    N = int(Z.size/2)
    X = Z[:N]
    Y = Z[N:]

    [LF, LH, RH, RF, _, _] = HipPosition(X, Y)
    X_L = np.concatenate((LF[0,:], LH[0,:]), axis=0)
    X_R = np.concatenate((RH[0,:], RF[0,:]), axis=0)
    Y_L = np.concatenate((LF[1,:], LH[1,:]), axis=0)
    Y_R = np.concatenate((RH[1,:], RF[1,:]), axis=0)
    
    height_L = terrain_GS(X_L, Y_L, obstacles)
    height_R = terrain_GS(X_R, Y_R, obstacles)
    y5 = sum((height_L - height_R) ** 2)

    return weight_LR*y5


def cost_y6(Z, weight):
    weight_orientation = weight.orientation
    weight_diff_orientation = weight.diff_orientation
    N = int(Z.size/2)
    X = Z[:N]
    Y = Z[N:]

    [_, _, _, _, body_orientation, _] = HipPosition(X, Y)

    A = np.zeros((N,N+1))
    A[np.arange(N), np.arange(N)] = -1 
    A[np.arange(N), np.arange(N) + 1] = 1

    angle = np.arccos(body_orientation[0,:])
    diff_orientation = A @ angle

    y6 = sum(diff_orientation ** 2) 
    y7 = sum(angle ** 2)

    return weight_diff_orientation*y6 + weight_orientation*y7


def cost_GS(Z, weight,scenario):
    y1 = cost_y1(Z, weight)
    y2 = cost_y2_GS(Z, weight, scenario)
    y3 = cost_y3(Z, weight)
    y6 = cost_y6(Z, weight)

    return y1 + y2 + y3 + y6




class obstacle:
    def __init__(self, width, length, height, x, y, z, theta):
        self.width = width
        self.length = length
        self.height = height
        self.x = x
        self.y = y
        self.z = z
        self.theta = theta

class weight:
    def __init__(self):

        self.x = 800
        self.y = 800
        self.terrain = 300 
        self.Y_seq = 2000 
        self.X_seq = 8000 
        self.orientation = 15
        self.diff_orientation = 4

        self.slope = 0
        self.LR = 0

if __name__ == "__main__":
    

    halfbody_x = 0.1934
    halfbody_y = 0.142
    cbf_mu = 0.001
    cbf_delta = 0.1

    y_start = 0
    x_start = 0
    y_end = 0
    x_end = 1

    velocity = 0.4
    T = 0.6
    dt = 0.1
    gaitNumber = math.ceil((x_end-x_start)/velocity/T*2)
    N = 2*(gaitNumber + 2)
    dim = int(N/2)

    delta_x = 0.2
    delta_y = 0.2


    xmean = np.linspace(x_start + (x_end - x_start)/dim, x_end, dim)

    ymean = np.linspace(y_start + (y_end - y_start)/dim, y_end, dim)


    zmean = np.concatenate((xmean, ymean), axis=0)
    print(zmean.shape)

    boundx_low = x_start*np.ones(dim)
    boundx_high = x_end*np.ones(dim)
    boundx = np.array([boundx_low, boundx_high]).T
    boundy_low = -0.51*np.ones(dim)
    boundy_high = 0.21*np.ones(dim)
    boundy = np.array([boundy_low, boundy_high]).T
    bounds_ = np.concatenate((boundx, boundy), axis=0)
    print(bounds_.shape)

    popsize = 100
    optimizer = CMA(mean=zmean, sigma=0.01, population_size=popsize, bounds = bounds_)

    weight = weight()

    start_time = time.time()  
    
    for i in range(400,401): # run different random seeds
        start_time = time.time() 
        solutions = []
        Z = None
        np.random.seed(i)  # set different random seed
        optimizer = CMA(mean=zmean, sigma=0.03, population_size=popsize)
    
        for generation in range(100):
            
            solutions = []
            for _ in range(optimizer.population_size):
                Z = optimizer.ask()
                s_time = time.time()

                Scenario = "stack8" # relpace your own scenario here
                
                value = cost_GS(Z, weight, Scenario)

                solutions.append((Z, value))

                print(f"#{generation} {value}")
                e_time = time.time()

            optimizer.tell(solutions)
            

        end_time = time.time()
        elapsed_time = end_time - start_time 
        print(f"Total elapsed time: {elapsed_time} seconds")


        best_solution = min(solutions, key=lambda s: s[1])
        best_Z, best_value = best_solution
        best_X = np.concatenate((np.array([x_start]), best_Z[:dim]), axis=0) 
        best_Y = np.concatenate((np.array([y_start]), best_Z[dim:]), axis=0)
        _,_,_,_,orientation,_ = HipPosition(best_X[1:], best_Y[1:]) 
        
    
