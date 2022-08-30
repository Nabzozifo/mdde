from gym.spaces import Discrete, MultiDiscrete, Dict
from collections import OrderedDict
import matplotlib.pyplot as plt
from gym.spaces import Discrete, MultiDiscrete
import gym
import numpy as np
K = 100  # number of time slot
I = 500  # number of IoT devices
U = 3  # number of UAV
xumin = 5  # UAV xmin bound
xumax = 30  # UAV xmax bound
yumin = 5  # UAV ymin bound

yumax = 30  # UAV ymax bound
# zumin=10 #UAV zmin bound
# zumax=40 #UAV zmax bound
H = np.random.randint(low=90, high=100, size=U)
# BUAV=np.random.randint(1700,2000,size=U) #UAVs bandwidth

y = yumax  # CDS position y
x = np.random.randint(0, xumax, size=U)  # CDS position x

initial_y = 0  # initial drone position y
initial_x = np.random.randint(0, xumax, size=U)  # initial drone position x
while len(set(initial_x)) != U:
    initial_x = np.random.randint(0, xumax, size=U)
# for u in range(1,U):
#   initial_x[u] = initial_x[u-1]+1 #initial drone position x

# Xi = np.array([np.random.randint(low=2, high=xumax, size=I) for _ in range(K)]) #X coordinate IOTs
# Yi = np.array([np.random.randint(low=2, high=yumax, size=I) for _ in range(K)]) #Y coordinate IOTs
# Zi = np.array([np.zeros((I)) for _ in range(K)])

Xi = np.random.randint(low=1, high=xumax, size=I)  # X coordinate IOTs
Yi = np.random.randint(low=1, high=yumax, size=I)  # Y coordinate IOTs
Zi = np.zeros((I))

BIoT = np.random.randint(1500, 1700, size=I)  # IoTs bandwidth
Ri = 135000  # minimum IoTs rate
noise = 1e-15
alpha = 2  # path loss exponent
PIot = np.random.rand(I, U, K)  # Puissance transmit of IoTs
fadingIot = np.random.rand(I, U, K)
beta0 = 1
deltat = 1  # time step length
V = 15  # vitesse du drone

R_bar_i_u = np.zeros((I, U))  # the total sum-rate over Time stepT
R_mission = np.zeros((I, U))  # reward obtained by completing a mission

# EmaxUAV=np.random.randint(800,1000,size=U)
# Variable for computing some metrics
RateIot = np.zeros((I, U, K))

# EnergyUAV=np.zeros((U,K))
# Associaton variable
a = np.zeros((I, U, K))
AoU = np.zeros((I, U, K))
# b=np.zeros((U,K))

UPWARD = 0
DOWNWARD = 1
FORWARD = 2
BACKWARD = 3
LEFT = 4
RIGHT = 5


lambda1 = 0.3
lambda2 = 0.4
lambda3 = 0.3


def computeDistance(x1, x2, y1, y2, z1, z2):
    return np.sqrt((x1 - x2)**2+(y1 - y2)**2+(z1 - z2)**2)


def computeRate(B, noise, beta0, P, fading, x1, x2, y1, y2, z1, z2, alpha=2):
    return B * np.log2(1 + ((beta0 * P * fading**2) / ((computeDistance(x1, x2, y1, y2, z1, z2)**alpha) * noise**2)))


def computeR_bar_i_u(R_bar_i_u, xu, yu, T):
    for u in range(U):
        for i in range(I):
            for t in range(T):
                R_bar_i_u[i, u] = computeRate(
                    BIoT[i], noise, beta0, PIot[i, u, t], fadingIot[i, u, t], Xi[i], xu, Yi[i], yu, 0, H[u], alpha=2)
    return R_bar_i_u


# def computeR_R_mission(R_mission, xu, yu, t):
#     for u in range(U):
#         for i in range(I):
#             for t in range (T):
#                 R_bar_i_u[i,u]=computeRate(BIoT[i],noise,beta0,PIot[i,u,t],fadingIot[i,u,t],Xi[i], xu, Yi[i], yu, 0, H[u],alpha=2)
#     return R_mission

# def objective(a,R_bar_i_u,T,xu,yu,Rplus):
#     computeR_bar_i_u(R_bar_i_u, xu, yu,T)
#     obj=0
#     for u in range(U):
#         obj2=0
#         for i in range(I):
#             obj2=obj2+ (a[i,u] * R_bar_i_u[i,u])
#     obj=obj+ ( (alphaObj * obj2 / I) + (mu * Rplus))
#     return obj


# def objective1(a,u,t,xu,yu,Rplus1):
#     return ((alphaObj / I) * sum(a[:,u] * computeRate(BIoT[:],noise,beta0,PIot[:,u,t],fadingIot[:,u,t],Xi[:], xu, Yi[:], yu, 0, H[u],alpha=2))) + (mu * Rplus1)

# def objective2(a,u,t,xu,yu):
#     return ((alphaObj / I) * sum(a[:,u] * computeRate(BIoT[:],noise,beta0,PIot[:,u,t],fadingIot[:,u,t],Xi[:], xu, Yi[:], yu, 0, H[u],alpha=2))) + (mu * (1 / np.exp(np.sqrt(computeDistance(x, xu, y, yu, 0, H[u])))))


# def objective1cont(a,u,t,xu,yu,zu,Rplus1):
#     return ((alphaObj / I) * sum(a[:,u] * computeRate(BIoT[:],noise,beta0,PIot[:,u,t],fadingIot[:,u,t],Xi[:], xu, Yi[:], yu, 0, zu,alpha=2))) + (mu * Rplus1)


def objective2cont(a, u, t, xu, yu, zu, x, y, Tui, lambda1=lambda1, lambda2=lambda2, lambda3=lambda3):
    if t == 0:
        return 0
    else:
        obj1 = sum(a[:, u, t] * computeRate(BIoT[:], noise, beta0, PIot[:,
                   u, t], fadingIot[:, u, t], Xi[:], xu, Yi[:], yu, 0, zu, alpha=2))
        # obj1 = sum(a[:, u, t])
        obj2 = sum((Tui[:, u, t-1] + 1) * (1 - a[:, u, t-1]))
        obj3 = (1 / np.exp(np.sqrt(computeDistance(x[u], xu, y, yu, 0, zu))))
    return ((lambda1 / I * U) * obj1) + ((-lambda2 / I * U) * obj2) + ((lambda3 / I * U) * obj3)


# Let's code our multi-agent environment.

class MultiAgentDrone(gym.Env):
    def __init__(self, lambda1=lambda1, lambda2=lambda2, lambda3=lambda3):
        self.n_agents = U
        self.lambda1 = lambda1
        self.lambda2 = lambda2
        self.lambda3 = lambda3
        self.imnum = 0
        self.a = np.zeros((I, U, K))
        self.AoU = np.zeros((I, U, K))
        self.width = xumax
        self.heigh = yumax
        self._agent_ids = set([i for i in range(self.n_agents)])
        self.observation_space = gym.spaces.Tuple(
            tuple([gym.spaces.Box(low=0, high=self.width,
                  shape=(2,), dtype=np.int32)] * self.n_agents)
        )

        self.action_space = gym.spaces.Tuple(
            tuple(self.n_agents * [Discrete(4)]))

    def reset(self):
        self.timestep = 0
        self.a = np.zeros((I, U, K))
        self.AoU = np.zeros((I, U, K))
        self.collison = []
        self.obs = [0 for _ in self._agent_ids]
        self.visited_square = {}
        for agent in self._agent_ids:
            self.obs[agent] = np.array([initial_x[agent], initial_y])
            self.visited_square[agent] = []
        self.obs = tuple(self.obs)
        return self.obs

    def step(
        self, action_list
    ):
        rewards = [0 for _ in self._agent_ids]
        dones = [False for _ in self._agent_ids]
        infos = {}
        self.obs = list(self.obs)
        for agent in self._agent_ids:
            if action_list[agent] == 0:
                if self.obs[agent][0]+1 <= xumax:
                    self.obs[agent][0] += 1
            elif action_list[agent] == 1:
                if self.obs[agent][0]-1 >= xumin:
                    self.obs[agent][0] -= 1
            elif action_list[agent] == 2:
                if self.obs[agent][1]+1 <= yumax:
                    self.obs[agent][1] += 1
            else:
                if self.obs[agent][1]-1 >= yumin:
                    self.obs[agent][1] -= 1

        for i in range(I):
            for agent in self._agent_ids:
                if computeRate(BIoT[i], noise, beta0, PIot[i, agent, self.timestep], fadingIot[i, agent, self.timestep], Xi[i], self.obs[agent][0], Yi[i], self.obs[agent][1], 0, H[agent], alpha=2) >= Ri and [self.obs[agent][0], self.obs[agent][1]] == [Xi[i], Yi[i]]:
                    self.a[i, agent, self.timestep] = 1
                    self.AoU[i, agent, self.timestep] = 0
                else:
                    self.AoU[i, agent, self.timestep] = 1

        for agent in self._agent_ids:
            for agent1 in self._agent_ids:
                if agent != agent1:
                    if self.obs[agent][0] != self.obs[agent1][0] or self.obs[agent][1] != self.obs[agent1][1]:
                        rewards[agent] = objective2cont(
                            self.a, agent, self.timestep, self.obs[agent][0], self.obs[agent][1], H[agent], x, y, self.AoU, self.lambda1, self.lambda2, self.lambda3)
                    else:
                        rewards[agent] = - objective2cont(
                            self.a, agent, self.timestep, self.obs[agent][0], self.obs[agent][1], H[agent], x, y, self.AoU, self.lambda1, self.lambda2, self.lambda3)
                        self.collison.append([True, agent, agent1])

            self.visited_square[agent].append(
                [self.obs[agent][0], self.obs[agent][1]])

        for agent in self._agent_ids:
            if self.obs[agent][0] == x[agent] and self.obs[agent][1] == y:
                dones[agent] = True
            elif self.timestep >= K-1:
                dones[agent] = True
            else:
                dones[agent] = False

        self.obs = tuple(self.obs)
        self.timestep += 1

        return self.obs, rewards, dones, infos

    def rendered(self):
        set_matplotlib_close = False
        fig, ax = plt.subplots(figsize=(20, 15))
        ax.scatter(Xi, Yi, marker='o', c='g', s=50)

        marker = ["v", "^", "<", ">"]
        color = ['b', 'r', 'c', 'y']
        for agent in self._agent_ids:
            # initial position
            ax.scatter(initial_x[agent], initial_y,
                       marker="X", c=color[agent], s=50)
            # Target position
            ax.scatter(x[agent], y, marker="*", c=color[agent], s=50)
            # Current position
            ax.scatter(self.obs[agent][0], self.obs[agent][1],
                       marker=marker[agent], c=color[agent], s=50)
            # Visited square
            for square in self.visited_square[agent]:
                ax.scatter(square[0], square[1],
                           marker=marker[agent], c=color[agent], s=50)

        # for collision in self.collison:
        #     if collision[0]==True:
        #         print('Collision between '+ str(self.collison[0]) + " and " +str(self.collison[1]))

        ax.set_yticks(np.arange(self.width+1))
        ax.set_yticks(np.arange(self.width+2)-0.5, minor=True)

        ax.set_xticks(np.arange(self.width+1))
        ax.set_xticks(np.arange(self.width+2)-0.5, minor=True)

        ax.grid(True, which="minor")
        ax.set_aspect("equal")
        plt.savefig(
            '/Users/te/Desktop/python code/epymarl/epymarl/images/'+str(self.imnum)+'.png')
        plt.close()
        self.imnum = self.imnum+1

        return

    def render(self, mode='rgb_array'):
        self.rendered()
        return
