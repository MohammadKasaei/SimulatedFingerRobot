from cProfile import label
import matplotlib
import numpy as np
import numpy as np
from   scipy.integrate import solve_ivp
import scipy.sparse as sparse
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


import osqp

class MPCController():
    def __init__(self) -> None:
        
        self.ts = 0.05
        self.Np = 20
        self.gt = 0
        self.sim_time = 0
        
        self.udateSystem(jac=np.zeros((3,3)))
        
        self.umin = 0.5*np.array([-0.01,-0.01,-0.01]) 
        self.umax = 0.5*np.array([ 0.01, 0.01, 0.01]) 
        
        self.xmin = np.array([-0.05, -0.05, 0.08 ])
        self.xmax = np.array([ 0.05 , 0.05, 0.15])


    def udateSystem(self, jac):
        self.Ad = np.matrix([[1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        self.Bd = jac*self.ts
        
        
    def ctrl(self,x0):

        [nx, nu] = self.Bd.shape # number of states and number or inputs
        
        # Objective function
        Q = sparse.diags([50.0, 50.0, 50.0])
        QN = sparse.diags([0.0, 0.0, 0.0]) # final cost
        R = 0.001*sparse.eye(nu)

        # Reference input 
        xref = self.ref 

        # Prediction horizon
        Np = self.Np

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(Np), Q), QN,
                            sparse.kron(sparse.eye(Np), R)]).tocsc()
        # - linear objective
        q = np.hstack([np.kron(np.ones(Np), -Q.dot(xref[:,Np-1])), -QN.dot(xref[:,Np-1]),
                    np.zeros(Np * nu)])
        
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(Np + 1), -sparse.eye(nx)) + sparse.kron(sparse.eye(Np + 1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, Np)), sparse.eye(Np)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-x0, np.zeros(Np * nx)])
        ueq = leq # for equality constraints -> upper bound  = lower bound!

        # - input and state constraints
        Aineq = sparse.eye((Np + 1) * nx + Np * nu)
        lineq = np.hstack([np.kron(np.ones(Np + 1), self.xmin), np.kron(np.ones(Np), self.umin)]) # lower bound of inequalities
        uineq = np.hstack([np.kron(np.ones(Np + 1), self.xmax), np.kron(np.ones(Np), self.umax)]) # upper bound of inequalities

        
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq]).tocsc()
        self.l = np.hstack([leq, lineq])
        self.u = np.hstack([ueq, uineq])

        # Create an OSQP object
        self.prob = osqp.OSQP()

        # Setup workspace
        self.prob.setup(P, q, A, self.l, self.u, warm_start=False)

        res = self.prob.solve()
        # Apply first control input to the plant
        uMPC = res.x[-Np * nu:-(Np - 1) * nu]
        return uMPC


class robotModel():
    def __init__(self) -> None:
        self.l  = 0
        self.uy = 0
        self.ux = 0
        self.dp = 0
        self.err = np.array((0,0,0))
        self.errp = np.array((0,0,0))

        self.simCableLength  = 0
        # initial length of robot
        self.l0 = 100e-3
        # cables offset
        self.d  = 7.5e-3
        # ode step time
        self.ds     = 0.005 #0.0005  
        r0 = np.array([0,0,0]).reshape(3,1)  
        R0 = np.eye(3,3)
        R0 = np.reshape(R0,(9,1))
        y0 = np.concatenate((r0, R0), axis=0)
        self.states = np.squeeze(np.asarray(y0))
        self.y0 = np.copy(self.states)


    def updateAction(self,action):
        self.l  = self.l0 + action[0]        
        self.uy = (action[1]) /  (self.l * self.d)
        self.ux = (action[2]) / -(self.l * self.d)


    def odeFunction(self,s,y):
        """
            Defines the differential equations for the soft robot dynamics. It calculates the derivative of the state vector.
        """   
        dydt  = np.zeros(12)
        # % 12 elements are r (3) and R (9), respectively
        e3    = np.array([0,0,1]).reshape(3,1)              
        u_hat = np.array([[0,0,self.uy], [0, 0, -self.ux],[-self.uy, self.ux, 0]])
        r     = y[0:3].reshape(3,1)
        R     = np.array( [y[3:6],y[6:9],y[9:12]]).reshape(3,3)
        # % odes
        dR  = R @ u_hat
        dr  = R @ e3
        dRR = dR.T
        dydt[0:3]  = dr.T
        dydt[3:6]  = dRR[:,0]
        dydt[6:9]  = dRR[:,1]
        dydt[9:12] = dRR[:,2]
        return dydt.T


    def odeStepFull(self):        
        cableLength          = (0,self.l)        
        t_eval               = np.linspace(0, self.l, int(self.l/self.ds))
        sol                  = solve_ivp(self.odeFunction,cableLength,self.y0,t_eval=t_eval)
        self.states          = np.squeeze(np.asarray(sol.y[:,-1]))
        return sol.y
    
    def Jac(self, q, dq=np.array((1e-4,1e-4,1e-4))):
        """
        Calculates the Jacobian of the system for a given state q and small changes dq in the state.
        This is used for sensitivity analysis and control purposes.
        """
        f = self.runOdeForJac
        fx0 = f(q)
        n   = len(q)
        m   = len(fx0)
        jac = np.zeros((n, m))
        for j in range(m):  # through rows 
            if (j==0):
                Dq = np.array((dq[0]/2.0,0,0))
            elif (j==1):
                Dq = np.array((0,dq[1]/2.0,0))
            elif (j==2):
                Dq = np.array((0,0,dq[2]/2.0))

            jac [j,:] = (f(q+Dq) - f(q-Dq))/dq[j]
        return jac    

    def runOdeForJac(self,q):    
        """
        Solves the ODE to find the robot's state given the input parameters. This is specifically used to assist in calculating the Jacobian.
        """           
        l  = self.l0 + q[0]
        self.uy = (q[1]) / (l *self.d)
        self.ux = (q[2]) / -(l*self.d)

        cableLength          = (0,l)
        t_eval               = np.linspace(0, l, int(l/self.ds))
        sol                  = solve_ivp(self.odeFunction,cableLength,self.y0,t_eval=t_eval)
        self.states          = np.squeeze(np.asarray(sol.y[:,-1]))
        return self.states[0:3]
    

class softRobotVisualizer():
    def __init__(self) -> None:
       
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.title = self.ax.set_title('Visualizer-1.01')
        self.xlabel = self.ax.set_xlabel("x (m)")
        self.ylabel = self.ax.set_ylabel("y (m)")
        self.zlabel = self.ax.set_zlabel("z (m)")
        self.ax.set_xlim([-0.08,0.08])
        self.ax.set_ylim([-0.08,0.08])
        self.ax.set_zlim([-0.0,0.15])
        self.speed = 1 
        
        self.actions = None
        self.endtips = None
        self.refTraj = None

        self.ode = robotModel()       
        self.robot = self.ax.scatter([], [], [],marker='o',lw=5)
        self.robotBackbone, = self.ax.plot([], [], [],'r',lw=4)
        self.endTipLine, = self.ax.plot([], [], [],'r',lw=2)
        self.refLine, = self.ax.plot([], [], [],'k--',lw=3)
        


    def update_graph(self,num):
        
        if self.actions is None:
            self.ode.updateAction(np.array((0+num/100,num/1000,num/1000)))
        else:
            self.ode.updateAction(self.actions[int(num*self.speed),:])

        self.sol = self.ode.odeStepFull()
        
        self.robot._offsets3d = (self.sol[0,:], self.sol[1,:], self.sol[2,:])
        self.robotBackbone.set_data(self.sol[0,:], self.sol[1,:])
        self.robotBackbone.set_3d_properties(self.sol[2,:])
        
        self.endTipLine.set_data(self.endtips[0:int(num*self.speed),0], self.endtips[0:int(num*self.speed),1])
        self.endTipLine.set_3d_properties(self.endtips[0:int(num*self.speed),2])
        
        self.refLine.set_data(self.refTraj[:,0], self.refTraj[:,1])
        self.refLine.set_3d_properties(self.refTraj[:,2])


        
if __name__ == "__main__":
    robot = robotModel()
    mpc = MPCController()
 
    q = np.array([0.0,-0.0,0.0])
    x0 = np.array([0, 0, 0.1])
    
    mpc.states = np.copy(x0)
    mpc.lastU  = np.copy(q)
    
    robot.updateAction(q)
    robot.odeStepFull()
    xc = robot.states[:3]
    ts = mpc.ts
    
    tf = 100
    T  = 25
    amp  = 0.02  
    w  = 2*np.pi/T
    
    logState = np.array([])
    refTraj = np.array([])
    tipTraj = np.array([])

    for t in range(int(tf/ts)):        
        gt = t*ts
        xd = x0 + np.array([amp*np.sin(w*(gt)), amp*np.sin(w/2*(gt)),0.00])
        t_ref = np.linspace(gt,np.clip(gt+(mpc.Np*mpc.ts),0,tf),mpc.Np)
        
        mpc.ref = np.vstack([(amp*np.sin(w*t_ref), amp*np.sin((w/2)*t_ref),x0[2]+0*t_ref)])

        jac = robot.Jac(q).T
        mpc.udateSystem(jac)
        qdot = mpc.ctrl(xc)
        q += (qdot * ts)
        
        robot.updateAction(q)
        robot.odeStepFull()
        xc = robot.states[:3]
        

        if t==0:
            logState = np.copy(q)
            refTraj = np.copy(xd)
            tipTraj = np.copy(xc)
        else:
            logState =  np.vstack((logState,q))
            refTraj =  np.vstack((refTraj,xd))
            refTraj =  np.vstack((refTraj,xd))
            tipTraj =  np.vstack((tipTraj,xc))
            
            

    fig, axs = plt.subplots(3)    
    t = np.linspace(0,tf,logState.shape[0])
    axs[0].plot(t,logState[:,1],'r',label='u_x')    
    axs[1].plot(t,logState[:,2],'g',label='u_y')    
    axs[2].plot(t,logState[:,0],'b',label='u_z')    
   
    # Adding labels
    axs[0].set_ylabel('u_x [m]')
    axs[1].set_ylabel('u_y [m]')
    axs[2].set_ylabel('u_z [m]')
    axs[2].set_xlabel('Time [s]')
    
    
    sfVis = softRobotVisualizer()
    sfVis.actions = logState
    sfVis.endtips = tipTraj
    sfVis.refTraj = refTraj
    
    ani = animation.FuncAnimation(sfVis.fig, sfVis.update_graph, logState.shape[0], interval=10, blit=False)

    plt.show()
        








