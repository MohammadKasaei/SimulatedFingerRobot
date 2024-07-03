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
import control 

class MPC():
    def __init__(self) -> None:
        
        self.ts = 0.05
        self.Np = 25
        self.gt = 0
        self.sim_time = 0
        self.obsPos1 = None
        self.obsPos2 = None
        self.obsPos3 = None
        self.obsPos4 = None
        

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
        

        
    def u_MPC(self,x0):

        [nx, nu] = self.Bd.shape # number of states and number or inputs

        
        # Objective function
        Q = sparse.diags([50.0, 50.0, 50.0])
        QN = sparse.diags([0.0, 0.0, 0.0]) # final cost
        R = 0.001*sparse.eye(nu)

        # Initial and reference states
        # x0 = xc 

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
        # T = 5
        # w = 2*np.pi/T

        # self.obsPos1 = np.array((0.02+0.01*np.sin(w*self.gt),0.0,0.11))
        # self.obsPos2 = np.array((-0.02,0.02+0.01*np.sin(w*self.gt),0.1))
        # self.obsPos3 = np.array((0.02,-0.02+0.01*np.sin(w*self.gt),0.1))
        # self.obsPos4 = np.array((-0.02,-0.02+0.01*np.sin(w*self.gt),0.1))
        
            

        # refTrajCost = []
        # for i in range(Np):
        #     d1 = np.linalg.norm(np.array((xref[0,i],xref[1,i],xref[2,i]))-self.obsPos1)
        #     d2 = np.linalg.norm(np.array((xref[0,i],xref[1,i],xref[2,i]))-self.obsPos2)
        #     d3 = np.linalg.norm(np.array((xref[0,i],xref[1,i],xref[2,i]))-self.obsPos3)
        #     d4 = np.linalg.norm(np.array((xref[0,i],xref[1,i],xref[2,i]))-self.obsPos4)
            
        #     if (d1>0.01): 
        #         cd1 = 0
        #     else:
        #         cd1 = (0.01**2-(d1**2))*1000*0

        #     if (d2>0.01): 
        #         cd2 = 0
        #     else:
        #         cd2 = (0.01**2-(d2**2))*1000*0

        #     if (d3>0.01): 
        #         cd3 = 0
        #     else:
        #         cd3 = (0.01**2-(d3**2))*1000*0
        #     if (d4>0.01): 
        #         cd4 = 0
        #     else:
        #         cd4 = (0.01**2-(d4**2))*1000*0
                
                

        #     refTrajCost = np.append(refTrajCost,-Q.dot(xref[:,i])+cd1+cd2+cd3+cd4,axis =0)

        # q = np.hstack([refTrajCost, -QN.dot(xref[:,Np-1]),
        #             np.zeros(Np * nu)])

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



    def ode_fcn(self,t,x):    
        u = np.matrix(self.u_MPC())
        
        checkNone = True if u[0,0] is None or u[0,1] is None or u[0,2] is None else False
        if checkNone:
            u = self.lastU            
        self.lastU = np.copy(u)
        
        # u = np.clip(u, -5,5)
        self.u = np.copy(u)


        # dxdt = self.A@(np.matrix(x).T) + self.B@(u.T)
        dxdt = self.Bd@(u.T)
        
        # dxdt = self.Ad@(np.matrix(x).T) + self.Bd@(u.T)
        
        return dxdt.T


    def ode_step(self):
        stime          = (self.sim_time,self.sim_time+self.ts)
        t_eval         = np.array([stime[1]]) #np.linspace(t0, tfinal, int(tfinal/ts))
        sol            = solve_ivp(self.ode_fcn,stime,self.states,t_eval=t_eval)
        self.sim_time += self.ts
        self.states    = [y[0] for y in sol.y]
        #print ("t: {0}, states: {1}".format(self.sim_time,self.states))
        return self.states


    def run_sim_steps(self,ref,x0,t0,ts,tfinal):  
        
        y = x0
        self.states = np.copy(x0)
        for n in range(int(tfinal/ts)):
            tfinal   = t0+ts
            sim_time = (t0,tfinal)
            self.ts  = ts
            t_eval   = np.array([t0+ts])#np.linspace(t0, tfinal, int(tfinal/ts))
            # self.ref = np.array([np.sin(t0),np.cos(t0),np.cos(t0),-np.sin(t0), t0,1]) #ref
            self.ref = np.array([1,0.0,2,0., 1.5,0.]) #ref
            
            sol      = solve_ivp(self.ode_fcn,sim_time,x0,t_eval=t_eval)
            x0 = [y[0] for y in sol.y]
            self.states    = [y[0] for y in sol.y]
           

            t0 +=ts
        
            if (n):
                yn = np.array([y[0] for y in sol.y])
                y = np.append(y,[yn],axis=0)
            else:
                y = [np.array([y[0] for y in sol.y])]
                
        self.animation_states = y.T

        return y.T

    def singleODEStep(self,):
        # stime          = (self.sim_time,self.sim_time+self.ts)
        # t_eval         = np.array([stime[1]]) #np.linspace(t0, tfinal, int(tfinal/ts))
        # sol            = solve_ivp(self.ode_fcn,stime,self.states,t_eval=t_eval)
        
        u = np.matrix(self.u_MPC())
        checkNone = True if u[0,0] is None or u[0,1] is None or u[0,2] is None else False
        if checkNone:
            u = self.lastU            
        self.lastU = np.copy(u)
        self.u = np.copy(u)
        self.states = self.states + self.Bd@(u.T)*self.ts
        self.sim_time += self.ts
        
        return np.array(self.states)




class SoftRobotControl():
    def __init__(self) -> None:
        """
        Initialize the SoftRobotControl class with default parameters.
        Sets up the initial state of the robot including its length, cable offsets, and initial orientation.
        """
        
        # initial length of robot
        self.l0 = 100e-3
        # cables offset
        self.d  = 7.5e-3
        # ode step time
        self.ds     = 0.005  

        r0 = np.array([0,0,0]).reshape(3,1)  
        R0 = np.eye(3,3)
        R0 = np.reshape(R0,(9,1))
        y0 = np.concatenate((r0, R0), axis=0)
        self.states = np.squeeze(np.asarray(y0))
        self.y0 = np.copy(self.states)

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
        #print ("t: {0}, states: {1}".format(self.sim_time,self.states))
        return self.states[0:3]

    def visualize(self,state):
        """
        Visualizes the robot's trajectory in 3D space using matplotlib.
        """        
        fig = plt.figure()
        ax  = fig.add_subplot(projection='3d')
        ax.scatter (state[:,0],state[:,1],state[:,2], c = 'g', label='robot')
        ax.plot3D(state[:,0], state[:,1],state[:,2], c='r',lw=2)

        
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
        ax.set_zlabel('z (m)')
        ax.set_xlim(-0.08, 0.08)
        ax.set_ylim(-0.06, 0.06)
        ax.set_zlim(-0.05, 0.15)
        
        ax.legend()
        
        plt.show()


class ODE():
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
        # self.l  = action0
        
        self.uy = (action[1]) /  (self.l * self.d)
        self.ux = (action[2]) / -(self.l * self.d)


    def odeFunction(self,s,y):
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

        self.ode = ODE()       
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
    env = SoftRobotControl()
    robot = ODE()
    
    mpc = MPC()
 
    q = np.array([0.0,-0.0,0.0])
    x0 = np.array([0, 0, 0.1])
    
    mpc.states = np.copy(x0)
    mpc.lastU  = np.copy(q)
    
    xdot = np.array([0.0,0.0,0.])
    robot.updateAction(q)
    robot.odeStepFull()
    K = np.diag((0.45, 0.45, 0.45))
    xc = robot.states[:3]
    ts = mpc.ts
    tf = 100
    logState = np.array([])
    refTraj = np.array([])
    tipTraj = np.array([])
    
    for t in range(int(tf/ts)):        
        gt = t*ts
        T  = 25
        A  = 0.04  
        w  = 2*np.pi/T
        radius = 0.02
        xd = x0 + np.array([A*np.sin(w*(gt)), A*np.sin(w/2*(gt)),0.00])
        # xd_dot = np.array([-radius*w*np.sin(w*(gt)),radius*w*np.cos(w*(gt)),0.00])
        
        
        tt = np.linspace(gt,np.clip(gt+(mpc.Np*mpc.ts),0,tf),mpc.Np)
        # mpc.gt += mpc.ts
        
        amp = [0.02,0.02,0.00]
        mpc.ref = np.vstack([(A*np.sin(w*tt), A*np.sin((w/2)*tt),x0[2]+amp[2]*tt)])

        jac = env.Jac(q).T
        mpc.udateSystem(jac)
        qdot = mpc.u_MPC(xc)
        # mpc.sim_time += mpc.ts
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
            
            
        
        # print (f"t = {gt:2.2f}")


    fig, axs = plt.subplots(3)    
    # t = np.linspace(0,tfinal,y.shape[0])
    axs[0].plot(logState[:,0],'b',label='z')    
    axs[1].plot(logState[:,1],'r',label='x')    
    axs[2].plot(logState[:,2],'g',label='y')    
    
    
    sfVis = softRobotVisualizer()
    sfVis.actions = logState
    sfVis.endtips = tipTraj
    sfVis.refTraj = refTraj
    
    ani = animation.FuncAnimation(sfVis.fig, sfVis.update_graph, logState.shape[0], interval=10, blit=False)

    plt.show()
        








