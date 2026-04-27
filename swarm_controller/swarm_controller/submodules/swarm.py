import numpy as np

from swarm_msgs.msg import Telemetry


class Agent:
    
    def __init__(self, id):
        self.id = id
        self.data = {
            'x': None,
            'y': None,
            'theta': None,
            'v': None,
            'w': None
        }
        self.v_ref = 0.0
        self.w_ref = 0.0
    
    def set_data(self, x, y, theta, v, w):
        self.data['x'] = x
        self.data['y'] = y
        self.data['theta'] = theta
        self.data['v'] = v
        self.data['w'] = w
    
    @property    
    def pt(self):
        return np.array([self.data['x'], self.data['y']]).reshape(-1, 1)
    
    @property
    def v(self):
        return self.data['v']
    
    @property
    def theta(self):
        return self.data['theta']
    
    def __repr__(self):
        return f"Agent {self.id}: x = {self.data['x']}, y = {self.data['y']}, theta = {self.data['theta']}, \
            v = {self.data['v']}, w = {self.data['w']} v_ref = {self.v_ref}, w_ref = {self.w_ref}"
    
    
class Swarm:
    
    def __init__(self, robots_num, robots_ids, pacemaker_idx):
        self.robots_num = robots_num
        self.pacemaker_idx = pacemaker_idx
        self.agents = []
        self.idx_dict = dict()
        for i, robot_id in enumerate(robots_ids):
            self.agents.append(Agent(robot_id))
            self.idx_dict[robot_id] = i
            if self.pacemaker_idx == robot_id:
                self.pacemaker_list_idx = i
                
    def __repr__(self):
        s = '\n'
        for agent in self.agents:
            s += agent.__repr__() + '\n'
        return s
        
    def get_data(self, data_type: str):
        data = []
        
        for agent in self.agents:
            data.append(agent.data[data_type])
            
        return data
    
    def set_data(self, agent_idx : int, data : list):
        self.agents[agent_idx].set_data(*data)
    
    def create_telemetry(self, is_valid):
        telemetry = Telemetry()
        telemetry.x = self.get_data('x')
        telemetry.y = self.get_data('y')
        telemetry.theta = self.get_data('theta')
        telemetry.v = self.get_data('v')
        telemetry.w = self.get_data('w')
        telemetry.is_valid = is_valid
        
        return telemetry
    
    def set_data_from_telemetry(self, t : Telemetry, logger):
        for i, agent in enumerate(self.agents):
            if agent.id == self.pacemaker_idx:
                x = t.peer_x
                y = t.peer_y
                agent.set_data(x, y, 0.0, 0.0, 0.0)
            else:
                agent.set_data(t.x, t.y, t.theta, t.v, t.w)
            
    @property
    def pacemaker(self):
        return self.agents[self.pacemaker_list_idx]
