import numpy as np
from scipy.spatial.transform import Rotation


class Transform3D:
    
    @staticmethod
    def eye():
        return Transform3D(np.eye(3), Transform3D.translation())
    
    @staticmethod
    def from_flatten(buffer):
        buffer_copy = np.array(buffer) if type(buffer) == list else buffer.copy()
        buffer_copy = buffer_copy.reshape(4, 4)
        return Transform3D(buffer_copy[:3, :3], buffer_copy[:3, 3])
    
    @staticmethod
    def translation(x=0.0, y=0.0, z=0.0):
        return np.array([[x, y, z]]).T
    
    @staticmethod
    def quat2R(q_data, scalar_first=False):
        return Rotation.from_quat(np.array(q_data).flatten(), scalar_first=scalar_first).as_matrix()
    
    @staticmethod
    def R2quat(R):
        q = np.array(Rotation.from_matrix(R).as_quat(scalar_first=False)).reshape(-1, 1)
        return q / np.linalg.norm(q)
    
    @staticmethod
    def quat_mul(q1, q2):
        R1 = Transform3D.quat2R(q1)
        R2 = Transform3D.quat2R(q2)
        return Transform3D.R2quat(R1 @ R2)
    
    @staticmethod
    def Rx(angle):
        return Rotation.from_euler('xyz', [angle, 0, 0], degrees=False).as_matrix()
    
    @staticmethod
    def Ry(angle):
        return Rotation.from_euler('xyz', [0, angle, 0], degrees=False).as_matrix()
    
    @staticmethod
    def Rz(angle):
        return Rotation.from_euler('xyz', [0, 0, angle], degrees=False).as_matrix()
    
    @staticmethod
    def yaw(Rz : np.ndarray):
        return Rotation.from_matrix(Rz).as_euler('xyz')[2]
    
    @staticmethod
    def homogeneous(vector : np.ndarray):
        return np.vstack([vector.reshape(-1, 1), np.ones((1, 1))])
    
    @staticmethod
    def from_homogeneous(vector : np.ndarray):
        return vector.reshape(-1, 1)[:-1, :]
    
    def __init__(self, R : np.array, t : np.array):
        self.T = np.zeros((4, 4))
        self.T[:3, :3] = R
        self.T[:3, 3] = t.flatten()
        self.T[3, 3] = 1
        
    def __str__(self) -> str:
        return f'{np.round(self.T, 3)}'
    
    def __mul__(self, pt : np.array):
        return self.T @ pt.reshape(-1, 1)
    
    def __matmul__(self, other):
        if type(other) is Transform3D:
            T = self.T @ other.T
            return Transform3D(T[:3, :3], T[:3, 3])
        else:
            return self.T @ other
    
    def __array__(self, dtype=None):
        if dtype is not None:
            return np.asarray(self.T, dtype=dtype)
        return self.T
    
    def copy(self):
        return Transform3D(self.R.copy(), self.t.copy())
    
    @property
    def inv(self):
        T = np.linalg.inv(self.T)
        return Transform3D(T[:3, :3], T[:3, 3])
    
    @property
    def numpy(self):
        return self.T
    
    @property
    def q(self):
        q = np.array(Rotation.from_matrix(self.T[:3, :3]).as_quat(scalar_first=False)).reshape(-1, 1)
        return q / np.linalg.norm(q)
    
    @property
    def R(self):
        return self.T[:3, :3]
    
    @property
    def t(self):
        return self.T[:3, 3].reshape(-1, 1)
    
    @property
    def shape(self):
        return self.T.shape
    
    def scale(self, s):
        self.T[:3, 3] = self.T[:3, 3] * s
    