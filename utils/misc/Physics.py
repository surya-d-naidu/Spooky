import numpy as np

class Vector3:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.vector = np.array([x, y, z])

    def __repr__(self):
        return f"Vector3({self.vector[0]}, {self.vector[1]}, {self.vector[2]})"

    def __add__(self, other):
        return Vector3(*self.vector + other.vector)

    def __sub__(self, other):
        return Vector3(*self.vector - other.vector)

    def __mul__(self, scalar):
        return Vector3(*self.vector * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def dot(self, other):
        return np.dot(self.vector, other.vector)

    def cross(self, other):
        return Vector3(*np.cross(self.vector, other.vector))

    def magnitude(self):
        return np.linalg.norm(self.vector)

    def normalize(self):
        mag = self.magnitude()
        if mag > 0:
            return Vector3(*self.vector / mag)
        return Vector3(0, 0, 0)

class Quaternion:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.quaternion = np.array([w, x, y, z])

    def __repr__(self):
        return f"Quaternion({self.quaternion[0]}, {self.quaternion[1]}, {self.quaternion[2]}, {self.quaternion[3]})"

    def conjugate(self):
        return Quaternion(self.quaternion[0], -self.quaternion[1], -self.quaternion[2], -self.quaternion[3])

    def magnitude(self):
        return np.linalg.norm(self.quaternion)

    def normalize(self):
        mag = self.magnitude()
        if mag > 0:
            return Quaternion(*(self.quaternion / mag))
        return Quaternion()

    def multiply(self, other):
        q1 = self.quaternion
        q2 = other.quaternion
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return Quaternion(
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        )

    def rotate(self, vector):
        qv = Quaternion(0, *vector.vector)
        return self.multiply(qv).multiply(self.conjugate())

    def dot(self, other):
        return np.dot(self.quaternion, other)

class Transform:
    def __init__(self, position=None, rotation=None, scale=None):
        self.position = position if position is not None else Vector3()
        self.rotation = rotation if rotation is not None else Quaternion()
        self.scale = scale if scale is not None else Vector3(1, 1, 1)

    def __repr__(self):
        return f"Transform(Position={self.position}, Rotation={self.rotation}, Scale={self.scale})"

    def get_forward(self):
        forward = Vector3(0, 0, 1)
        return self.rotation.rotate(forward)

    def get_right(self):
        right = Vector3(1, 0, 0)
        return self.rotation.rotate(right)

    def get_up(self):
        up = Vector3(0, 1, 0)
        return self.rotation.rotate(up)

    def translate(self, translation):
        self.position += translation

    def rotate(self, quaternion):
        self.rotation = self.rotation.multiply(quaternion).normalize()

    def scale_transform(self, scale_factor):
        self.scale *= scale_factor
