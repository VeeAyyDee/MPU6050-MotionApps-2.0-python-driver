import math

class Quaternion:
    def __init__(self, w=1.0, x=0.0, y=0.0, z=0.0):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def get_product(self, q):
        """Multiply this quaternion with another quaternion."""
        return Quaternion(
            self.w * q.w - self.x * q.x - self.y * q.y - self.z * q.z,  # new w
            self.w * q.x + self.x * q.w + self.y * q.z - self.z * q.y,  # new x
            self.w * q.y - self.x * q.z + self.y * q.w + self.z * q.x,  # new y
            self.w * q.z + self.x * q.y - self.y * q.x + self.z * q.w   # new z
        )

    def get_conjugate(self):
        """Return the conjugate of the quaternion."""
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def get_magnitude(self):
        """Calculate the magnitude of the quaternion."""
        return math.sqrt(self.w ** 2 + self.x ** 2 + self.y ** 2 + self.z ** 2)

    def normalize(self):
        """Normalize the quaternion in place."""
        m = self.get_magnitude()
        if m > 0:
            self.w /= m
            self.x /= m
            self.y /= m
            self.z /= m

    def get_normalized(self):
        """Return a new normalized quaternion."""
        r = Quaternion(self.w, self.x, self.y, self.z)
        r.normalize()
        return r
    
    def __str__(self):
        """Return a string representation of the quaternion with fixed precision."""
        return f"({self.w:.2f}, {self.x:.2f}, {self.y:.2f}, {self.z:.2f})"


class Vector3:
    def __init__(self, nx=0, ny=0, nz=0):
        self.x = nx
        self.y = ny
        self.z = nz

    def get_magnitude(self):
        """Calculate the magnitude of the vector."""
        return math.sqrt(self.x ** 2 + self.y ** 2 + self.z ** 2)

    def normalize(self):
        """Normalize the vector in place."""
        m = self.get_magnitude()
        if m > 0:
            self.x /= m
            self.y /= m
            self.z /= m

    def get_normalized(self):
        """Return a new normalized vector."""
        r = Vector3(self.x, self.y, self.z)
        r.normalize()
        return r

    def rotate(self, q):
        """Rotate the vector by a quaternion."""
        # Create a quaternion representation of the vector
        p = Quaternion(0, self.x, self.y, self.z)

        # Quaternion multiplication: q * p
        p = q.get_product(p)

        # Quaternion multiplication: p * conj(q)
        p = p.get_product(q.get_conjugate())

        # Update the vector components with the rotated quaternion parts
        self.x = p.x
        self.y = p.y
        self.z = p.z

    def get_rotated(self, q):
        """Return a new vector that is the result of rotating this vector by a quaternion."""
        r = Vector3(self.x, self.y, self.z)
        r.rotate(q)
        return r