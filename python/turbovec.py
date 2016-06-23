import rospy
import time
from math import sqrt
from turbotrig import turbosin, turbocos

def dot(v1, v2):
    return (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2])/1000

def cross(u, v):
    return [(u[1]*v[2] - u[2]*v[1])/1000,
            (u[2]*v[0] - u[0]*v[2])/1000,
            (u[0]*v[1] - u[1]*v[0])/1000]

def scalar_multiply(s, v):
    return [s*v[0]/1000, s*v[1]/1000, s*v[2]/1000]


def vector_add(u, v):
    return [u[0] + v[0],
            u[1] + v[1],
            u[2] + v[2]]

def vector_sub(u, v):
    return [u[0] - v[0],
            u[1] - v[1],
            u[2] - v[2]]

def sqred_norm(v):
    return v[0]*v[0] + v[1]*v[1] + v[2]*v[2]

def vector_normalize(u):
    # convert to floats for vector_normalize
    recipNorm = turboInvSqrt((u[0]*u[0] + u[1]*u[1] + u[2]*u[2])/1000000.0)
    recipNorm = int(1000*recipNorm)
    return scalar_multiply(recipNorm, u)

def quaternion_normalize(q):
    # convert to floats and back for turboInvSqrt
    recipNorm = turboInvSqrt((q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]) / 1000000.0)
    recipNorm = int(1000 * recipNorm)
    return [recipNorm*q[0]/1000, recipNorm*q[1]/1000, recipNorm*q[2]/1000, recipNorm*q[3]/1000]

def quat_multiply(q1, q2):
    s1 = q1[0]
    s2 = q2[0]

    v1 = [q1[1], q1[2], q1[3]]
    v2 = [q2[1], q2[2], q2[3]]

    w = (s1*s2)/1000 - dot(v1, v2)
    # xyz = s1*v2 + s2*v1 - v1 x v2)
    xyz = vector_sub(vector_add(scalar_multiply(s1,v2), scalar_multiply(s2,v1)), cross(v1,v2))
    return quaternion_normalize([w, xyz[0], xyz[1], xyz[2]])


def quat_between_vectors(u, v):
    u = vector_normalize(u)
    v = vector_normalize(v)
    half = vector_add(u, v)
    vector_normalize(half)

    w = dot(u,half)
    xyz = cross(u, v)

    return quaternion_normalize([w, xyz[0], xyz[1], xyz[2]])

def quat_inverse(q):
    return [-q[0], q[1], q[2], q[3]]

def turboInvSqrt(x):
    return x**(-0.5)




