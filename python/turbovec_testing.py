from turbovec import *

# Dot product and Cross Product Testing
v1 = [1000, 0, 0]
v2 = [0, 2000, 0]

v1_actual = [v1[0]/1000.0, v1[1]/1000.0]
v2_actual = [v2[0]/1000.0, v2[1]/1000.0]

print "dot product %d" % dot(v1, v2)
print "cross product %s" % str(cross(v1,v2))


# Scalar Multiply, add and subtract
print "scalar_multiply %s" % str(scalar_multiply(1500, v2))
print "vector_add %s" % str(vector_add(v1, v2))
print "vector_sub %s" % str(vector_sub(v1, v2))

# Normalize
print "normalize %s" % str(vector_normalize(v1))
print "normalize %s" % str(vector_normalize(v2))

# Quat Between Vectors
q = quat_between_vectors(v1, v2)
print "quat between vectors %s" % str(q)

qinv = quat_inverse(q)
print "quat inverse %s" % str(qinv)

qdiff = quat_multiply(qinv, q)
print "qdiff %s" % str(qdiff)





