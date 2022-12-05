from spatialmath.pose2d import *
import matplotlib.pyplot as plt 

robotpose = """Value x1: (gtsam::Pose2)
(-2.3673e-12, 4.76497e-12, 1.42138e-12)

Value x2: (gtsam::Pose2)
(0.105581, 0.00102021, 0.00154999)

Value x3: (gtsam::Pose2)
(0.218714, 0.00120562, 0.00342984)

Value x4: (gtsam::Pose2)
(0.331966, -0.00292976, 0.00613628)

Value x5: (gtsam::Pose2)
(0.428838, -0.00430583, 0.0102276)

Value x6: (gtsam::Pose2)
(0.550445, -0.00305818, 0.0107713)

Value x7: (gtsam::Pose2)
(0.675079, 0.00281329, 0.0097599)

Value x8: (gtsam::Pose2)
(0.790451, 0.013046, 0.00530321)

Value x9: (gtsam::Pose2)
(0.900963, 0.0138567, 0.00726388)

Value x10: (gtsam::Pose2)
(1.0254, 0.0167434, 0.00935672)

Value x11: (gtsam::Pose2)
(1.13684, 0.0169412, 0.00793073)

Value x12: (gtsam::Pose2)
(1.23773, 0.0181017, -0.0107628)

Value x13: (gtsam::Pose2)
(1.33485, 0.0137817, -0.0440898)

Value x14: (gtsam::Pose2)
(1.45925, 0.00904284, -0.0796039)

Value x15: (gtsam::Pose2)
(1.53725, 0.00382521, -0.0783491)

Value x16: (gtsam::Pose2)
(1.68941, -0.0122125, -0.0783938)

Value x17: (gtsam::Pose2)
(1.78196, -0.0219385, -0.0791442)

Value x18: (gtsam::Pose2)
(1.89472, -0.0468467, -0.0885777)

Value x19: (gtsam::Pose2)
(2.00188, -0.0597851, -0.0844205)

Value x20: (gtsam::Pose2)
(2.09672, -0.0590521, -0.0595189)

Value x21: (gtsam::Pose2)
(2.20786, -0.0724601, 0.0470299)

Value x22: (gtsam::Pose2)
(2.33407, -0.0707366, 0.188724)

Value x23: (gtsam::Pose2)
(2.46319, -0.0338565, 0.335358)

Value x24: (gtsam::Pose2)
(2.5754, 0.00646844, 0.484883)

Value x25: (gtsam::Pose2)
(2.68109, 0.0637003, 0.650972)

Value x26: (gtsam::Pose2)
(2.77398, 0.143945, 0.82329)

Value x27: (gtsam::Pose2)
(2.82938, 0.182981, 0.937617)

Value x28: (gtsam::Pose2)
(2.87372, 0.268289, 1.07437)

Value x29: (gtsam::Pose2)
(2.85608, 0.337901, 1.19106)

Value x30: (gtsam::Pose2)
(2.87078, 0.418803, 1.31618)

Value x31: (gtsam::Pose2)
(2.87617, 0.51828, 1.46548)

Value x32: (gtsam::Pose2)
(2.88638, 0.606588, 1.60082)

Value x33: (gtsam::Pose2)
(2.8892, 0.709395, 1.76742)

Value x34: (gtsam::Pose2)
(2.85732, 0.793628, 1.90641)

Value x35: (gtsam::Pose2)
(2.78938, 0.870266, 2.05167)

Value x36: (gtsam::Pose2)
(2.71908, 0.943901, 2.19613)

Value x37: (gtsam::Pose2)
(2.61883, 0.996632, 2.32878)

Value x38: (gtsam::Pose2)
(2.55472, 1.06149, 2.48357)

Value x39: (gtsam::Pose2)
(2.47061, 1.18112, 2.70078)

Value x40: (gtsam::Pose2)
(2.38952, 1.26523, 2.88619)

Value x41: (gtsam::Pose2)
(2.27495, 1.29366, 3.0637)

Value x42: (gtsam::Pose2)
(2.16922, 1.30445, -3.05173)

Value x43: (gtsam::Pose2)
(2.06593, 1.28889, -2.89402)

Value x44: (gtsam::Pose2)
(1.96021, 1.26301, -2.73904)

Value x45: (gtsam::Pose2)
(1.86921, 1.18663, -2.57319)

Value x46: (gtsam::Pose2)
(1.78394, 1.14797, -2.42693)

Value x47: (gtsam::Pose2)
(1.70593, 1.062, -2.2518)

Value x48: (gtsam::Pose2)
(1.63847, 0.950356, -2.06797)

Value x49: (gtsam::Pose2)
(1.60899, 0.886949, -1.94054)

Value x50: (gtsam::Pose2)
(1.57454, 0.756813, -1.74194)

Value x51: (gtsam::Pose2)
(1.53179, 0.650165, -1.58637)

Value x52: (gtsam::Pose2)
(1.54602, 0.530953, -1.40804)

Value x53: (gtsam::Pose2)
(1.5858, 0.410613, -1.22508)

Value x54: (gtsam::Pose2)
(1.64473, 0.305805, -1.043)

Value x55: (gtsam::Pose2)
(1.69675, 0.211455, -0.878675)

Value x56: (gtsam::Pose2)
(1.79073, 0.132374, -0.698188)

Value x57: (gtsam::Pose2)
(1.8682, 0.0866754, -0.509233)

Value x58: (gtsam::Pose2)
(1.96061, 0.0510557, -0.348305)

Value x59: (gtsam::Pose2)
(2.06114, 0.0245817, -0.18722)

Value x60: (gtsam::Pose2)
(2.21051, 0.0146927, 0.0279322)

Value x61: (gtsam::Pose2)
(2.32077, 0.025218, 0.209917)

Value x62: (gtsam::Pose2)
(2.42667, 0.0616494, 0.37906)

Value x63: (gtsam::Pose2)
(2.50097, 0.102604, 0.530287)

Value x64: (gtsam::Pose2)
(2.59639, 0.197768, 0.73602)

Value x65: (gtsam::Pose2)
(2.66492, 0.277716, 0.909642)

Value x66: (gtsam::Pose2)
(2.74024, 0.372476, 1.08639)

Value x67: (gtsam::Pose2)
(2.80064, 0.476135, 1.26829)

Value x68: (gtsam::Pose2)
(2.92292, 0.655849, 1.52266)

Value x69: (gtsam::Pose2)
(2.96186, 0.815344, 1.70601)

Value x70: (gtsam::Pose2)
(2.9587, 0.937674, 1.7126)

Value x71: (gtsam::Pose2)
(2.95084, 1.0294, 1.71661)

Value x72: (gtsam::Pose2)
(2.92831, 1.15867, 1.7184)

Value x73: (gtsam::Pose2)
(2.9153, 1.26852, 1.72348)

Value x74: (gtsam::Pose2)
(2.90095, 1.40128, 1.71742)

Value x75: (gtsam::Pose2)
(2.87988, 1.50584, 1.71904)

Value x76: (gtsam::Pose2)
(2.86501, 1.60545, 1.71904)

"""

landmarks = """Value l1: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	1.6114;
	-0.669501
]

Value l2: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	2.14665;
	0.650617
]

Value l3: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	1.15222;
	1.28525
]

Value l4: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	2.09859;
	1.98667
]

Value l5: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	3.0091;
	-1.44942
]

Value l6: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	3.50319;
	-0.452985
]

Value l7: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	4.36342;
	0.709169
]

Value l8: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	4.14283;
	1.47617
]

Value l9: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	3.1122;
	-1.34018
]

Value l10: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	3.68363;
	2.20303
]

"""

cores = """l1 x1
l1 x1
l2 x1
l2 x1
l3 x1
l3 x1
l1 x2
l2 x2
l3 x2
l1 x3
l2 x3
l3 x3
l1 x4
l2 x4
l4 x4
l4 x4
l3 x4
l1 x5
l2 x5
l4 x5
l3 x5
l1 x6
l2 x6
l4 x6
l3 x6
l1 x7
l2 x7
l4 x7
l3 x7
l1 x8
l5 x8
l5 x8
l2 x8
l4 x8
l3 x8
l1 x9
l5 x9
l6 x9
l6 x9
l2 x9
l4 x9
l3 x9
l1 x10
l5 x10
l6 x10
l2 x10
l4 x10
l3 x10
l1 x11
l5 x11
l6 x11
l2 x11
l4 x11
l3 x11
l1 x12
l5 x12
l6 x12
l2 x12
l4 x12
l3 x12
l1 x13
l5 x13
l6 x13
l2 x13
l4 x13
l3 x13
l1 x14
l5 x14
l6 x14
l2 x14
l4 x14
l3 x14
l1 x15
l5 x15
l6 x15
l2 x15
l4 x15
l3 x15
l1 x16
l5 x16
l6 x16
l2 x16
l4 x16
l3 x16
l1 x17
l5 x17
l6 x17
l2 x17
l4 x17
l3 x17
l1 x18
l5 x18
l6 x18
l7 x18
l7 x18
l2 x18
l1 x19
l5 x19
l6 x19
l7 x19
l2 x19
l5 x20
l6 x20
l7 x20
l2 x20
l5 x21
l6 x21
l7 x21
l8 x21
l8 x21
l4 x21
l2 x21
l5 x22
l6 x22
l7 x22
l8 x22
l4 x22
l2 x22
l3 x22
l5 x23
l6 x23
l7 x23
l8 x23
l4 x23
l2 x23
l3 x23
l5 x24
l6 x24
l7 x24
l8 x24
l4 x24
l2 x24
l5 x25
l6 x25
l7 x25
l8 x25
l4 x25
l2 x25
l6 x26
l7 x26
l8 x26
l4 x26
l2 x26
l6 x27
l7 x27
l8 x27
l4 x27
l3 x27
l2 x27
l6 x28
l7 x28
l8 x28
l4 x28
l3 x28
l2 x28
l7 x29
l8 x29
l4 x29
l3 x29
l2 x29
l7 x30
l8 x30
l4 x30
l3 x30
l2 x30
l7 x31
l8 x31
l4 x31
l3 x31
l2 x31
l7 x32
l8 x32
l4 x32
l3 x32
l2 x32
l7 x33
l8 x33
l4 x33
l3 x33
l2 x33
l1 x33
l7 x34
l8 x34
l4 x34
l3 x34
l2 x34
l1 x34
l8 x35
l4 x35
l3 x35
l2 x35
l1 x35
l8 x36
l4 x36
l3 x36
l2 x36
l1 x36
l8 x37
l4 x37
l3 x37
l2 x37
l8 x38
l4 x38
l3 x38
l2 x38
l4 x39
l3 x39
l2 x39
l4 x40
l3 x40
l1 x40
l2 x40
l4 x41
l3 x41
l1 x41
l2 x41
l4 x42
l3 x42
l1 x42
l2 x42
l6 x42
l4 x43
l3 x43
l1 x43
l2 x43
l6 x43
l3 x44
l1 x44
l2 x44
l6 x44
l3 x45
l1 x45
l2 x45
l3 x46
l1 x46
l2 x46
l3 x47
l1 x47
l6 x47
l2 x47
l3 x48
l1 x48
l6 x48
l2 x48
l3 x49
l1 x49
l6 x49
l2 x49
l1 x50
l9 x50
l9 x50
l6 x50
l2 x50
l1 x51
l9 x51
l6 x51
l2 x51
l1 x52
l9 x52
l6 x52
l2 x52
l1 x53
l9 x53
l6 x53
l2 x53
l1 x54
l9 x54
l6 x54
l2 x54
l1 x55
l9 x55
l6 x55
l2 x55
l1 x56
l9 x56
l6 x56
l2 x56
l4 x56
l1 x57
l9 x57
l6 x57
l8 x57
l2 x57
l1 x58
l9 x58
l6 x58
l8 x58
l2 x58
l1 x59
l9 x59
l6 x59
l8 x59
l2 x59
l9 x60
l6 x60
l8 x60
l10 x60
l10 x60
l4 x60
l2 x60
l9 x61
l6 x61
l8 x61
l10 x61
l4 x61
l2 x61
l9 x62
l6 x62
l8 x62
l10 x62
l4 x62
l2 x62
l3 x62
l9 x63
l6 x63
l8 x63
l10 x63
l4 x63
l2 x63
l9 x64
l6 x64
l8 x64
l10 x64
l4 x64
l2 x64
l6 x65
l8 x65
l10 x65
l4 x65
l2 x65
l6 x66
l8 x66
l10 x66
l4 x66
l3 x66
l2 x66
l6 x67
l8 x67
l10 x67
l4 x67
l3 x67
l2 x67
l8 x68
l10 x68
l4 x68
l3 x68
l2 x68
l8 x69
l10 x69
l4 x69
l3 x69
l2 x69
l8 x70
l10 x70
l4 x70
l3 x70
l2 x70
l8 x71
l10 x71
l4 x71
l3 x71
l2 x71
l8 x72
l10 x72
l4 x72
l3 x72
l2 x72
l8 x73
l10 x73
l4 x73
l3 x73
l2 x73
l8 x74
l10 x74
l4 x74
l3 x74
l8 x75
l10 x75
l4 x75
l3 x75"""

# SE2(*[float(j) for j in i[1:-1].split(', ')]) 
poses = [SE2(*[float(j) for j in i[1:-1].split(', ')]) for i in robotpose.split('\n') if ',' in i]

xs = [0]
ys = [0]

lxs = []
lys = []

a = 0
for i in landmarks.split("\n"):
    if a % 6 == 2:
        lxs.append(float(i.replace(';', '')))
    if a % 6 == 3:
        lys.append(float(i))

    a+=1

for i in poses:
    # robot_poses.append(i*robot_poses[-1])
    xs.append(i.xyt()[0])
    ys.append(i.xyt()[1])

for i in cores.split('\n'):

	ld = int(i.split(' ')[0].replace('l', '')) - 1
	p = int(i.split(' ')[1].replace('x', '')) - 1

	plt.plot([xs[p], lxs[ld]], [ys[p], lys[ld]], color='green', linewidth = '0.5', alpha=0.4)

for i in range(1,len(xs)):
	plt.plot([xs[i], xs[i-1]], [ys[i],ys[i-1]], color='blue', linewidth = '0.5', alpha=0.5)

plt.scatter(xs, ys)
plt.scatter(lxs, lys)
plt.show()

