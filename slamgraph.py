from spatialmath.pose2d import *
import matplotlib.pyplot as plt 

robotpose = """Value x1: (gtsam::Pose2)
(-0.0351812979, 0.0133339594, -0.00113466159)

Value x2: (gtsam::Pose2)
(0.0800188576, 0.012970932, -0.00340256004)

Value x3: (gtsam::Pose2)
(0.212257517, 0.0130801954, -0.00851992557)

Value x4: (gtsam::Pose2)
(0.330515894, 0.0088092074, -0.0110401484)

Value x5: (gtsam::Pose2)
(0.430847804, 0.00551042363, -0.0132991973)

Value x6: (gtsam::Pose2)
(0.521491426, -0.00404791912, -0.0137416864)

Value x7: (gtsam::Pose2)
(0.638601656, -0.0100234688, -0.0162998183)

Value x8: (gtsam::Pose2)
(0.798453143, -0.0130630709, -0.0196436501)

Value x9: (gtsam::Pose2)
(0.88650148, -0.024907591, -0.0200975813)

Value x10: (gtsam::Pose2)
(1.00720451, -0.0324558756, -0.0227468721)

Value x11: (gtsam::Pose2)
(1.11337121, -0.0391061138, -0.0248478133)

Value x12: (gtsam::Pose2)
(1.2360556, -0.0409336899, -0.0274479115)

Value x13: (gtsam::Pose2)
(1.31661622, -0.0468659091, -0.0292488943)

Value x14: (gtsam::Pose2)
(1.3773926, -0.0533113083, -0.0321538987)

Value x15: (gtsam::Pose2)
(1.51554118, -0.0492938141, -0.0378033062)

Value x16: (gtsam::Pose2)
(1.59878773, -0.0525635825, -0.0467329162)

Value x17: (gtsam::Pose2)
(1.69644375, -0.064219462, -0.0401847195)

Value x18: (gtsam::Pose2)
(1.77133396, -0.076245993, -0.0193326176)

Value x19: (gtsam::Pose2)
(1.91756999, -0.0792913297, 0.0363653035)

Value x20: (gtsam::Pose2)
(2.01954549, -0.0790465207, 0.0884456817)

Value x21: (gtsam::Pose2)
(2.11726132, -0.0620650769, 0.128847629)

Value x22: (gtsam::Pose2)
(2.23287866, -0.0388585974, 0.154705001)

Value x23: (gtsam::Pose2)
(2.35346752, -0.018795626, 0.166947731)

Value x24: (gtsam::Pose2)
(2.45062299, -0.00577954821, 0.17423376)

Value x25: (gtsam::Pose2)
(2.51412438, 0.00840265368, 0.178889127)

Value x26: (gtsam::Pose2)
(2.59188699, 0.022846387, 0.181053811)

Value x27: (gtsam::Pose2)
(2.71499624, 0.0388399493, 0.178903461)

Value x28: (gtsam::Pose2)
(2.80376799, 0.0460599532, 0.201922404)

Value x29: (gtsam::Pose2)
(2.90260109, 0.0591219622, 0.239524056)

Value x30: (gtsam::Pose2)
(3.02144448, 0.0943964461, 0.297558725)

Value x31: (gtsam::Pose2)
(3.20890406, 0.171664057, 0.389142013)

Value x32: (gtsam::Pose2)
(3.31296399, 0.209821716, 0.461044982)

Value x33: (gtsam::Pose2)
(3.45221868, 0.286138957, 0.53874545)

Value x34: (gtsam::Pose2)
(3.54084379, 0.331111293, 0.599542189)

Value x35: (gtsam::Pose2)
(3.65573375, 0.417334281, 0.669169816)

Value x36: (gtsam::Pose2)
(3.7509027, 0.504164873, 0.737084195)

Value x37: (gtsam::Pose2)
(3.83108271, 0.583954657, 0.794811898)

Value x38: (gtsam::Pose2)
(3.93196899, 0.697115771, 0.876346606)

Value x39: (gtsam::Pose2)
(4.03163581, 0.825643511, 0.96423208)

Value x40: (gtsam::Pose2)
(4.10845644, 0.963091584, 1.05530002)

Value x41: (gtsam::Pose2)
(4.15699726, 1.06531232, 1.09432556)

Value x42: (gtsam::Pose2)
(4.1765568, 1.07184203, 1.10561734)

Value x43: (gtsam::Pose2)
(4.11340627, 0.929093351, 1.04212352)

Value x44: (gtsam::Pose2)
(4.06337079, 0.778078, 0.975188203)

Value x45: (gtsam::Pose2)
(3.98401173, 0.675337119, 0.907562386)

Value x46: (gtsam::Pose2)
(3.93179422, 0.598371668, 0.858814247)

Value x47: (gtsam::Pose2)
(3.8390422, 0.47453639, 0.783529769)

Value x48: (gtsam::Pose2)
(3.76490572, 0.383988255, 0.738660941)

Value x49: (gtsam::Pose2)
(3.7038388, 0.335412064, 0.679933599)

Value x50: (gtsam::Pose2)
(3.64165563, 0.282401435, 0.631515448)

Value x51: (gtsam::Pose2)
(3.56511674, 0.221383473, 0.577683038)

Value x52: (gtsam::Pose2)
(3.48626185, 0.161674494, 0.523539899)

Value x53: (gtsam::Pose2)
(3.41658839, 0.120579463, 0.474083325)

Value x54: (gtsam::Pose2)
(3.33070486, 0.0713874988, 0.416759451)

Value x55: (gtsam::Pose2)
(3.22804299, 0.0105223479, 0.353600578)

Value x56: (gtsam::Pose2)
(3.12218394, -0.0385086588, 0.294310342)

Value x57: (gtsam::Pose2)
(3.05463812, -0.0571032153, 0.250373124)

Value x58: (gtsam::Pose2)
(2.94792637, -0.0940366904, 0.189996111)

Value x59: (gtsam::Pose2)
(2.83040009, -0.127065333, 0.124718318)

Value x60: (gtsam::Pose2)
(2.7602949, -0.138396875, 0.0790552301)

Value x61: (gtsam::Pose2)
(2.67789846, -0.146481154, 0.0315440526)

Value x62: (gtsam::Pose2)
(2.57505473, -0.155167951, -0.0281682623)

Value x63: (gtsam::Pose2)
(2.43591006, -0.162117597, -0.103329135)

Value x64: (gtsam::Pose2)
(2.34450081, -0.156246393, -0.153670778)

Value x65: (gtsam::Pose2)
(2.26127586, -0.162230422, -0.1879904)

Value x66: (gtsam::Pose2)
(2.17598515, -0.151537125, -0.209012477)

Value x67: (gtsam::Pose2)
(2.07390718, -0.12625758, -0.22030298)

Value x68: (gtsam::Pose2)
(1.97473226, -0.116037384, -0.201415608)

Value x69: (gtsam::Pose2)
(1.89102464, -0.0974768013, -0.192471906)

Value x70: (gtsam::Pose2)
(1.80445553, -0.0694127152, -0.174638067)

Value x71: (gtsam::Pose2)
(1.71233889, -0.0419674198, -0.162396461)

Value x72: (gtsam::Pose2)
(1.63618125, -0.0221055384, -0.158173317)

Value x73: (gtsam::Pose2)
(1.50335565, 0.00325429037, -0.156657314)

Value x74: (gtsam::Pose2)
(1.41960703, 0.0160180988, -0.146438368)

Value x75: (gtsam::Pose2)
(1.30947468, 0.017767658, -0.117112313)

Value x76: (gtsam::Pose2)
(1.24058484, 0.0294815757, -0.0929626154)

Value x77: (gtsam::Pose2)
(1.11235898, 0.0723626803, -0.0480314206)

Value x78: (gtsam::Pose2)
(0.957838641, 0.089605921, -0.0297253849)

Value x79: (gtsam::Pose2)
(0.823513133, 0.0965379272, -0.0193876486)

Value x80: (gtsam::Pose2)
(0.641659803, 0.104548866, -0.00537450671)

Value x81: (gtsam::Pose2)
(0.540718405, 0.110523677, -0.000717220557)

Value x82: (gtsam::Pose2)
(0.410946875, 0.110436952, 0.00680784446)

Value x83: (gtsam::Pose2)
(0.398470747, 0.116263419, 0.00903027851)

Value x84: (gtsam::Pose2)
(0.555230588, 0.107500661, 0.0788519154)

Value x85: (gtsam::Pose2)
(0.724444587, 0.12781458, 0.152012983)

Value x86: (gtsam::Pose2)
(0.874815953, 0.149739836, 0.225715087)

Value x87: (gtsam::Pose2)
(0.992344285, 0.169911806, 0.287505988)

Value x88: (gtsam::Pose2)
(1.08625816, 0.204210348, 0.334715936)

Value x89: (gtsam::Pose2)
(1.16391138, 0.231830279, 0.379114206)

Value x90: (gtsam::Pose2)
(1.27747171, 0.286023675, 0.441682496)

Value x91: (gtsam::Pose2)
(1.38931449, 0.341483902, 0.51342945)

Value x92: (gtsam::Pose2)
(1.4815527, 0.404789996, 0.548103789)

Value x93: (gtsam::Pose2)
(1.57561604, 0.467460849, 0.56814853)

Value x94: (gtsam::Pose2)
(1.63839999, 0.518326155, 0.566017049)

Value x95: (gtsam::Pose2)
(1.72432705, 0.572658055, 0.557278049)

"""

landmarks = """Value l1: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	1.59205997;
	-0.653035998
]

Value l2: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	1.77280998;
	1.96800995
]

Value l3: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	3.09865999;
	0.655031025
]

Value l4: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	4.54379988;
	-0.745512009
]

Value l5: (Eigen::Matrix<double, 2, 1, 0, 2, 1>)
[
	4.42332983;
	2.10600996
]

"""

cores = """l1 x1
l1 x2
l2 x2
l1 x3
l2 x3
l1 x4
l2 x4
l1 x5
l2 x5
l1 x6
l3 x6
l2 x6
l1 x7
l3 x7
l2 x7
l1 x8
l3 x8
l2 x8
l1 x9
l3 x9
l2 x9
l1 x10
l3 x10
l2 x10
l1 x11
l3 x11
l2 x11
l1 x12
l3 x12
l2 x12
l1 x13
l3 x13
l2 x13
l1 x14
l3 x14
l2 x14
l1 x15
l3 x15
l2 x15
l1 x16
l3 x16
l2 x16
l1 x17
l3 x17
l2 x17
l1 x18
l3 x18
l2 x18
l4 x19
l3 x19
l2 x19
l4 x20
l3 x20
l2 x20
l4 x21
l3 x21
l2 x21
l4 x22
l3 x22
l2 x22
l4 x23
l3 x23
l2 x23
l4 x24
l3 x24
l2 x24
l4 x25
l3 x25
l2 x25
l4 x26
l3 x26
l2 x26
l4 x27
l3 x27
l2 x27
l4 x28
l5 x28
l3 x28
l2 x28
l4 x29
l5 x29
l3 x29
l2 x29
l4 x30
l5 x30
l3 x30
l2 x30
l4 x31
l5 x31
l3 x31
l4 x32
l5 x32
l3 x32
l4 x33
l5 x33
l2 x33
l3 x33
l4 x34
l5 x34
l2 x34
l3 x34
l4 x35
l5 x35
l2 x35
l4 x36
l5 x36
l4 x37
l5 x37
l4 x38
l5 x38
l5 x39
l5 x40
l5 x41
l5 x42
l5 x43
l5 x44
l4 x45
l5 x45
l4 x46
l5 x46
l4 x47
l5 x47
l4 x48
l5 x48
l3 x48
l4 x49
l5 x49
l3 x49
l4 x50
l5 x50
l3 x50
l4 x51
l5 x51
l3 x51
l4 x52
l5 x52
l3 x52
l4 x53
l5 x53
l3 x53
l4 x54
l5 x54
l3 x54
l4 x55
l5 x55
l3 x55
l2 x55
l4 x56
l5 x56
l3 x56
l2 x56
l4 x57
l5 x57
l3 x57
l2 x57
l4 x58
l5 x58
l3 x58
l2 x58
l4 x59
l3 x59
l2 x59
l4 x60
l3 x60
l2 x60
l4 x61
l3 x61
l2 x61
l4 x62
l3 x62
l2 x62
l4 x63
l3 x63
l2 x63
l4 x64
l3 x64
l2 x64
l4 x65
l3 x65
l2 x65
l4 x66
l3 x66
l2 x66
l4 x67
l3 x67
l2 x67
l1 x68
l4 x68
l3 x68
l2 x68
l1 x69
l3 x69
l2 x69
l1 x70
l3 x70
l2 x70
l1 x71
l3 x71
l2 x71
l1 x72
l3 x72
l2 x72
l1 x73
l3 x73
l2 x73
l1 x74
l3 x74
l2 x74
l1 x75
l3 x75
l2 x75
l1 x76
l3 x76
l2 x76
l1 x77
l3 x77
l2 x77
l1 x78
l3 x78
l2 x78
l1 x79
l3 x79
l2 x79
l1 x80
l3 x80
l2 x80
l1 x81
l3 x81
l2 x81
l1 x82
l2 x82
l1 x83
l2 x83
l1 x84
l3 x84
l2 x84
l1 x85
l3 x85
l2 x85
l1 x86
l3 x86
l2 x86
l1 x87
l3 x87
l2 x87
l1 x88
l3 x88
l2 x88
l1 x89
l3 x89
l2 x89
l1 x90
l3 x90
l2 x90
l1 x91
l3 x91
l2 x91
l1 x92
l3 x92
l2 x92
l1 x93
l3 x93
l2 x93
l3 x94
l2 x94"""

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

print("{",end="")
for i in range(len(lxs)):
	print("{",end="")
	print(f"{lxs[i]},{lys[i]}",end="},")
print("}")
plt.scatter(xs, ys)
plt.scatter(lxs, lys)
plt.show()

