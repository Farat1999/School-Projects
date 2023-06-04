#!/usr/bin/python3
# -*- coding: utf-8 -*

import numpy as np
import scipy.io
import matplotlib.pyplot as plt
import random
import copy


# ___________________________________________________________
# Global variables
# -----------------------------------------------------------
# Load the data
Calib_Beads2D = scipy.io.loadmat('Calib_Beads2D.mat')
Calib_Beads3D = scipy.io.loadmat('Calib_Beads3D.mat')
Vertebrae2D = scipy.io.loadmat('Vertebrae2D.mat')

# ___________________________________________________________
# Function definitions
# -----------------------------------------------------------


def load_to_arrays(Calib_Beads2D, Calib_Beads3D, Vertebrae2D):
    beads_2d_lat = []
    beads_2d_lat_mat = Calib_Beads2D['Beads2D_LAT'][0]
    for i in range(47):
        beads_2d_lat.append([])
        beads_2d_lat[i].append(beads_2d_lat_mat[i][0][0])
        beads_2d_lat[i].append(beads_2d_lat_mat[i][1][0].astype(float))

    beads_2d_pa = []
    beads_2d_pa_mat = Calib_Beads2D['Beads2D_PA0'][0]
    for i in range(49):
        beads_2d_pa.append([])
        beads_2d_pa[i].append(beads_2d_pa_mat[i][0][0])
        beads_2d_pa[i].append(beads_2d_pa_mat[i][1][0].astype(float))

    beads_3d = []
    beads_3d_mat = Calib_Beads3D['Calib_Beads3D'][0]
    for i in range(110):
        beads_3d.append([])
        beads_3d[i].append(beads_3d_mat[i][0][0])
        beads_3d[i].append(beads_3d_mat[i][1][0].astype(float))

    vert_lat = Vertebrae2D['Vertebrae_LAT']
    vert_pa = Vertebrae2D['Vertebrae_PA0']

    lat_names = get_all_objects(beads_2d_lat)[:,0]
    pa_names = get_all_objects(beads_2d_pa)[:,0]

    index_pa = np.where(pa_names == 'B_3_5')
    pa_names = np.delete(pa_names, index_pa)
    index_pa = np.where(pa_names == 'A_4_1')
    pa_names = np.delete(pa_names, index_pa)
    # D35, C41

    return beads_2d_pa, beads_2d_lat, beads_3d, vert_lat, vert_pa, pa_names, lat_names


def get_all_objects(beads):
    objects = []
    for i in range(len(beads)):
        objects.append([beads[i][0],])
    return np.array(objects)


def get_object(beads, vert):
    """
    Description:
        Looks for an Object by its name.

    Arguments:
        1) beads - list of all the objects (i.e. struct array).
        2) vert  -	name of the requested object.

    Results:
        Index and bead.
    """

    for i in range(len(beads)):
        if beads[i][0] == vert:
            return i, beads[i][1].astype(float)

    return "Not found"


def get_vert(verts, vert_index, point_index):
    """
    Description:
        Return the names and u, v location of a vertebrae given an vert index
        and a point index
    """

    vert_name = verts[vert_index][0][0][0]
    vert_point_name = verts[vert_index][0][1][0][point_index][0][0]
    value = verts[vert_index][0][1][0][point_index][1][0].astype(float)
    return vert_name, vert_point_name, value


def calibrate(beads_2d, beads_3d, view, names=None):
    """
    Description:
        Find M in: [u,v] = M x [X,Y,Z]
    """

    # Q1.1
    if names is None:
        names = get_all_objects(beads_2d)

    n = len(names)

    A = np.zeros([2 * n, 11])
    b = np.zeros([2 * n,])
    for i, name in enumerate(names):

        _, uv = get_object(beads_2d, name)
        _, xyz = get_object(beads_3d, name)

        A[2 * i, :] = np.array([xyz[0], xyz[1], xyz[2], 1, 0, 0, 0, 0, -uv[0] * xyz[0], -uv[0] * xyz[1], -uv[0] * xyz[2]])
        A[2 * i + 1, :] = np.array([0,0,0,0, xyz[0], xyz[1], xyz[2], 1, -uv[1] * xyz[0], -uv[1] * xyz[1], -uv[1] * xyz[2]])

        b[2 * i] = uv[0]
        b[2 * i + 1] = uv[1]

    A_inv = np.linalg.inv(A.T @ A) @ A.T
    # A_inv= np.linalg.pinv(A)  # Same as the above line

    X = A_inv @ b
    X = np.append(X, 1)
    M = X.reshape(3, 4)

    # # Sanity check
    # for i in range(n):
    #     name = beads_2d[i][0][0]
    #     _, uv = get_object(beads_2d, name)
    #     _, xyz = get_object(beads_3d, name)
    #     xyz = np.append(xyz, 1)
    #     p = M @ xyz
    #     if not np.all(np.isclose(p[:2]/p[2], uv, atol=1, rtol=0.1)):
    #         print(f"{view}, {name} not close: {p/p[2]}, {uv}")
    #     else:
    #         print("good")
    return M


def find_params(M):
    """
    Desciption:
        Find the parameters of the system from an M matrix.
        The units seem to be millimeters (mm).
    """

    l = M.reshape(-1)

    d = -1 / (np.sqrt(l[8] ** 2 + l[9] ** 2 + l[10] ** 2))
    print(f"d: {d}")
    u0 = (l[0] * l[8] + l[1] * l[9] + l[2] * l[10]) * d ** 2
    print(f"u0: {u0}")
    v0 = (l[4] * l[8] + l[5] * l[9] + l[6] * l[10]) * d ** 2
    print(f"v0: {v0}")
    cu = np.sqrt((d ** 2) * (((u0 * l[8] - l[0]) ** 2) + ((u0 * l[9] - l[1]) ** 2) + ((u0 * l[10] - l[2]) ** 2)))
    print(f"cu: {cu}")
    cv = np.sqrt((d ** 2) * (((v0 * l[8] - l[4]) ** 2) + ((v0 * l[9] - l[5]) ** 2) + ((v0 * l[10] - l[6]) ** 2)))
    print(f"cv: {cv}")

    r11 = d / cu * (u0 * l[8] - l[0])
    r12 = d / cu * (u0 * l[9] - l[1])
    r13 = d / cu * (u0 * l[10] - l[2])
    r21 = d / cv * (v0 * l[8] - l[4])
    r22 = d / cv * (v0 * l[9] - l[5])
    r23 = d / cv * (v0 * l[10] - l[6])
    r31 = l[8] * d
    r32 = l[9] * d
    r33 = l[10] * d
    r = [r11, r12, r13, r21, r22, r23, r31, r32, r33]
    print(f"r: {r}")

    A = np.array([l[0], l[1], l[2], l[4], l[5], l[6], l[8], l[9], l[10]])
    A = A.reshape(3, 3)
    B = np.array([-l[3], -l[7], -1])
    A_inv = np.linalg.inv(A)
    xyz =  A_inv @ B
    print(f"xyz: {xyz}")


def reconstruct(M1_lat, M2_pa, vert_lat, vert_pa):
    """
    Description:
        Reconstruct 3d points from 2 x 2d views
    """

    l1 = M1_lat.reshape(-1)
    # print("L1: ",l1)
    l2 = M2_pa.reshape(-1)
    # print("L2: ",l2)
    output = np.zeros([102, 3])
    n_verts = len(vert_lat)
    n_points = 6

    for i_vert in range(n_verts):
        for i_point in range(n_points):

            A = np.zeros([4, 3])

            name, _, (u1, v1) = get_vert(vert_lat, i_vert, i_point)
            name2, _, (u2, v2) = get_vert(vert_pa, i_vert, i_point)

            A[0, :] = np.array([-u1 * l1[8] + l1[0], -u1 * l1[9] + l1[1], -u1 * l1[10] + l1[2]])
            A[1, :] = np.array([-v1 * l1[8] + l1[4], -v1 * l1[9] + l1[5], -v1 * l1[10] + l1[6]])
            A[2, :] = np.array([-u2 * l2[8] + l2[0], -u2 * l2[9] + l2[1], -u2 * l2[10] + l2[2]])
            A[3, :] = np.array([-v2 * l2[8] + l2[4], -v2 * l2[9] + l2[5], -v2 * l2[10] + l2[6]])

            A_inv = np.linalg.pinv(A)
            # A_inv = np.linalg.inv(A.T @ A) @ A.T
            B = np.array([u1 * l1[11] - l1[3], v1 * l1[11] - l1[7], u2 * l2[11] - l2[3], v2 * l2[11] - l2[7]])

            xyz = A_inv @ B

            output[i_vert * 6 + i_point] = xyz
            # print(f"Vert: {i_vert}, xyz: {xyz}, uv1: {[u1,v1]}, uv2: {[u2,v2]}")

    return output


def plot_3d(xyz):
    """
    Description:
        Plot different points in 3d
    """

    plt.figure()
    ax = plt.axes(projection='3d')
    n_vert = 102 / 6
    colors = plt.cm.rainbow(np.linspace(0, 1, int(n_vert)))
    color_a_vert = range(0,17,2)
    for i_vert in range(int(n_vert)):
        if i_vert in color_a_vert:
            ax.scatter3D(xyz[6 * i_vert:6 * (i_vert + 1), 0], xyz[6 * i_vert:6 * (i_vert + 1), 1], xyz[6 * i_vert:6 * (i_vert + 1), 2], color=colors[i_vert])
        else:
            ax.scatter3D(xyz[6 * i_vert:6 * (i_vert + 1), 0], xyz[6 * i_vert:6 * (i_vert + 1), 1], xyz[6 * i_vert:6 * (i_vert + 1), 2], color='gray')
    # a_vert = 12
    # ax.scatter3D(xyz[6*a_vert:6*(a_vert+1), 0], xyz[6*a_vert:6*(a_vert+1), 1], xyz[6*a_vert:6*(a_vert+1), 2])
    ax.set_box_aspect(aspect=[1,0.75,2])
    plt.show()


def rms(true_values, test_values):
    """
    All true_values
    Some test test_values
    """
    x = test_values[:, 0]
    x0 = true_values[:, 0]
    y = test_values[:, 1]
    y0 = true_values[:, 1]
    z = test_values[:, 2]
    z0 = true_values[:, 2]
    n = len(test_values)

    rms_x = np.sqrt(np.sum(((x - x0) ** 2) / n))
    rms_y = np.sqrt(np.sum(((y - y0) ** 2) / n))
    rms_z = np.sqrt(np.sum(((z - z0) ** 2) / n))

    rms_xyz = np.sqrt(np.sum((((x - x0) ** 2) + ((y - y0) ** 2) + ((z - z0) ** 2)) / n))

    return rms_x, rms_y, rms_z, rms_xyz


def plot_diff_number_beads(all_names, lat_names, pa_names, beads_2d_lat, beads_2d_pa, beads_3d, vert_lat, vert_pa, xyz0):
    # plot decreasing # of beads
    # set_corners = set(['A_1_1', 'A_1_6', 'A_4_2', 'A_4_6', 'B_1_1', 'B_1_5', 'B_5_1', 'B_5_5', 'C_1_1', 'C_1_6', 'C_4_2', 'C_4_6', 'D_1_1', 'D_1_5', 'D_5_1', 'D_5_5'])
    set_corners = set(['A_1_1', 'A_4_2', 'A_4_6', 'B_1_1', 'B_1_5', 'B_5_5', 'C_1_1', 'C_4_2', 'C_4_6', 'D_1_1', 'D_1_5', 'D_5_5'])

    set_all_beads = set(all_names)
    set_allowed_beads_remove = set_all_beads - set_corners

    rmss = []
    n_beads = 47
    min_beads = 6
    while n_beads != min_beads:

        number = random.randrange(0, n_beads - min_beads)

        name = str(sorted(list(set_allowed_beads_remove))[number])
        set_allowed_beads_remove = set_allowed_beads_remove - set([name,])

        index_pa = np.where(pa_names == name)
        pa_names = np.delete(pa_names, index_pa)

        if name[0] == 'A':
            name = 'C' + name[1:]
        elif name[0] == 'B':
            name = 'D' + name[1:]
        else:
            raise ValueError("Wrong")

        set_allowed_beads_remove = set_allowed_beads_remove - set([name,])

        index_lat = np.where(lat_names == name)
        lat_names = np.delete(lat_names, index_lat)

        M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', lat_names)
        M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', pa_names)

        xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)

        rmss.append(rms(xyz0, xyz))
        n_beads -= 1

    rmss = np.array(rmss)

    print("Minimum number of beads is 6")

    plt.figure()
    plt.plot(range(47, min_beads, -1), rmss[:, 0], label='rms_x')
    plt.plot(range(47, min_beads, -1), rmss[:, 1], label='rms_y')
    plt.plot(range(47, min_beads, -1), rmss[:, 2], label='rms_z')
    plt.plot(range(47, min_beads, -1), rmss[:, 3], label='rms_xyz')
    plt.legend()
    plt.xlabel("Number of beads")
    plt.ylabel("RMS error")
    plt.show()


def q2_2(beads_2d_lat, beads_3d, beads_2d_pa, vert_lat, vert_pa, xyz0):
    # beads_pa = np.array(['A_1_1', 'A_4_2', 'A_1_6', 'A_4_6', 'A_3_3', 'A_2_1', 'A_2_5', 'A_1_4'])
    # beads_lat = np.array(['C_1_1', 'C_4_2', 'C_1_6', 'C_4_6', 'C_3_3', 'C_2_1', 'C_2_5', 'C_1_4'])
    beads_pa = np.array(['B_1_1', 'B_1_5', 'B_5_1', 'B_5_5', 'B_2_2', 'B_2_4', 'B_4_2', 'B_4_4'])
    beads_lat = np.array(['D_1_1', 'D_1_5', 'D_5_1', 'D_5_5', 'D_2_2', 'D_2_4', 'D_4_2', 'D_4_4'])

    M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
    M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)

    xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)
    rmss_SamePlate = rms(xyz0, xyz)
    print(f"RMSE all in the same plate: x_error: {rmss_SamePlate[0]}, y_error: {rmss_SamePlate[1]}, z_error: {rmss_SamePlate[2]}, xyz_error: {rmss_SamePlate[3]}")
#-----------------------------------------------------------------------------------------------------------------------
    # beads_pa = np.array(['A_2_3', 'A_4_2', 'A_2_4', 'A_3_2', 'B_2_3', 'B_4_2', 'B_2_4', 'B_3_2'])
    # beads_lat = np.array(['C_2_3', 'C_4_2', 'C_2_4', 'C_4_2', 'D_2_3', 'D_4_2', 'D_2_4', 'D_3_2'])
    beads_pa = np.array(['A_1_1', 'A_1_3', 'A_3_1', 'A_1_3', 'B_1_1', 'B_1_3', 'B_3_1', 'B_3_3'])
    beads_lat = np.array(['C_1_1', 'C_1_3', 'C_3_1', 'C_3_3', 'D_1_1', 'D_1_3', 'D_3_1', 'D_3_3'])
    M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
    M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)

    xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)
    rmss_NoCorners = rms(xyz0, xyz)
    print(f"RMSE Upper left of A and B: x_error: {rmss_NoCorners[0]}, y_error: {rmss_NoCorners[1]}, z_error: {rmss_NoCorners[2]}, xyz_error: {rmss_NoCorners[3]}")
#-----------------------------------------------------------------------------------------------------------------------
    beads_pa = np.array(['A_1_1', 'A_1_6', 'A_4_1', 'A_4_6', 'B_1_1', 'B_1_5', 'B_5_1', 'B_5_5'])
    beads_lat = np.array(['C_1_1', 'C_1_6', 'C_4_2', 'C_4_6', 'D_1_1', 'D_1_5', 'D_5_1', 'D_5_5'])

    M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
    M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)

    xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)
    rmss_Corners = rms(xyz0, xyz)
    print(f"RMSE corners: x_error: {rmss_Corners[0]}, y_error: {rmss_Corners[1]}, z_error: {rmss_Corners[2]}, xyz_error: {rmss_Corners[3]}")
#-----------------------------------------------------------------------------------------------------------------------
    beads_pa = np.array(['A_2_3', 'A_2_5', 'A_3_3', 'A_3_5', 'B_2_2', 'B_2_4', 'B_4_2', 'B_4_4'])
    beads_lat = np.array(['C_2_3', 'C_2_5', 'C_3_3', 'C_3_5', 'D_2_2', 'D_2_4', 'D_4_2', 'D_4_4'])

    M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
    M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)

    xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)
    rmss_Corners = rms(xyz0, xyz)
    print(
        f"RMSE left of A, center of B: x_error: {rmss_Corners[0]}, y_error: {rmss_Corners[1]}, z_error: {rmss_Corners[2]}, xyz_error: {rmss_Corners[3]}")
#-----------------------------------------------------------------------------------------------------------------------

def q2_3(beads_3d, beads_2d_lat, beads_2d_pa, vert_lat, vert_pa, xyz0):
    beads_pa = ['A_2_3', 'A_2_5', 'A_3_3', 'A_3_5', 'B_2_2', 'B_2_3', 'B_3_2', 'B_3_3']
    beads_lat = ['C_2_3', 'C_2_5', 'C_3_3', 'C_3_5', 'D_2_2', 'D_2_3', 'D_3_2', 'D_3_3']

    gravity = np.array([0, 0, 0], dtype=float)

    for bead in beads_pa:
        _, xyz_bead = get_object(beads_3d, bead)
        gravity += xyz_bead

    for bead in beads_lat:
        _, xyz_bead = get_object(beads_3d, bead)
        gravity += xyz_bead
    gravity /= 16

    M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
    M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)

    xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)

    x = xyz[:, 0]
    x0 = gravity[0]
    y = xyz[:, 1]
    y0 = gravity[1]
    z = xyz[:, 2]
    z0 = gravity[2]
    d_gravity = np.sqrt(((x - x0) ** 2) + ((y - y0) ** 2) + ((z - z0) ** 2))

    x0 = xyz0[:, 0]
    y0 = xyz0[:, 1]
    z0 = xyz0[:, 2]

    d_x = x - x0
    d_y = y - y0
    d_z = z - z0
    d_xyz = np.sqrt(((x - x0) ** 2) + ((y - y0) ** 2) + ((z - z0) ** 2))

    plt.figure()
    plt.plot(d_gravity, d_x, '.', label='d_x')
    plt.plot(d_gravity, d_y, '.', label='d_y')
    plt.plot(d_gravity, d_z, '.', label='d_z')
    plt.plot(d_gravity, d_xyz, '.', label='d_xyz')
    # plt.plot(d_gravity, d_x, label='d_x')
    # plt.plot(d_gravity, d_y, label='d_y')
    # plt.plot(d_gravity, d_z, label='d_z')
    # plt.plot(d_gravity, d_xyz, label='d_xyz')
    plt.legend()
    plt.xlabel("Distance from gravity")
    plt.ylabel("Distance from ideal spinal cord point")
    plt.show()

    # The farther from the center of gravity ish, the more of an error we get. Not necessarily in all individual directions, but in d_xyz yes.


def add_noise(beads, var):
    std = np.sqrt(var)
    beads_noise = copy.deepcopy(beads)
    for i in range(len(beads_noise)):
        # beads_2d_lat_noise[i][1] = np.random.normal(beads_2d_lat_noise[i][1], std) # not used
        beads_noise[i][1] += np.random.normal(0.0, std)
    return beads_noise


def q_3_1(beads_2d_lat, beads_2d_pa, beads_3d, vert_lat, vert_pa, xyz0):

    rmss = []
    vars = np.linspace(0, 100, 100)

    for var in vars:
        beads_2d_lat_noise = add_noise(beads_2d_lat, var)
        beads_2d_pa_noise = add_noise(beads_2d_pa, var)

        M_lat = calibrate(beads_2d_lat_noise, beads_3d, 'lat')
        M_pa = calibrate(beads_2d_pa_noise, beads_3d, 'pa')

        xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)

        rmss.append(rms(xyz0, xyz))

    rmss = np.array(rmss)

    plt.figure()
    plt.plot(vars, rmss[:, 0], label='rms_x')
    plt.plot(vars, rmss[:, 1], label='rms_y')
    plt.plot(vars, rmss[:, 2], label='rms_z')
    plt.plot(vars, rmss[:, 3], label='rms_xyz')
    plt.legend()
    plt.xlabel("Variance")
    plt.ylabel("RMS error")
    plt.show()


def get_specific_beads(beads_2d, bead_names):

    beads_2d_reduced = []
    for i, bead_name in enumerate(bead_names):
        beads_2d_reduced.append([])
        beads_2d_reduced[i].append(bead_name)
        beads_2d_reduced[i].append(get_object(beads_2d, bead_name)[1])

    return beads_2d_reduced


def q_3_2(xyz0, beads_2d_lat, beads_2d_pa, beads_3d, vert_lat, vert_pa):
    # Corners
    beads_pa = np.array(['A_1_1', 'A_1_6', 'A_4_1', 'A_4_6', 'B_1_1', 'B_1_5', 'B_5_1', 'B_5_5'])
    beads_lat = np.array(['C_1_1', 'C_1_6', 'C_4_2', 'C_4_6', 'D_1_1', 'D_1_5', 'D_5_1', 'D_5_5'])

    beads_2d_lat_reduced = get_specific_beads(beads_2d_lat, beads_lat)
    beads_2d_pa_reduced = get_specific_beads(beads_2d_pa, beads_pa)

    rmss = []
    vars = np.linspace(0, 100, 100)

    for var in vars:
        beads_2d_lat_noise_reduced = add_noise(beads_2d_lat_reduced, var)
        beads_2d_pa_noise_reduced = add_noise(beads_2d_pa_reduced, var)

        M_lat = calibrate(beads_2d_lat_noise_reduced, beads_3d, 'lat')
        M_pa = calibrate(beads_2d_pa_noise_reduced, beads_3d, 'pa')

        xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)

        rmss.append(rms(xyz0, xyz))

    rmss = np.array(rmss)

    plt.figure()
    plt.plot(vars, rmss[:, 0], label='rms_x')
    plt.plot(vars, rmss[:, 1], label='rms_y')
    plt.plot(vars, rmss[:, 2], label='rms_z')
    plt.plot(vars, rmss[:, 3], label='rms_xyz')
    plt.legend()
    plt.xlabel("Variance")
    plt.ylabel("RMS error")
    plt.show()


def q_4_1(beads_3d, beads_2d_lat, beads_2d_pa, vert_lat, vert_pa, xyz0):
    rmss = []
    vars = np.linspace(0, 100, 100)

    for var in vars:
        beads_3d_noise = add_noise(beads_3d, var)

        M_lat = calibrate(beads_2d_lat, beads_3d_noise, 'lat')
        M_pa = calibrate(beads_2d_pa, beads_3d_noise, 'pa')

        xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)

        rmss.append(rms(xyz0, xyz))

    rmss = np.array(rmss)

    plt.figure()
    plt.plot(vars, rmss[:, 0], label='rms_x')
    plt.plot(vars, rmss[:, 1], label='rms_y')
    plt.plot(vars, rmss[:, 2], label='rms_z')
    plt.plot(vars, rmss[:, 3], label='rms_xyz')
    plt.legend()
    plt.xlabel("Variance")
    plt.ylabel("RMS error")
    plt.show()


def q_4_2(beads_3d, beads_2d_lat, beads_2d_pa, vert_lat, vert_pa, xyz0):
    rmss = []
    vars = np.linspace(0, 100, 100)

    for var in vars:
        beads_3d_noise = add_noise(beads_3d, var)
        beads_2d_lat_noise = add_noise(beads_2d_lat, var)
        beads_2d_pa_noise = add_noise(beads_2d_pa, var)

        M_lat = calibrate(beads_2d_lat_noise, beads_3d_noise, 'lat')
        M_pa = calibrate(beads_2d_pa_noise, beads_3d_noise, 'pa')

        xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)

        rmss.append(rms(xyz0, xyz))

    rmss = np.array(rmss)

    plt.figure()
    plt.plot(vars, rmss[:, 0], label='rms_x')
    plt.plot(vars, rmss[:, 1], label='rms_y')
    plt.plot(vars, rmss[:, 2], label='rms_z')
    plt.plot(vars, rmss[:, 3], label='rms_xyz')
    plt.legend()
    plt.xlabel("Variance")
    plt.ylabel("RMS error")
    plt.show()

# def plot_hist(beads_2d_lat, beads_3d, beads_2d_pa, vert_lat, vert_pa, xyz0):
#     beads_pa = np.array(['A_1_1', 'A_4_2', 'A_1_6', 'A_4_6', 'A_3_3', 'A_2_1', 'A_2_5', 'A_1_4'])
#     beads_lat = np.array(['C_1_1', 'C_4_2', 'C_1_6', 'C_4_6', 'C_3_3', 'C_2_1', 'C_2_5', 'C_1_4'])
#
#     M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
#     M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)
#
#     xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)
#     rmss = rms(xyz0, xyz)
#     print(f"RMSE all in the same plate: x_error: {rmss[0]}, y_error: {rmss[1]}, z_error: {rmss[2]}, xyz_error: {rmss[3]}")
#
#     beads_pa = np.array(['A_2_3', 'A_4_2', 'A_2_4', 'A_3_2', 'B_2_3', 'B_4_2', 'B_2_4', 'B_3_2'])
#     beads_lat = np.array(['C_2_3', 'C_4_2', 'C_2_4', 'C_4_2', 'D_2_3', 'D_4_2', 'D_2_4', 'D_3_2'])
#
#     M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
#     M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)
#
#     xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)
#     rmss = rms(xyz0, xyz)
#     print(f"RMSE not corners: x_error: {rmss[0]}, y_error: {rmss[1]}, z_error: {rmss[2]}, xyz_error: {rmss[3]}")
#
#     beads_pa = np.array(['A_1_1', 'A_1_6', 'A_4_1', 'A_4_6', 'B_1_1', 'B_1_5', 'B_5_1', 'B_5_5'])
#     beads_lat = np.array(['C_1_1', 'C_1_6', 'C_4_2', 'C_4_6', 'D_1_1', 'D_1_5', 'D_5_1', 'D_5_5'])
#
#     M_lat = calibrate(beads_2d_lat, beads_3d, 'lat', beads_lat)
#     M_pa = calibrate(beads_2d_pa, beads_3d, 'pa', beads_pa)
#
#     xyz = reconstruct(M_lat, M_pa, vert_lat, vert_pa)
#     rmss = rms(xyz0, xyz)
#     print(f"RMSE corners: x_error: {rmss[0]}, y_error: {rmss[1]}, z_error: {rmss[2]}, xyz_error: {rmss[3]}")

def main():

    beads_2d_pa, beads_2d_lat, beads_3d, vert_lat, vert_pa, pa_names, lat_names = load_to_arrays(Calib_Beads2D, Calib_Beads3D, Vertebrae2D)
    all_names = np.concatenate((lat_names, pa_names))

    # Q1.1
    M_lat = calibrate(beads_2d_lat, beads_3d, 'lat')
    M_pa = calibrate(beads_2d_pa, beads_3d, 'pa')

    # Q1.2
    find_params(M_pa)

    # Q1.3
    xyz0 = reconstruct(M_lat, M_pa, vert_lat, vert_pa)

    # Show figure of the reconstructed values
    plot_3d(xyz0)

    # Q2.1
    plot_diff_number_beads(all_names, lat_names, pa_names, beads_2d_lat, beads_2d_pa, beads_3d, vert_lat, vert_pa, xyz0)

    # Q2.2
    q2_2(beads_2d_lat, beads_3d, beads_2d_pa, vert_lat, vert_pa, xyz0)

    # Q2.3
    q2_3(beads_3d, beads_2d_lat, beads_2d_pa, vert_lat, vert_pa, xyz0)

    # Q3.1
    q_3_1(beads_2d_lat, beads_2d_pa, beads_3d, vert_lat, vert_pa, xyz0)

    # Q3.2
    q_3_2(xyz0, beads_2d_lat, beads_2d_pa, beads_3d, vert_lat, vert_pa)

    # Q4.1
    q_4_1(beads_3d, beads_2d_lat, beads_2d_pa, vert_lat, vert_pa, xyz0)

    # Q4.2
    q_4_2(beads_3d, beads_2d_lat, beads_2d_pa, vert_lat, vert_pa, xyz0)


if __name__ == "__main__":
    main()
