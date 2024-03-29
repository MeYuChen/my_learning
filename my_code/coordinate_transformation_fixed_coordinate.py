import numpy 
###
# 坐标系变换，坐标系不动，点绕着坐标系变换
#
#

def x_roatiton_martix(x_radius):
    # x = [
    #     1 0 0
    #     0 cos(yaw) -sin(yaw)
    #     0 sin(yaw) cos(yaw)
    # ]
     return  numpy.array(
            [
                [1, 0, 0],
                [0, numpy.cos(x_radius), -numpy.sin(x_radius)],
                [0, numpy.sin(x_radius), numpy.cos(x_radius)],
            ]
        )


def y_roatiton_martix(y_radius):
    # x = [
    #     
    #     cos(yaw) 0 sin(yaw)
    #      0       1    0
    #     -sin(yaw) 0 cos(yaw)
    # ]
    return numpy.array(
            [
                [numpy.cos(y_radius), 0, numpy.sin(y_radius)],
                [0, 1, 0],
                [-numpy.sin(y_radius), 0, numpy.cos(y_radius)],
            ]
        )

def z_roatiton_martix(z_radius):
    # x = [
    #     cos(yaw) -sin(yaw) 0
    #     sin(yaw)  cos(yaw) 0
    #         0        0     1
    # ]
    
   return numpy.array(
            [
                [numpy.cos(z_radius), -numpy.sin(z_radius), 0],
                [numpy.sin(z_radius), numpy.cos(z_radius), 0],
                [0, 0, 1],
            ]
        )