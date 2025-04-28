import numpy as np

def hexa_thrust_to_moment(Parameter, thrust):
    '''
    Convert thrust to moment
    :param thrust: Six rotor thrusts
    :param arm_length: arm length
    :param C_moment: Coefficient of moment
    :return: m_x, m_y, m_z
    '''
    l = Parameter['l']
    C_T = Parameter['C_T']
    C_M = Parameter['C_M']
    m_x = l*(
        (thrust[0] + thrust[2])*np.cos(np.pi/3.0) + thrust[1]
        -(thrust[3] + thrust[5])*np.cos(np.pi/3.0) - thrust[4]
    )

    m_y = l*(
        -(thrust[0] + thrust[5])*np.sin(np.pi/3.0)
        +(thrust[2] + thrust[3])*np.sin(np.pi/3.0)
    )

    m_z = C_M/C_T*( -thrust[0] + thrust[1] - thrust[2]
                    + thrust[3] - thrust[4] + thrust[5])

    return m_x, m_y, m_z