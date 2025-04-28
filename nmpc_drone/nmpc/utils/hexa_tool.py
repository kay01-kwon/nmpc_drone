import numpy as np

def hexa_thrust_to_moment(model_description, Parameter, thrust):
    '''
    Convert thrust to moment
    :param model_description: '+' or 'x'
    :param thrust: Four rotor thrusts
    :param arm_length: arm length
    :param C_moment: Coefficient of moment
    :return: m_x, m_y, m_z
    '''
    l = Parameter['l']
    C_T = Parameter['C_T']
    C_M = Parameter['C_M']

    if model_description == '+':
        m_x = l*( thrust[1] - thrust[3])
        m_y = l*( thrust[2] - thrust[0])
    else:
        l = l*np.sqrt(2)/2
        m_x = l*( -thrust[0] + thrust[1] + thrust[2] - thrust[3])
        m_y = l*( -(thrust[0] + thrust[1]) + (thrust[2] + thrust[3]))

    m_z = C_M/C_T*( thrust[0] - thrust[1] + thrust[2] - thrust[3] )

    return m_x, m_y, m_z