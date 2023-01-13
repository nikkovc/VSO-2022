import numpy as np

def phase_dist(phase_a, phase_b):
    """computes a distance that accounts for the modular arithmetic of phase
    and guarantees that the output is between 0 and .5
    
    Args:
        phase_a (float): a phase between 0 and 1
        phase_b (float): a phase between 0 and 1
    
    Returns:
        dist_prime: the difference between the phases, modulo'd between 0 and 0.5
    """
    if isinstance(phase_a, np.ndarray):
        dist_prime = (phase_a-phase_b)
        dist_prime[dist_prime > 0.5] = 1-dist_prime[dist_prime > 0.5]

        dist_prime[dist_prime < -0.5] = -1-dist_prime[dist_prime < -0.5]

    else:
        dist_prime = (phase_a-phase_b)
        if dist_prime > 0.5:
            dist_prime = 1-dist_prime

        elif dist_prime < -0.5:
            dist_prime = -1-dist_prime
    return dist_prime



def select_subj_leg_length(SUBJ_NAME):
    """Extract the subject specific leg length
    
    Args:
        SUBJ_NAME (str): the subject identifier
    
    Returns:
        float: subject leg length in meters
    """
    if SUBJ_NAME == 'AB01':
        leg_length = 927/1000
    elif SUBJ_NAME == 'AB02':  
        leg_length = 934/1000

    elif SUBJ_NAME == 'AB03':  
        leg_length =  823/1000

    elif SUBJ_NAME == 'AB04':
        leg_length = 824/1000
    elif SUBJ_NAME == 'AB05':
        leg_length = 870/1000
    elif SUBJ_NAME == 'AB06':
        leg_length = 856/1000
    elif SUBJ_NAME == 'AB07':
        leg_length = 939/1000
    elif SUBJ_NAME == 'AB08':
        leg_length = 850/1000
    elif SUBJ_NAME == 'AB09':
        leg_length = 960/1000
    elif SUBJ_NAME == 'AB10':
        leg_length = 870/1000

    return leg_length