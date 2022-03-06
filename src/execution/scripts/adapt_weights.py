def adapt_weights(forces):
    c = 1
    fsum = forces[0] + forces[1] + forces[2]
    scale = 1 / (1 + c*fsum)

    return scale
    
