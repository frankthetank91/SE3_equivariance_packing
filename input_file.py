def get_packing_input():
    """
    Define the input parameters for the packing problem.

    :return: Dictionary of input parameters.
    """
    return {
        "N": 3,               # Number of objects
        "L": 5,               # Container length
        "W": 5,               # Container width
        "H": 6,               # Container height
        "l": [2, 3, 4],       # Object lengths
        "w": [3, 2, 5],       # Object widths
        "h": [4, 5, 3],       # Object heights
        "wx": 1,              # Weight in x direction
        "wy": 1,              # Weight in y direction
        "wz": 1,              # Weight in z direction
    }
