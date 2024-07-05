# IMU sensor fusion
    ## Input data format and meeaning:
        `a` acceleration(acc)
        `omega` angular velocity(gyro)
        `dt` time step
        `g` gravity vector

        CSV data folder or the .mat file includes the same data.

    ## Outputs
        You should output the estimated euler angle `euler_est` and compare it with the ground truth `euler_gt` to evaluate the performance of your algorithm.

    ## Requirements
        - Please refer to(and play with) AHRSFilter usage and algorithms overview. https://www.mathworks.com/help/nav/ref/ahrsfilter-system-object.html
        - Convert it to Python or C code.
        - Please compare the results with the `euler_gt` data provided.
