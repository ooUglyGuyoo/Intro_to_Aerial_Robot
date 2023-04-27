Use the script "test_trajectory.m" as the main entry point.

readonly folder: supposed to be "read only"
    quadModel_readonly.m: parameters of a 300g quadrotor
    quadEOM_readonly.m: dynamics model of quadrotor.
    run_trajectory_readonly: solve the equation of motion, receive desired trajectory, run your controller, iteratively. visualization is also included.

utils: useful functions.

test_trajectory.m: main entry.

-----------------------------------------------------------------------------------------------------------
controller.m: You have already had a good controller if you finished phase 1. Use it directly, or improve its performance if necessary.

trajectory_generator.m: What you need to work with in this assignment. Design the trajectory for quadrotor given the path. And calculate desired state given current time.

Contact TAs with any questions you may have.

hwangeh@connect.ust.hk
pliuan@connect.ust.hk
