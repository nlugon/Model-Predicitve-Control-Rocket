# Model Predictive Control for Rocket
The goal of this project is to design and implement an MPC controller that controls a simulated rocket protype. Inputs considered are rocket position, orientation, angular velocity, and linear velocity. Outputs considered are throttle and thrust deflection angles.

A linear MPC controller was first implemented, for which rocket dynamics were linearized. Next was implemented a non linear MPC controller, and performance of both controllers was compared.

The final goal of the project was to follow a path formed by the letters "EPFL" in the air. 

## Authors
- [Bassam El Rawas](https://github.com/BassamR)
- [Vincent Gherold](https://github.com/VinceGHER)
- [Noah Lugon](https://github.com/nlugon)

## Performance 

<img width="620" alt="MPC Performance" src="https://user-images.githubusercontent.com/29159082/219495978-bf6a9ec0-db05-44c4-ac64-a5780d01ffaa.png">


 Results are detailed in "MPC_Miniproject_Report.pdf".


##
This project was realised in the scope of the "Model predictive control" class (ME-425) thaught by Jones Colin Neil.
 
