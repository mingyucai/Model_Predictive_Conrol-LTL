## Description

This package contains implementations for online-plan synthesis algorithm give a dynamic
environment (as the finite transition system) and a potential infeasible Linear temporal
logic formula (as the robot’s task). It outputs the predicted trajectories at each
time-step that optimize the reward-related objectives and finally fulfill the task maximumly.

The project applies a standard model predictive control based on the P_MAS_TG implementation provided at [P_MAS_TG](https://mengguo.github.io/personal_site/P_MAS_TG.html) and [RHTLC](http://sites.bu.edu/hyness/rhtlc/).
The reference of P_MAS_TG and RHTLC implementation:
* Guo, Meng, and Dimos V. Dimarogonas, "Multi-agent plan reconfiguration under local LTL specifications", The International Journal of Robotics Research 34.2 (2015). [[PDF]](https://journals.sagepub.com/doi/full/10.1177/0278364914546174?casa_token=OSKkS6Vqz-gAAAAA%3A5E1-CDKymr6gHfepShhnvEP-Jnmw8afGzfxwV_wL4s69r_0uxAoh-hJNpGgwt0jj5Fa00NX2vJvd3A)
* Ding, Xuchu, Mircea Lazar, and Calin Belta, "LTL receding horizon control for finite deterministic systems", Automatica 50.2 (2014). [[PDF]](https://www.sciencedirect.com/science/article/pii/S0005109813005475?casa_token=bF8jKGWx-oMAAAAA:v_QZJMRZVz6y04OQtbP0tA6LujTguHcyYrVa9ISHTR55BmNcQ91MspkD2QIvq7yM-Xj3kpilaSw)

[Project Webpage](https://github.com/mingyucai/Model_Predictive_Conrol-LTL)

## Results
1. In the simlulations, the graph-like environments are tested. 

   Demo: [Simulation Vidoe](https://www.youtube.com/watch?v=RyRnKXDDH5U)
![grid.png](grid.png)

2. In the experiment, the neighbor numbers of simulation snapshot provide the time-varying reward 
(Here we use the random values) which is our optimization objective.

   Demo: [Experiment video](https://www.youtube.com/watch?v=16j6TmVUrTk)
![experiment.jpg](experiment.jpg)

## Publication
* Cai, Mingyu, Hao Peng, Zhijun Li, Hongbo Gao, and Zhen Kan., "Receding Horizon Control-Based Motion Planning With Partially Infeasible LTL Constraints.", IEEE Control Systems Letters 5, no. 4 (2020): 1279-1284. [[PDF]](https://ieeexplore.ieee.org/abstract/document/9234439?casa_token=cyg4D_wIfCsAAAAA:hL5irFNksL4g4YL4RWAUs1vVp5IdByJ91qUJzEAbFVrLVRwxRfUvwjS58sBLW3Clvvuwq-cA9Q)
* Cai, Mingyu, Hao Peng, Zhijun Li, Hongbo Gao, and Zhen Kan., "Receding Horizon Control-Based Motion Planning With Partially Infeasible LTL Constraints.", arXiv:2007.12123 (2020). [[PDF]](https://arxiv.org/abs/2007.12123)

## Features
- Allow both normal and infeasible LTL based product automaton task formulas
- Motion model can be dynamic and initially unknown
- Soft specification is maximumly satisfied.
- Online-Path planning is designed from the model predicted control methodology.
- Collect and transfer the real-time data via Optitrack camera systems 
- Allow automatically calibrate the mobile robots to obtain its orientation and dynamics.

## Debugging
### Ptthon3
* Install python packages like networkx2.0.ply
* Add to your PYTHONPATH, to import it in your own project.
* ltlba_32 and ltlba_64 are executable files complied under Linux, please follow [ltl2ba/README.txt]
```
Try [path_plan.py](https://github.com/mingyucai/Model-Predictive-Control/blob/master/path_plan.py) 
```
### Matlab
* Add folder to your PYTHONPATH.
```
Execute [Receding_Horizon_Control.py](https://github.com/mingyucai/Model-Predictive-Control/blob/master/Matlab_simulation/LTL_MPC/RHC_ACC.m)
```


## References
Please use this bibtex entry if you want to cite this repository in your publication:
```
@article{cai2020receding,
  title={Receding Horizon Control-Based Motion Planning With Partially Infeasible LTL Constraints},
  author={Cai, Mingyu and Peng, Hao and Li, Zhijun and Gao, Hongbo and Kan, Zhen},
  journal={IEEE Control Systems Letters},
  volume={5},
  number={4},
  pages={1279--1284},
  year={2020},
  publisher={IEEE}
}

```
