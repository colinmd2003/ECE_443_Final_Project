# ECE 443 Final Project - Reynolds Flocking Algorithms (Boids) in MATLAB

This project implements Reynolds' flocking algorithms (boids) using alpha, beta, and gamma agents in both 2D and 3D environments. Developed as part of **ECE 443: Controls III**, the simulations visualize agent interactions such as flocking, obstacle avoidance, and leader-follower dynamics.

## Features

- **Agent Types**:
  - **Alpha Agents**: Follow traditional flocking rules of cohesion, separation, and alignment.
  - **Beta Agents**: Focus on obstacle avoidance.
  - **Gamma Agents**: Leader agents guiding the flock.

- **2D and 3D Simulations**:
  - Flocking in free space and maneuvers like split/rejoin and squeeze.

## Running the Simulations

1. Clone the repository:
   ```bash
   git clone https://github.com/colinmd2003/ECE_443_Final_Project.git
   ```
2. Open MATLAB and navigate to the project directory.
    Run the relevant simulation script for 2D or 3D flocking.

## Future Work

    - Refine the algorithm to improve flocking behavior and obstacle avoidance.
    - Add more complex obstacle configurations to test agent behaviors.
    - Implement interactive control for the gamma agent for dynamic leader-follower simulations.
    - Reformat the existing code, as it's currently spaghetti code, to make it cleaner, more modular, and easier to maintain.
    - Separate each simulation to improve accessibility

## License

This project is licensed under the MIT License. See the LICENSE file for more details.
