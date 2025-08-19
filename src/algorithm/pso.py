import numpy as np
import matplotlib.pyplot as plt
from typing import Callable, Tuple, List, Optional

N_PARTICLES = 25
MAX_ITERATIONS = 50

class PSO:
    """
    Particle Swarm Optimization for 2D parameter tuning [Kp, Ki]
    """
    
    def __init__(self, 
                 objective_function: Callable[[np.ndarray], float],
                 bounds: Tuple[Tuple[float, float], Tuple[float, float]],
                 n_particles: int = N_PARTICLES,
                 max_iterations: int = MAX_ITERATIONS,
                 w: float = 0.9,
                 c1: float = 2.0,
                 c2: float = 2.0,
                 random_seed: Optional[int] = None):
        """
        Initialize PSO optimizer
        
        Parameters:
        -----------
        objective_function : Callable
            Function to minimize f(x) where x is [Kp, Ki]
        bounds : Tuple of tuples
            ((Kp_min, Kp_max), (Ki_min, Ki_max))
        n_particles : int               Number of particles in swarm
        max_iterations : int            Maximum number of iterations
        w : float                       Inertia weight
        c1 : float                      Cognitive parameter (personal best)
        c2 : float                      Social parameter (global best)
        random_seed : int, optional     Random seed for reproducibility
        """
        self.objective_function = objective_function
        self.bounds = bounds
        self.n_particles = n_particles
        self.max_iterations = max_iterations
        self.w = w
        self.c1 = c1
        self.c2 = c2
        
        if random_seed is not None:
            np.random.seed(random_seed)
        
        # Initialize particles
        self._initialize_particles()
        
        # History tracking
        self.best_fitness_history = []
        self.best_position_history = []
        
    def _initialize_particles(self):
        """Initialize particle positions and velocities"""
        # Extract bounds
        kp_min, kp_max = self.bounds[0]
        ki_min, ki_max = self.bounds[1]
        
        # Initialize positions randomly within bounds
        self.positions = np.random.uniform(
            low=[kp_min, ki_min],
            high=[kp_max, ki_max],
            size=(self.n_particles, 2)
        )
        
        # Initialize velocities randomly (small values)
        vel_range = 0.1 * np.array([[kp_max - kp_min, ki_max - ki_min]])
        self.velocities = np.random.uniform(
            low=-vel_range,
            high=vel_range,
            size=(self.n_particles, 2)
        )
        
        # Calculate initial fitness
        self.fitness = np.array([self.objective_function(pos) for pos in self.positions])
        
        # Initialize personal best
        self.personal_best_positions = self.positions.copy()
        self.personal_best_fitness = self.fitness.copy()
        
        # Initialize global best
        best_idx = np.argmin(self.fitness)
        self.global_best_position = self.positions[best_idx].copy()
        self.global_best_fitness = self.fitness[best_idx]
        
    def _update_velocity(self, particle_idx: int):
        """Update velocity of a particle"""
        r1, r2 = np.random.random(2)
        
        # Current position and velocity
        pos = self.positions[particle_idx]
        vel = self.velocities[particle_idx]
        
        # Personal best and global best
        p_best = self.personal_best_positions[particle_idx]
        g_best = self.global_best_position
        
        # Update velocity
        inertia = self.w * vel
        cognitive = self.c1 * r1 * (p_best - pos)
        social = self.c2 * r2 * (g_best - pos)
        
        new_velocity = inertia + cognitive + social
        
        # Velocity clamping (optional - prevents excessive velocities)
        kp_range = self.bounds[0][1] - self.bounds[0][0]
        ki_range = self.bounds[1][1] - self.bounds[1][0]
        max_vel = np.array([0.2 * kp_range, 0.2 * ki_range])
        
        new_velocity = np.clip(new_velocity, -max_vel, max_vel)
        
        return new_velocity
    
    def _update_position(self, particle_idx: int):
        """Update position of a particle"""
        new_pos = self.positions[particle_idx] + self.velocities[particle_idx]
        
        # Boundary handling - reflect particles that go out of bounds
        kp_min, kp_max = self.bounds[0]
        ki_min, ki_max = self.bounds[1]
        
        # Clamp to boundaries
        new_pos[0] = np.clip(new_pos[0], kp_min, kp_max)
        new_pos[1] = np.clip(new_pos[1], ki_min, ki_max)
        
        return new_pos
    
    def optimize(self, verbose: bool = True) -> Tuple[np.ndarray, float]:
        """
        Run PSO optimization
        
        Parameters:
        -----------
        verbose : bool
            Print progress information
            
        Returns:
        --------
        best_position : np.ndarray
            Best [Kp, Ki] parameters found
        best_fitness : float
            Best fitness value achieved
        """
        
        for iteration in range(self.max_iterations):
            # Update all particles
            for i in range(self.n_particles):
                # Update velocity and position
                self.velocities[i] = self._update_velocity(i)
                self.positions[i] = self._update_position(i)
                
                # Evaluate new fitness
                new_fitness = self.objective_function(self.positions[i])
                self.fitness[i] = new_fitness
                
                # Update personal best
                if new_fitness < self.personal_best_fitness[i]:
                    self.personal_best_fitness[i] = new_fitness
                    self.personal_best_positions[i] = self.positions[i].copy()
                
                # Update global best
                if new_fitness < self.global_best_fitness:
                    self.global_best_fitness = new_fitness
                    self.global_best_position = self.positions[i].copy()
            
            # Store history
            self.best_fitness_history.append(self.global_best_fitness)
            self.best_position_history.append(self.global_best_position.copy())
            
            # Print progress
            if verbose and (iteration + 1) % 10 == 0:
                print(f"Iteration {iteration + 1:3d}: "
                      f"Best fitness = {self.global_best_fitness:.6f}, "
                      f"Kp = {self.global_best_position[0]:.4f}, "
                      f"Ki = {self.global_best_position[1]:.4f}")
        
        return self.global_best_position, self.global_best_fitness
    
    def plot_convergence(self):
        """Plot convergence history"""
        plt.figure(figsize=(12, 4))
        
        # Plot fitness convergence
        plt.subplot(1, 2, 1)
        plt.plot(self.best_fitness_history)
        plt.xlabel('Iteration')
        plt.ylabel('Best Fitness')
        plt.title('PSO Convergence')
        plt.grid(True)
        
        # Plot parameter evolution
        plt.subplot(1, 2, 2)
        kp_history = [pos[0] for pos in self.best_position_history]
        ki_history = [pos[1] for pos in self.best_position_history]
        
        plt.plot(kp_history, label='Kp')
        plt.plot(ki_history, label='Ki')
        plt.xlabel('Iteration')
        plt.ylabel('Parameter Value')
        plt.title('Parameter Evolution')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def plot_particles(self, iteration_snapshots: List[int] = None):
        """Plot particle positions at different iterations"""
        if iteration_snapshots is None:
            iteration_snapshots = [0, self.max_iterations // 4, 
                                 self.max_iterations // 2, self.max_iterations - 1]
        
        # Re-run optimization to capture snapshots (simplified version)
        # In practice, you'd modify the optimize method to save snapshots
        plt.figure(figsize=(10, 8))
        
        # Create a simple visualization of the search space
        kp_min, kp_max = self.bounds[0]
        ki_min, ki_max = self.bounds[1]
        
        # Plot final particle positions
        plt.scatter(self.positions[:, 0], self.positions[:, 1], 
                   alpha=0.6, label='Final Particles')
        plt.scatter(self.global_best_position[0], self.global_best_position[1], 
                   color='red', s=100, marker='*', label='Global Best')
        
        plt.xlabel('Kp')
        plt.ylabel('Ki')
        plt.title('PSO Particle Positions')
        plt.xlim(kp_min, kp_max)
        plt.ylim(ki_min, ki_max)
        plt.legend()
        plt.grid(True)
        plt.show()


# Example usage and test functions
def example_objective_function(params: np.ndarray) -> float:
    """
    Example objective function for PI controller tuning
    This is a simplified example - replace with your actual system response
    
    Parameters:
    -----------
    params : np.ndarray
        [Kp, Ki] parameters
        
    Returns:
    --------
    float
        Fitness value (lower is better)
    """
    kp, ki = params[0], params[1]
    
    # Example: Minimize ISE (Integral Square Error) for step response
    # This is a simplified analytical approximation
    # In practice, you'd simulate your control system
    
    # Penalty for negative parameters
    if kp < 0 or ki < 0:
        return 1e6
    
    # Simple quadratic function with optimum around [1.5, 0.8]
    target_kp, target_ki = 1.5, 0.8
    error = (kp - target_kp)**2 + (ki - target_ki)**2
    
    # Add some complexity to make optimization more interesting
    cross_term = 0.1 * np.sin(kp * ki)
    noise = 0.01 * np.random.random()  # Small noise
    
    return error + cross_term + noise


def control_system_objective(params: np.ndarray) -> float:
    """
    More realistic objective function for control system tuning
    Based on typical PI controller performance metrics
    """
    kp, ki = params[0], params[1]
    
    # Bounds checking
    if kp <= 0 or ki <= 0:
        return 1e6
    
    # Simulate closed-loop response characteristics
    # Natural frequency and damping ratio for PI controller
    wn = np.sqrt(ki)  # Natural frequency
    zeta = kp / (2 * wn)  # Damping ratio
    
    # Performance penalties
    overshoot_penalty = max(0, 20 - 100 * np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2)))**2 if zeta < 1 else 0
    settling_time_penalty = (4 / (zeta * wn))**2 if zeta > 0 else 1e6
    rise_time_penalty = (1.8 / wn)**2
    
    # Stability margin penalty
    if zeta < 0.1:  # Too oscillatory
        stability_penalty = (0.1 - zeta)**2 * 1000
    elif zeta > 2:  # Too slow
        stability_penalty = (zeta - 2)**2 * 100
    else:
        stability_penalty = 0
    
    total_cost = overshoot_penalty + settling_time_penalty + rise_time_penalty + stability_penalty
    
    return total_cost

# Additional utility function for custom objective functions
def create_step_response_objective(plant_tf_num: List[float], 
                                 plant_tf_den: List[float],
                                 desired_specs: dict) -> Callable:
    """
    Create an objective function based on step response specifications
    
    Parameters:
    -----------
    plant_tf_num : List[float]
        Numerator coefficients of plant transfer function
    plant_tf_den : List[float] 
        Denominator coefficients of plant transfer function
    desired_specs : dict
        Desired performance specifications
        e.g., {'overshoot_max': 10, 'settling_time_max': 2.0, 'rise_time_max': 1.0}
    
    Returns:
    --------
    Callable
        Objective function for PSO
    """
    def objective(params):
        kp, ki = params[0], params[1]
        
        # This is a simplified example
        # In practice, you would use control system libraries like python-control
        # to simulate the actual step response
        
        # Simple second-order approximation
        wn = np.sqrt(ki)
        zeta = kp / (2 * wn) if wn > 0 else 0
        
        if zeta <= 0 or wn <= 0:
            return 1e6
        
        # Calculate performance metrics
        if zeta < 1:
            overshoot = 100 * np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2))
        else:
            overshoot = 0
            
        settling_time = 4 / (zeta * wn)
        rise_time = 1.8 / wn
        
        # Penalty function
        cost = 0
        
        if 'overshoot_max' in desired_specs:
            if overshoot > desired_specs['overshoot_max']:
                cost += (overshoot - desired_specs['overshoot_max'])**2
        
        if 'settling_time_max' in desired_specs:
            if settling_time > desired_specs['settling_time_max']:
                cost += (settling_time - desired_specs['settling_time_max'])**2 * 10
        
        if 'rise_time_max' in desired_specs:
            if rise_time > desired_specs['rise_time_max']:
                cost += (rise_time - desired_specs['rise_time_max'])**2 * 5
        
        return cost + 0.01 * (kp**2 + ki**2)  # Small regularization
    
    return objective


# Example of using custom objective function
def example_custom_tuning():
    """Example of using PSO with custom control system specifications"""
    print("=== Example 3: Custom Control System Tuning ===")
    
    # Define desired performance specifications
    specs = {
        'overshoot_max': 15.0,      # Maximum 15% overshoot
        'settling_time_max': 3.0,   # Settle within 3 seconds
        'rise_time_max': 1.5        # Rise time under 1.5 seconds
    }
    
    # Create custom objective function
    custom_objective = create_step_response_objective(
        plant_tf_num=[1.0],
        plant_tf_den=[1.0, 0.5, 0.0],  # Example plant: 1/(s^2 + 0.5s)
        desired_specs=specs
    )
    
    # Define bounds
    bounds = ((0.1, 8.0), (0.1, 4.0))
    
    # Run PSO
    pso_custom = PSO(
        objective_function=custom_objective,
        bounds=bounds,
        n_particles=35,
        max_iterations=60,
        random_seed=456
    )
    
    best_params, best_fitness = pso_custom.optimize()
    
    print(f"\nCustom Tuning Results:")
    print(f"Optimal Kp: {best_params[0]:.4f}")
    print(f"Optimal Ki: {best_params[1]:.4f}")
    print(f"Cost Function Value: {best_fitness:.6f}")
    
    # Verify performance
    kp, ki = best_params
    wn = np.sqrt(ki)
    zeta = kp / (2 * wn)
    
    if zeta < 1:
        overshoot = 100 * np.exp(-zeta * np.pi / np.sqrt(1 - zeta**2))
        print(f"Achieved Overshoot: {overshoot:.2f}%")
    
    settling_time = 4 / (zeta * wn)
    rise_time = 1.8 / wn
    
    print(f"Achieved Settling Time: {settling_time:.2f} s")
    print(f"Achieved Rise Time: {rise_time:.2f} s")

if __name__ == "__main__":
    # Example 1: Simple optimization
    print("=== Example 1: Simple Optimization ===")
    
    # Define bounds for [Kp, Ki]
    bounds = ((0.1, 5.0), (0.1, 2.0))  # (Kp_min, Kp_max), (Ki_min, Ki_max)
    
    # Create PSO instance
    pso = PSO(
        objective_function=example_objective_function,
        bounds=bounds,
        n_particles=25,
        max_iterations=50,
        w=0.9,
        c1=2.0,
        c2=2.0,
        random_seed=42
    )
    
    # Run optimization
    best_params, best_fitness = pso.optimize(verbose=True)
    
    print(f"\nOptimization Results:")
    print(f"Best Kp: {best_params[0]:.4f}")
    print(f"Best Ki: {best_params[1]:.4f}")
    print(f"Best Fitness: {best_fitness:.6f}")
    
    # Plot results
    pso.plot_convergence()
    pso.plot_particles()
    
    print("\n" + "="*50)