import numpy as np
import matplotlib.pyplot as plt

# Parameters
NumberParticles = 100           # kich thuoc quan the
NumberDimensions = 2            # so luong ca the
SearchSpaceBounds = [-1, 1]     # gioi han tim kiem
InertiaWeight = 0.3             # trong so
CognitiveCoefficient = 0.5      # he so hoc tap ca nhan
SocialCoefficient = 0.5         # he so hoc tap xa hoi
MaxIterations = 1000            # so vong lap toi da

# Initialize variables
    # np.inf = infinity
    # np.full = array of infinity
PersonalBestFitness = np.full(NumberParticles, np.inf)      # P_best
GlobalBestFitness = np.inf                                  # G_best

    # np.zeros = array of zeros
PersonalBestPositions = np.zeros([NumberParticles, NumberDimensions])   # P_location
GlobalBestPosition = np.zeros([NumberDimensions, 1])                    # G_location

FitnessPerIteration = np.zeros(MaxIterations)           # fitness_Value
FitnessMinPerIteration = np.zeros(MaxIterations)        # fitness_Generation
FitnessMeanPerIteration = np.zeros(MaxIterations)       # fitness_Mean

    # np.random.uniform = random number in range
    # np.zeros_like = array of zeros
ParticlePositions = np.random.uniform(SearchSpaceBounds[0], SearchSpaceBounds[1], (NumberParticles, NumberDimensions))  # P_matrix
ParticleVelocities = np.zeros_like(ParticlePositions)                                                                   # Velocity


# Loss function
def ObjectiveFunction(xVal, yVal):
    # calculate the objective function value
    Fitness = 4 * xVal**2 - 2.1 * xVal**4 + (1/3) * xVal**6 + xVal * yVal - 4 * yVal**2 + 4 * yVal**4
    
    # check if the constraint is satisfied
    if (-np.sin(4 * np.pi * xVal) + 2 * np.sin(2 * np.pi * yVal)**2 > 1.5):
        Fitness += 1000     # penalty term
        
    return Fitness


# Update particles and velocities function
def UpdateParticlePositionsAndVelocities(ParticlePositions, ParticleVelocities, PersonalBestPositions, GlobalBestPosition, InertiaWeight, CognitiveCoefficient, SocialCoefficient):
    # np.random.rand = random number in range
    RandomCognitive = np.random.rand(NumberParticles, NumberDimensions)     # cognitive component
    RandomSocial = np.random.rand(NumberParticles, NumberDimensions)        # social component
    
    # update the velocities
    ParticleVelocities = (
        InertiaWeight * ParticleVelocities
        + CognitiveCoefficient * RandomCognitive * (PersonalBestPositions - ParticlePositions)
        + SocialCoefficient * RandomSocial * (GlobalBestPosition - ParticlePositions)
    )
    ParticlePositions += ParticleVelocities
    return ParticlePositions, ParticleVelocities


# Particle Swarm Optimization algorithm
def ParticleSwarmOptimization():
    global GlobalBestFitness, GlobalBestPosition, PersonalBestFitness, PersonalBestPositions
    
    # iterate over the maximum number of iterations
    for iteration in range(MaxIterations):
        # iterate over each particle
        for ParticleIndex, ParticlePositions in enumerate(ParticlePositions):
            # calculate the fitness value
            xVal, yVal = ParticlePositions
            Fitness = ObjectiveFunction(xVal, yVal)
            
            # update the personal best
            if Fitness < PersonalBestFitness[ParticleIndex]:
                PersonalBestFitness[ParticleIndex] = Fitness
                PersonalBestPositions[ParticleIndex] = ParticlePositions
                
            # update the global best
            if Fitness < GlobalBestFitness:
                GlobalBestFitness = Fitness
                GlobalBestPosition = ParticlePositions
                
            
        # Log fitness statistics
        FitnessPerIteration[iteration] = GlobalBestFitness
        FitnessMinPerIteration[iteration] = np.min(PersonalBestFitness)
        FitnessMeanPerIteration[iteration] = np.mean(PersonalBestFitness)
        
        # update the particles positions and velocities
        ParticlePositions, ParticleVelocities = UpdateParticlePositionsAndVelocities(
            ParticlePositions, ParticleVelocities, PersonalBestPositions, GlobalBestPosition, InertiaWeight, CognitiveCoefficient, SocialCoefficient
        )
        
        print(f"Iteration: {iteration}, Best Fitness: {GlobalBestFitness:.4f}, Best Position: {GlobalBestPosition}")
        
    print(f"Optimal Fitness: {GlobalBestFitness:.4f}, Optimal Position: {GlobalBestPosition}")
    PlotOptimizationResults()
    

# Plot the optimization results
def PlotOptimizationResults():
    plt.figure()
    plt.plot(FitnessPerIteration, label="Global Best Fitness")
    plt.title("Fitness Progression")
    plt.xlabel("Iteration")
    plt.ylabel("Fitness")
    plt.legend()
    
    plt.figure()
    plt.plot(FitnessMinPerIteration, label="Minimum Fitness")
    plt.title("Minimum Fitness Per Iteration")
    plt.xlabel("Iteration")
    plt.ylabel("Fitness")
    plt.legend()
    
    plt.figure()
    plt.plot(FitnessMeanPerIteration, label="Mean Fitness")
    plt.title("Mean Fitness Per Iteration")
    plt.xlabel("Iteration")
    plt.ylabel("Fitness")
    plt.legend()
    
    plt.show()
    

# Run the Particle Swarm Optimization algorithm
if __name__ == "__main__":
    ParticleSwarmOptimization()