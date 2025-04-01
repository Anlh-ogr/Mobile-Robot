import numpy as np
import matplotlib.pyplot as plt

# Parameters
ParticleCount = 30                              # kich thuoc quan the
ParamCount = (2 * 10) + (10) + (10 * 3) + (3)   # so luong tham so mo hinh
PositionBounds = [-1, 1]                        # gioi han tim kiem
InertiaWeight = 0.92                            # trong so
CognitiveCoefficient = 0.90                     # he so hoc tap ca nhan
SocialCoefficient = 1.20                        # he so hoc tap xa hoi
MaxIterations = 500                             # so vong lap toi da


# Neural Network Structure
InputLayer = 2              # so luong input
HiddenLayer = 10            # so luong hidden
OutputLayer = 3             # so luong output


# Initialize PSO variables
    # np.inf = infinity
    # np.ones = create an array of ones : tao mang 1 chieu voi tat ca la 1
PersonalBestFitness = np.inf * np.ones(ParticleCount)                       # fitness cua moi ca the
GlobalBestFitness = np.inf                                                  # fitness tot nhat cua quan the

    # np.zeros = create an array of zeros : tao mang 1 chieu voi tat ca la 0
PersonalBestPosition = np.zeros([ParticleCount, ParamCount])                # vi tri tot nhat cua moi ca the
GlobalBestPosition = np.zeros(ParamCount)                                   # vi tri tot nhat cua quan the

FitnessOverIterations = np.zeros(MaxIterations)                             # fitness qua cac vong lap
FitnessMinOverIterations = np.zeros(MaxIterations)                          # fitness nho nhat qua cac vong lap
FitnessMeanOverIterations = np.zeros(MaxIterations)                         # fitness trung binh qua cac vong lap

    # np.random.uniform = generate random numbers : sinh so ngau nhien trong khoang [-1, 1]
    # np.zeros_like = create an array of zeros with the same shape and type as a given array : tao mang 1 chieu voi cung kich thu
ParticlePositions = np.random.uniform(PositionBounds[0], PositionBounds[1], (ParticleCount, ParamCount))    # vi tri cua cac ca the
ParticleVelocities = np.zeros_like(ParticlePositions)                                                       # van toc cua cac ca the


# Neural network forward propagation with bias
def ForwardPropagation(Inputs, WeightsInputHidden, BiasHidden, WeightsHiddenOutput, BiasOutput):
    
    # Activation function
    def ActivationFunction(xVal):
        return np.tanh(xVal)
    
    # Hidden layer computation
        # np.dot = dot product of two arrays : tich vo huong cua hai mang
        # WeightsInputHidden.T = transpose of WeightsInputHidden : chuyen vi cua WeightsInputHidden
    HiddenLayerInput = np.dot(WeightsInputHidden.T, Inputs) + BiasHidden
    HiddenLayerOutput = ActivationFunction(HiddenLayerInput)
    
    # Output layer computation
    OutputLayerInput = np.dot(WeightsHiddenOutput.T, HiddenLayerOutput) + BiasOutput
    return OutputLayerInput


# Cost function
def ComputeCost(Particle):
    # Input data
    InputData = np.array([
        [0.3, 0.35, 0.4, 0.8, 0.9, 1.0, 1.2, 1.6, 2.0],     # Input 1
        [0.3, 0.4, 0.5, 0.75, 0.7, 0.8, 0.4, 0.5, 0.5]      # Input 2
    ])
    
    # Output data
    ExpectedOutput = np.array([
        [1, 1, 1, 0, 0, 0, 0, 0, 0],      # Output 1
        [0, 0, 0, 1, 1, 1, 1, 1, 1],      # Output 2
        [0, 0, 0, 0, 0, 0, 1, 1, 1]       # Output 3
    ])
    
    # Reshape weight and bias
    WeightBiasSplitIndex1 = InputLayer * HiddenLayer                            # chia vi tri cua weight va bias
    WeightBiasSplitIndex2 = WeightBiasSplitIndex1 + HiddenLayer
    WeightBiasSplitIndex3 = WeightBiasSplitIndex2 + HiddenLayer * OutputLayer
    WeightBiasSplitIndex4 = WeightBiasSplitIndex3 + OutputLayer

        # reshape() = reshape an array : chuyen doi kich thuoc cua mang
    WeightsInputHidden = Particle[:WeightBiasSplitIndex1].reshape(InputLayer, HiddenLayer)
    BiasHidden = Particle[WeightBiasSplitIndex1:WeightBiasSplitIndex2].reshape(HiddenLayer, 1)
    WeightsHiddenOutput = Particle[WeightBiasSplitIndex2:WeightBiasSplitIndex3].reshape(HiddenLayer, OutputLayer)
    BiasOutput = Particle[WeightBiasSplitIndex3:WeightBiasSplitIndex4].reshape(OutputLayer, 1)    
    
    # Compute prediction
    Predictions = ForwardPropagation(InputData, WeightsInputHidden, BiasHidden, WeightsHiddenOutput, BiasOutput)
    
    # Compute error
    Error = ExpectedOutput - Predictions
    return np.sum(Error ** 2)
    
# PSO optimization
for Iteration in range(MaxIterations):
        # enumerate = return an enumerate object : tra ve mot doi tuong liet ke
    for ParticleIndex, Particle in enumerate(ParticlePositions):
        # Compute cost
        Cost = ComputeCost(Particle)
        
        # Update personal best
        if Cost < PersonalBestFitness[ParticleIndex]:
            PersonalBestFitness[ParticleIndex] = Cost
            PersonalBestPosition[ParticleIndex] = Particle
            
        # Update global best
        if Cost < GlobalBestFitness:
            GlobalBestFitness = Cost
            GlobalBestPosition = Particle
            
    # Update particle velocities and positions
        # np.random.rand = generate random numbers : sinh so ngau nhien trong khoang [0, 1]
    RandomCognitive = np.random.rand(ParticleCount, ParamCount)
    RandomSocial = np.random.rand(ParticleCount, ParamCount)
        
    # Update particle velocities
    ParticleVelocities = (
        InertiaWeight * ParticleVelocities
        + CognitiveCoefficient * RandomCognitive * (PersonalBestPosition - ParticlePositions)
        + SocialCoefficient * RandomSocial * (GlobalBestPosition - ParticlePositions)
    )
    
    # Update particle positions
        # np.clip = clip (limit) the values in an array : gioi han cac gia tri trong mang
    ParticlePositions += ParticleVelocities
    ParticlePositions = np.clip(ParticlePositions, PositionBounds[0], PositionBounds[1])
    
    # Record fitness values
    FitnessOverIterations[Iteration] = GlobalBestFitness
    FitnessMinOverIterations[Iteration] = np.min(PersonalBestFitness)
    FitnessMeanOverIterations[Iteration] = np.mean(PersonalBestFitness)
    
    print(f"Iteration: {Iteration}, Loss: {GlobalBestFitness:.6f}, Location: {GlobalBestPosition}")
    

print(f"Best loss: {GlobalBestFitness:.6f}, Best location: {GlobalBestPosition}")


# Plot results
plt.figure(figsize=(12, 6))
plt.plot(FitnessOverIterations, label='Global Best Fitness')
plt.plot(FitnessMinOverIterations, label='Min Fitness Per Generation')
plt.plot(FitnessMeanOverIterations, label='Mean Fitness Per Generation')
plt.xlabel('Iteration')
plt.ylabel('Fitness')
plt.legend()
plt.title('PSO for Optimization Process')
plt.grid()
plt.show()