# multiAgents.py
# --------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from util import manhattanDistance
from game import Directions
import random, util

from game import Agent

class ReflexAgent(Agent):
    """
    A reflex agent chooses an action at each choice point by examining
    its alternatives via a state evaluation function.

    The code below is provided as a guide.  You are welcome to change
    it in any way you see fit, so long as you don't touch our method
    headers.
    """
    
    def getAction(self, gameState):
        """
        You do not need to change this method, but you're welcome to.
        getAction chooses among the best options according to the evaluation function.
        Just like in the previous project, getAction takes a GameState and returns
        some Directions.X for some X in the set {North, South, West, East, Stop}
        """
        # Collect legal moves and successor states
        legalMoves = gameState.getLegalActions()

        # Choose one of the best actions
        scores = [self.evaluationFunction(gameState, action) for action in legalMoves]
        bestScore = max(scores)
        bestIndices = [index for index in range(len(scores)) if scores[index] == bestScore]
        chosenIndex = random.choice(bestIndices) # Pick randomly among the best

        "Add more of your code here if you want to"

        return legalMoves[chosenIndex]

    def evaluationFunction(self, currentGameState, action):
        """
        Design a better evaluation function here.

        The evaluation function takes in the current and proposed successor
        GameStates (pacman.py) and returns a number, where higher numbers are better.

        The code below extracts some useful information from the state, like the
        remaining food (newFood) and Pacman position after moving (newPos).
        newScaredTimes holds the number of moves that each ghost will remain
        scared because of Pacman having eaten a power pellet.

        Print out these variables to see what you're getting, then combine them
        to create a masterful evaluation function.
        """
        # Useful information you can extract from a GameState (pacman.py)
        successorGameState = currentGameState.generatePacmanSuccessor(action)
        newPos = successorGameState.getPacmanPosition()
        newFood = successorGameState.getFood()
        newGhostStates = successorGameState.getGhostStates()
        newScaredTimes = [ghostState.scaredTimer for ghostState in newGhostStates]

        "*** YOUR CODE HERE ***"
        return successorGameState.getScore()

def scoreEvaluationFunction(currentGameState):
    """
    This default evaluation function just returns the score of the state.
    The score is the same one displayed in the Pacman GUI.

    This evaluation function is meant for use with adversarial search agents
    (not reflex agents).
    """
    return currentGameState.getScore()

class MultiAgentSearchAgent(Agent):
    """
    This class provides some common elements to all of your
    multi-agent searchers.  Any methods defined here will be available
    to the MinimaxPacmanAgent, AlphaBetaPacmanAgent & ExpectimaxPacmanAgent.

    You *do not* need to make any changes here, but you can if you want to
    add functionality to all your adversarial search agents.  Please do not
    remove anything, however.

    Note: this is an abstract class: one that should not be instantiated.  It's
    only partially specified, and designed to be extended.  Agent (game.py)
    is another abstract class.
    """

    def __init__(self, evalFn = 'scoreEvaluationFunction', depth = '2'):
        self.index = 0 # Pacman is always agent index 0
        self.evaluationFunction = util.lookup(evalFn, globals())
        self.depth = int(depth)

class MinimaxAgent(MultiAgentSearchAgent):
    """
    Your minimax agent (question 2)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action from the current gameState using self.depth
        and self.evaluationFunction.

        Here are some method calls that might be useful when implementing minimax.

        gameState.getLegalActions(agentIndex):
        Returns a list of legal actions for an agent
        agentIndex=0 means Pacman, ghosts are >= 1

        gameState.generateSuccessor(agentIndex, action):
        Returns the successor game state after an agent takes an action

        gameState.getNumAgents():
        Returns the total number of agents in the game

        gameState.isWin():
        Returns whether or not the game state is a winning state

        gameState.isLose():
        Returns whether or not the game state is a losing state
        """
        "*** YOUR CODE HERE ***"

        def miniMax(gameState, depth, agentNr):

            #Reset agentNr
            #if agentNr is greater than total number of agents, set agentNr=0 (pacman) and increase depth by 1
            totalNumAgent = gameState.getNumAgents()
            if agentNr >= totalNumAgent:
                depth += 1
                agentNr = 0

            #check termination: reached final layer or win/loss
            if (depth == self.depth or gameState.isWin() or gameState.isLose()):
                return self.evaluationFunction(gameState)

            #agent is pacman   
            elif (agentNr == 0):
                return maxValue(gameState, depth, agentNr)
            
            #agent is ghost
            else:
                return minValue(gameState, depth, agentNr)
        

        def maxValue(gameState, depth, agentNr):

            #set lower bound for maximum value, action is index 0, maxValue index 1
            maxScore = ["", -float("inf")]

            #Collect legal actions
            legalActions = gameState.getLegalActions(agentNr)

            #terminal-test: if game is over
            if not legalActions:
                return self.evaluationFunction(gameState) #utility at current state
                
            #if non-terminal state
            for action in legalActions:
                
                #position after agentNr complete action
                nextState = gameState.generateSuccessor(agentNr,action)

                #score for the next agent as a consequence of action
                adversialNextState = miniMax(nextState,depth,agentNr+1)
        
                #update maxScore
                if type(adversialNextState) is not list: #initial case
                    temp = adversialNextState
                else:
                    temp = adversialNextState[1]

                if temp > maxScore[1]:
                    maxScore = [action, temp]
            return maxScore

    
        def minValue(gameState, depth, agentNr):

            #set lower bound for maximum value  
            minScore = None, float("inf")

            #Collect legal actions
            legalActions = gameState.getLegalActions(agentNr)

            #terminal-test: if game is over
            if not legalActions:
                return self.evaluationFunction(gameState) #utility at current state
                
            #if non-terminal state
            for action in legalActions:
                
                #position after agentNr complete action
                nextState = gameState.generateSuccessor(agentNr,action)

                #score for the next agent as a consequence of this action
                adversialNextState = miniMax(nextState,depth,agentNr+1)
        
                #update minScore
                if type(adversialNextState) is not list: #initial case
                    temp = adversialNextState
                else:
                    temp = adversialNextState[1]

                if temp < minScore[1]:
                    minScore = [action, temp]
            return minScore

        #run minimax algotithm setting depth=0 and agent=pacman
        action = miniMax(gameState, 0, 0)
        
        return action[0] #first element is actions
        

class AlphaBetaAgent(MultiAgentSearchAgent):
    """
    Your minimax agent with alpha-beta pruning (question 3)
    """

    def getAction(self, gameState):
        """
        Returns the minimax action using self.depth and self.evaluationFunction
        """
        "*** YOUR CODE HERE ***"
        def alphaBetaSearch(gameState, depth, agentNr, alpha, beta):

            #Reset agentNr
            #if agentNr is greater than total number of agents, set agentNr=0 (pacman) and increase depth by 1
            totalNumAgent = gameState.getNumAgents()
            if agentNr >= totalNumAgent:
                depth += 1
                agentNr = 0

            #check termination: reached final layer or win/loss
            if (depth == self.depth or gameState.isWin() or gameState.isLose()):
                return self.evaluationFunction(gameState)

            #agent is pacman   
            elif (agentNr == 0):
                return maxValue(gameState, depth, agentNr, alpha, beta)
            
            #agent is ghost
            else:
                return minValue(gameState, depth, agentNr, alpha, beta)
        

        def maxValue(gameState, depth, agentNr, alpha, beta):

            #set lower bound for maximum value, action is index 0, maxValue index 1
            maxScore = ["", -float("inf")]

            #Collect legal actions
            legalActions = gameState.getLegalActions(agentNr)

            #terminal-test: if game is over
            if not legalActions:
                return self.evaluationFunction(gameState) #utility at current state
                
            #if non-terminal state
            for action in legalActions:
                
                #position after agentNr complete action
                nextState = gameState.generateSuccessor(agentNr,action)

                #score for the next agent as a consequence of action
                adversialNextState = alphaBetaSearch(nextState,depth,agentNr+1, alpha, beta)
        
                #update maxScore
                if type(adversialNextState) is not list: #initial case
                    temp = adversialNextState
                else:
                    temp = adversialNextState[1]

                if temp > maxScore[1]:
                    maxScore = [action, temp]

                #pruning
                if temp > beta:
                    return [action, temp]

                #update alpha
                alpha = max(alpha,temp)

            return maxScore

    
        def minValue(gameState, depth, agentNr, alpha, beta):

            #set lower bound for maximum value  
            minScore = None, float("inf")

            #Collect legal actions
            legalActions = gameState.getLegalActions(agentNr)

            #terminal-test: if game is over
            if not legalActions:
                return self.evaluationFunction(gameState) #utility at current state
                
            #if non-terminal state
            for action in legalActions:
                
                #position after agentNr complete action
                nextState = gameState.generateSuccessor(agentNr,action)

                #score for the next agent as a consequence of this action
                adversialNextState = alphaBetaSearch(nextState,depth,agentNr+1, alpha, beta)
        
                #update minScore
                if type(adversialNextState) is not list: #initial case
                    temp = adversialNextState
                else:
                    temp = adversialNextState[1]

                if temp < minScore[1]:
                    minScore = [action, temp]

                #pruning
                if temp < alpha:
                    return [action, temp]

                #update beta
                beta = min(beta,temp)
            return minScore

        #run minimax algotithm setting depth=0 and agent=pacman
        action = alphaBetaSearch(gameState, 0, 0, -float("inf"), float("inf"))
        
        return action[0] #first element is actions
        

class ExpectimaxAgent(MultiAgentSearchAgent):
    """
      Your expectimax agent (question 4)
    """

    def getAction(self, gameState):
        """
        Returns the expectimax action using self.depth and self.evaluationFunction

        All ghosts should be modeled as choosing uniformly at random from their
        legal moves.
        """
        "*** YOUR CODE HERE ***"
        util.raiseNotDefined()

def betterEvaluationFunction(currentGameState):
    """
    Your extreme ghost-hunting, pellet-nabbing, food-gobbling, unstoppable
    evaluation function (question 5).

    DESCRIPTION: <write something here so we know what you did>
    """
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()

# Abbreviation
better = betterEvaluationFunction
