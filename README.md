# 🌱🤖🚰 Watering Problem 
This project is an **AI-Agent** which has one goal: Finding the optimal way to water plants in the watering problem  
<br><br>
## 🤔 What Even is a Watering Problem? 
A watering problem consists of an $N\times M$ **grid**, which contatins **robots, plants, taps and walls**.  
🤖 Each robot has: 
1. 🚧  **capacity**: The maximal number of Water-Units it can hold at any given moment
2. 📦  **load**:  he current number of Water-Units the robot holds.
   
🌱 Each Plant has: 
1. 💦 **water_needed**: The number of Water-Units the plant needs in order to be fully saturated
   
🚰 Each Tap has:
1. 💦 **water_available**: The number of Water-Units the tap has left to giveֿ

<br><br>
### 📜 Rules:
1. Each step, only one robot is allowed to make a move: UP, DOWN, LEFT, RIGHT, LOAD, POUR
2. All robots begin with load = 0
3. A robot cannot move to a square which has a wall, another robot, or outside the boundaries of the grid
   
<br><br>
### Example of a watering problem:

<p align="center">
  <img src="images/grid_example.jpeg" width="450">
</p>

<br><br>
## 🌟 What Even is an _Optimal_ Solution?
Excellent question!  
Before we discuss the strategies to solve the problem, we need to begin by defining what our goal even is.  
This agent is able to support these two types of 
