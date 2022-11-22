# Tom Jackson Cwk-02: Obstacle Dodging Game
### How to play:

- Run the main.py file
- Press start
- Try to avoid touching the other objects for 10 seconds to progress to the next level
- You are granted invulnerabiliy for the first 3 seconds of each level (indicated by the player being red instead of black)

### Controls:

- Use WSAD to *accelerate* your object (black square) in a specific direction
- Press / to cheat and allow your object to hit the obstacles without losing the game
- Press b to display the boss screen and pause the game
- Press esc to pause the game

### Known Bugs:

- Sometimes the obstacles can glitch out of bounds or get stuck to other each other
-> This occurs when an object collides with a wall close to a corner and is moved through the adjacent wall when calculating the new location
-> To fix would have to add indefinite iteration when calculating the new direction vector to see if it intersects with any other walls (would repeat collision logic until no collision within remaining time/tick)
