from tkinter import *
from math import *
from random import *
from PIL import Image, ImageTk


class ObjectCollision:
    e = 1

    def __init__(self, obj1, obj2, timeOfCollision, obj1Pos, obj2Pos):
        self.obj1 = obj1
        self.obj2 = obj2
        self.t = timeOfCollision
        self.remainingTime = 1 - timeOfCollision
        self.s1 = obj1Pos
        self.s2 = obj2Pos
        self.calcLineOfCentres()
        self.transformVelocities()

    def calcLineOfCentres(self):
        mod = sqrt((self.s1[0] - self.s2[0]) ** 2 + (self.s1[1] - self.s2[1]) ** 2)
        # unit vector representing the line of centres
        self.LOC = [(self.s1[0] - self.s2[0]) / mod, (self.s1[1] - self.s2[1]) / mod]

    def transformVelocities(self):
        v1 = self.obj1.getR()[1]
        v2 = self.obj2.getR()[1]
        CT = [self.LOC[1], -self.LOC[0]]  # common tangent
        self.u1 = [
            v1[0] * self.LOC[0] + v1[1] * self.LOC[1],
            v1[0] * CT[0] + v1[1] * CT[1],
        ]
        self.u2 = [
            v2[0] * self.LOC[0] + v2[1] * self.LOC[1],
            v2[0] * CT[0] + v2[1] * CT[1],
        ]

    def calcObjsAfterV(self):
        mA = self.obj1.getMass()
        mB = self.obj2.getMass()

        e = self.e

        uA = self.u1
        uB = self.u2

        i = mA * uA[0] + mB * uB[0]
        j = e * (uA[0] - uB[0])
        k = mA + mB

        # transformed velocities relative to the line of centres
        tv1 = [(i - mB * j) / k, uA[1]]
        tv2 = [(i + mA * j) / k, uB[1]]

        L1 = self.LOC[0]
        L2 = self.LOC[1]

        l = L1**2 + L2**2

        self.v1 = [(L1 * tv1[0] + L2 * tv1[1]) / l, (L2 * tv1[0] - L1 * tv1[1]) / l]
        self.v2 = [(L1 * tv2[0] + L2 * tv2[1]) / l, (L2 * tv2[0] - L1 * tv2[1]) / l]

        return [self.v1, self.v2]

    def calcObjsAfterS(self):
        return [
            [
                self.s1[0] + self.v1[0] * self.remainingTime,
                self.s1[1] + self.v1[1] * self.remainingTime,
            ],
            [
                self.s2[0] + self.v2[0] * self.remainingTime,
                self.s2[1] + self.v2[1] * self.remainingTime,
            ],
        ]


class WallCollision:
    e = 1

    def __init__(self, wall, object, timeOfCollision, posOfCollision):
        self.wall = wall
        self.object = object
        self.t = timeOfCollision
        self.remainingTime = 1 - timeOfCollision
        self.s = posOfCollision

    def calcObjAfterV(self):
        w = self.wall.getR()[1]
        u = self.object.getR()[1]
        e = self.e
        I = [w[1], -1 * w[0]]

        j = u[0] * w[0] + u[1] * w[1]
        k = I[0] * u[0] + I[1] * u[1]
        l = w[0] * I[1] - w[1] * I[0]

        vx = (I[1] * j + e * w[1] * k) / l
        vy = (-1 * I[0] * j + -1 * e * w[0] * k) / l

        self.objectAfterV = [vx, vy]
        return [vx, vy]

    def calcObjAfterS(self):
        return [
            self.s[0] + self.objectAfterV[0] * self.remainingTime,
            self.s[1] + self.objectAfterV[1] * self.remainingTime,
        ]


class Wall:
    def __init__(self, r, startμ, endμ):
        # vector equation of the wall in form [a, b] for vector a + λb i.e. [[ax,ay], [bx, by]]
        self.r = r
        self.start = startμ
        self.end = endμ

    def getR(self):
        return self.r

    def isCollision(self, objectR, objectRadius):
        # wall a + λb, objectR c + μd
        # note 0 < μ <= 1 for any r and represents the proportion of the tick duration, λ is different
        r = objectRadius
        a = self.r[0]
        b = self.r[1]
        c = objectR[0]
        d = objectR[1]
        g = b[0] ** 2 + b[1] ** 2
        i = r * sqrt(g)
        j = b[0] * c[1] - b[1] * c[0] + a[0] * b[1] - a[1] * b[0]
        k = b[1] * d[0] - b[0] * d[1]
        if k == 0:  # parallel case
            return False, None, None
        pμ = []
        pμ.append((i + j) / k)
        pμ.append((i - j) / -k)
        pμ = list(filter(lambda x: x >= 0 and x < 1, pμ))
        if len(pμ) == 0:
            return False, None, None
        μ = min(pμ)

        l = c[0] - a[0]
        m = c[1] - a[1]
        n = b[0] * d[0] + b[1] * d[1]
        λ = (b[0] * l + b[1] * m + n * μ) / g

        # isCollision, time of collision, location of collision
        return λ >= self.start and λ <= self.end, μ, [c[0] + μ * d[0], c[1] + μ * d[1]]


class Object:
    def __init__(self, initPos, initVel, mass, radius):
        self.f = [0, 0]  # current applied force on object
        self.a = [0, 0]  # current acceleration
        # the vector the object will travel in the next tick [a, b] for vector a + λb
        self.r = [[0, 0], [0, 0]]
        self.s = initPos
        self.v = initVel
        self.m = mass
        self.radius = radius

    def calcNextMove(self):
        self.calcNextA()
        self.calcNextV()
        self.calcNextR()

    def getR(self):
        return self.r

    def getRadius(self):
        return self.radius

    def getMass(self):
        return self.m

    def applyForce(self, f):
        self.f = f

    def calcNextA(self):
        self.a = [self.f[0] / self.m, self.f[1] / self.m]

    def calcNextV(self):
        self.v = [self.v[0] + self.a[0], self.v[1] + self.a[1]]

    def calcNextR(self):
        self.r = [self.s, self.v]

    def override(self, s, v):
        self.s = s
        self.v = v

    def move(self):
        self.s = [self.s[0] + self.v[0], self.s[1] + self.v[1]]

    def isCollision(self, objectR, radius):
        a = objectR[0]
        b = objectR[1]
        c = self.r[0]
        d = self.r[1]
        r2 = radius
        r1 = self.radius

        g = (r1 + r2) ** 2
        i = a[0] - c[0]
        ii = i**2
        j = a[1] - c[1]
        jj = j**2
        k = b[0] - d[0]
        kk = k**2
        l = b[1] - d[1]
        ll = l**2

        if k == 0 and l == 0:
            # same velocity
            return False, None, None, None

        A = kk + ll
        B = 2 * (i * k + j * l)
        C = ii + jj - g

        disc = B**2 - 4 * A * C
        if disc < 0:
            # no solution
            return False, None, None, None
        pλ = []
        pλ.append((-B + sqrt(disc)) / (2 * A))
        pλ.append((-B - sqrt(disc)) / (2 * A))
        pλ = list(filter(lambda x: x >= 0 and x < 1, pλ))
        if len(pλ) == 0:
            # solution but not a valid lambda
            return False, None, None, None
        λ = min(pλ)
        # is Collision?, time of collision, centre of obj1, centre of obj2 (self)
        return (
            True,
            λ,
            [a[0] + λ * b[0], a[1] + λ * b[1]],
            [c[0] + λ * d[0], c[1] + λ * d[1]],
        )


class Obstacle(Object):
    def __init__(self, master, initS, initV):
        self.radius = 10
        super().__init__(initS, initV, 1, self.radius)
        self.obstacleImage = PhotoImage(
            file="images/obstacle.png", height=self.radius * 2, width=self.radius * 2
        )
        self.obstacle = Label(
            master,
            image=self.obstacleImage,
            borderwidth=0,
            highlightthickness=0,
            anchor=CENTER,
        )
        self.updatePos()

    def updatePos(self):  # runs after collision logic
        self.obstacle.place(x=self.s[0] - self.radius, y=self.s[1] - self.radius)

    def tick(self):
        self.calcNextMove()

    def destroy(self):
        self.destroy()


class Player(Object):
    def __init__(self, master):
        self.curKeysPressed = []
        self.radius = 10
        super().__init__([20, 150], [1, 0], 1, self.radius)
        self.playerImage = PhotoImage(
            file="images/player.png", height=self.radius * 2, width=self.radius * 2
        )
        self.invulnerableImage = PhotoImage(
            file="images/playerInv.png", height=self.radius * 2, width=self.radius * 2
        )
        self.player = Label(
            master,
            image=self.playerImage,
            borderwidth=0,
            highlightthickness=0,
            anchor=CENTER,
        )
        self.setInvulnerable(False)
        self.updatePos()

    def updatePos(self):  # runs after collision logic
        self.player.place(
            x=self.s[0] - self.radius, y=self.s[1] - self.radius
        )  # - radius to adjust for tk taking placing top corner on coord

    def getIsInvulnerable(self):
        return self.isInvulnerable

    def setInvulnerable(self, isInv):
        self.isInvulnerable = isInv
        self.player.configure(
            image=(self.invulnerableImage if isInv else self.playerImage)
        )

    def keyPress(self, key):
        if key not in self.curKeysPressed:
            self.curKeysPressed.append(key)

    def keyRelease(self, key):
        if key in self.curKeysPressed:
            self.curKeysPressed.remove(key)

    # runs before collision logic
    def tick(self):
        force = [0, 0]
        for key in self.curKeysPressed:
            if key == "w":
                force[1] -= 0.05
            elif key == "s":
                force[1] += 0.05
            elif key == "a":
                force[0] -= 0.05
            elif key == "d":
                force[0] += 0.05
        self.applyForce(force)
        self.calcNextMove()


class Level:
    def __init__(self, master, levelNum, gameOver):
        self.obstacles = []
        self.walls = []
        self.master = master
        self.levelNum = levelNum
        self.gameOver = gameOver
        self.setupLevel()
        self.startLevel()

    def setupLevel(self):
        self.levelFrame = Frame(self.master, width=600, height=300, bg="blue")
        self.levelFrame.pack_propagate(False)
        self.player = Player(self.levelFrame)

        self.walls.append(Wall([[0, 0], [1, 0]], 0 - 10, 600 + 10))
        self.walls.append(Wall([[0, 0], [0, 1]], 0 - 10, 300 + 10))
        self.walls.append(Wall([[600, 300], [-1, 0]], 0 - 10, 600 + 10))
        self.walls.append(Wall([[600, 300], [0, -1]], 0 - 10, 300 + 10))

        obs = []
        while len(obs) < self.levelNum:
            radius = 10
            isInvalid = True
            while isInvalid:
                if len(obs) == 0:
                    isInvalid = False
                obR = [
                    [randint(radius, 600 - radius), randint(radius, 300 - radius)],
                    [random() * self.levelNum, random() * self.levelNum],
                ]
                i = 0
                while i < len(obs) and isInvalid:
                    isInvalid = obs[i].isCollision(obR, radius)[
                        0
                    ]  # hard coded obstacle radius
                    i += 1
                if not isInvalid or len(obs) == 0:
                    obs.append(
                        Obstacle(
                            self.levelFrame,
                            obR[0],
                            obR[1],
                        )
                    )

            self.obstacles = obs

    def tick(self):
        self.player.tick()
        for o in self.obstacles:
            o.tick()

        # check for player collisions with obstacles if not invulnerable
        if not self.player.getIsInvulnerable():
            isCollision = False
            for ob in self.obstacles:
                isCollision, TOC, s1, s2 = self.player.isCollision(
                    ob.getR(), ob.getRadius()
                )
                if isCollision:
                    self.levelFrame.place_forget()
                    return self.gameOver()

        # check for any collisions and calculate them
        liveObjects = [self.player, *self.obstacles]
        while len(liveObjects) > 0:
            o = liveObjects[0]
            liveObjects.pop(0)
            isCollision = False

            # proccess wall collisions
            i = 0
            wallCollisions = []
            while i < len(self.walls) and len(wallCollisions) < 2:
                isWallCol, TOC, LOC = self.walls[i].isCollision(o.getR(), o.getRadius())
                if isWallCol:
                    wallCollisions.append([self.walls[i], TOC, LOC])
                i += 1

            if len(wallCollisions) == 2:  # case where object hits a corner
                isCollision = True
                w, TOC, LOC = wallCollisions[0]
                v = [-o.getR()[1][0], -o.getR()[1][1]]
                t = 1 - TOC
                s = [LOC[0] + v[0] * t, LOC[1] + v[1] * t]
                o.override(s, v)

            if len(wallCollisions) == 1:
                isCollision = True
                w, TOC, LOC = wallCollisions[0]
                wallCol = WallCollision(w, o, TOC, LOC)
                afterV = wallCol.calcObjAfterV()
                afterS = wallCol.calcObjAfterS()
                o.override(afterS, afterV)

            # if not collided with a wall check for object collisions
            if not isCollision:
                i = 0
                while i < len(liveObjects) and not isCollision:
                    o2 = liveObjects[i]
                    isCollision, TOC, s1, s2 = o2.isCollision(o.getR(), o.getRadius())
                    if isCollision:
                        obCol = ObjectCollision(o, o2, TOC, s1, s2)
                        v1, v2 = obCol.calcObjsAfterV()
                        s1, s2 = obCol.calcObjsAfterS()
                        o.override(s1, v1)
                        o2.override(s2, v2)
                        liveObjects.pop(i)
                    i += 1
            o.move()
            o.updatePos()

    def onKeyPress(self, key):
        self.player.keyPress(key)

    def onKeyRelease(self, key):
        self.player.keyRelease(key)

    def startLevel(self):
        self.levelFrame.place(relx=0.5, rely=0.5, anchor=CENTER)
        self.player.setInvulnerable(True)

    def setPlayerIsInvulnerable(self, isInv):
        self.player.setInvulnerable(isInv)

    def pause(self):
        self.levelFrame.place_forget()

    def resume(self):
        self.levelFrame.place(relx=0.5, rely=0.5, anchor=CENTER)


class Game:
    def __init__(self, master):
        self.isPaused = False
        self.levelNumber = 1
        self.isGameOver = False
        self.time = 0
        self.master = master
        self.isCheat = False

    def tick(self):
        # rough estimate of times as losing time with processing
        if not self.isGameOver and not self.isPaused:
            self.time += 5
            if self.time >= 1500 and not self.isCheat:
                self.level.setPlayerIsInvulnerable(False)
            self.level.tick()
            if self.time == 5000:
                self.nextLevel()
            self.master.after(5, self.tick)

    def startGame(self):
        self.levelNumber = 1
        self.isGameOver = False
        self.time = 0
        self.startLevel()
        self.tick()

    def startLevel(self):
        self.master.title("Level " + str(self.levelNumber))
        self.level = Level(self.master, self.levelNumber, self.gameOver)
        self.level.startLevel()

    def nextLevel(self):
        self.isCheat = False
        self.time = 0
        self.levelNumber += 1
        self.startLevel()

    def gameOver(self):
        self.gameOverScreen = GameOverScreen(
            self.master, self.startGame, (self.levelNumber - 1) * 5000 + self.time
        )
        self.isGameOver = True
        self.gameOverScreen.show()

    def onKeyPress(self, key):
        if key == "/" and self.time >= 1500 and not self.isCheat:
            self.isCheat = True
            self.level.setPlayerIsInvulnerable(True)
        else:
            self.level.onKeyPress(key)

    def onKeyRelease(self, key):
        if key == "/" and self.time >= 1500 and self.isCheat:
            self.isCheat = False
            self.level.setPlayerIsInvulnerable(False)
        else:
            self.level.onKeyRelease(key)

    def pause(self):
        self.level.pause()
        self.isPaused = True

    def resume(self):
        self.isPaused = False
        self.tick()
        self.level.resume()


class GameOverScreen:
    def __init__(self, master, retry, score):
        self.frame = Frame(master, width=600, height=300, bg="gray")
        self.retry = retry
        msg = Label(self.frame, text="Game Over")
        msg.pack()
        displayScore = Label(self.frame, text="Your score was: " + str(score))
        displayScore.pack()
        retryButton = Button(self.frame, text="Try Again?", command=self.retry)
        retryButton.place(relx=0.5, rely=0.5, anchor=CENTER)

        self.inputLabel = Label(self.frame, text="Enter your name: ")
        self.inputLabel.pack()
        self.inputBox = Entry(self.frame)
        self.inputBox.pack()
        self.submitButton = Button(
            self.frame, text="Submit Score", command=lambda: self.submitScore(score)
        )
        self.submitButton.pack()

        self.frame.pack_propagate(False)

    def submitScore(self, score):
        if self.inputBox.get() != "":
            with open("leaderboard.txt", "a+") as f:
                f.write(self.inputBox.get() + "," + str(score) + "\n")
            self.inputLabel.destroy()
            self.inputBox.destroy()
            self.submitButton.destroy()

    def show(self):
        self.frame.place(relx=0.5, rely=0.5, anchor=CENTER)


class StartMenu:
    def __init__(self, master, start):
        self.startFrame = Frame(master, width=600, height=300, bg="gray")
        self.start = start

        self.leaderBoard = Label(self.startFrame)
        self.leaderBoard.place(relx=0.5, rely=0.5, anchor=CENTER)

        startButton = Button(self.startFrame, text="Start", command=self.hide)
        startButton.pack()

        self.startFrame.pack_propagate(False)

        with open("leaderboard.txt", "r+") as f:
            scores = [
                [l.strip().split(",")[0], int(l.strip().split(",")[1])]
                for l in f.readlines()
            ]
            scores = sorted(scores, key=lambda x: x[1], reverse=True)
            if len(scores) > 5:
                scores = scores[:5]
            elif len(scores) < 5:
                for i in range(5 - len(scores)):
                    scores.append(["", ""])
            self.scores = scores

    def hide(self):
        self.start()
        self.startFrame.place_forget()

    def show(self):
        Label(self.leaderBoard, text="Leaderboard").pack()
        Label(self.leaderBoard, text="Name, Score:").pack()
        for i, s in enumerate(self.scores):
            Label(self.leaderBoard, text=s[0] + ", " + str(s[1])).pack()
        self.startFrame.place(relx=0.5, rely=0.5, anchor=CENTER)


class PauseMenu:
    def __init__(self, master, resumeCb):
        self.resumeCb = resumeCb
        self.menuFrame = Frame(master, width=600, height=300, bg="gray")

        resumeButton = Button(self.menuFrame, text="Resume", command=self.hide)
        resumeButton.place(relx=0.5, rely=0.5, anchor=CENTER)

        self.menuFrame.pack_propagate(False)

    def hide(self):
        self.resumeCb()
        self.menuFrame.place_forget()

    def show(self):
        self.menuFrame.place(relx=0.5, rely=0.5, anchor=CENTER)


class BossScreen:
    def __init__(self, master):
        self.img = PhotoImage(file="images/bossScreen.png")
        self.frame = Frame(
            master,
            width=600,
            height=300,
        )
        self.screen = Label(
            master,
            image=self.img,
            borderwidth=0,
            highlightthickness=0,
            anchor=CENTER,
        )
        self.frame.pack_propagate(False)

    def hide(self):
        self.screen.place_forget()

    def show(self):
        self.screen.place(relx=0.5, rely=0.5, anchor=CENTER)


class Window:
    allowedKeys = ["w", "a", "s", "d", "/"]

    def __init__(self):
        self.root = Tk()
        self.root.geometry("800x400")
        self.root.focus()
        self.game = Game(self.root)

        self.startMenu = StartMenu(self.root, self.game.startGame)
        self.startMenu.show()

        self.pauseMenu = PauseMenu(self.root, self.onHidePauseMenu)
        self.bossScreen = BossScreen(self.root)
        self.root.bind("<Escape>", self.showPauseMenu)
        self.root.bind("b", self.showBossScreen)
        self.root.bind("<KeyPress>", self.keyDown)
        self.root.bind("<KeyRelease>", self.keyUp)

    def keyDown(self, event):
        if event.char in self.allowedKeys:
            self.game.onKeyPress(event.char)

    def keyUp(self, event):
        if event.char == "b":
            self.hideBossScreen()
        elif event.char in self.allowedKeys:
            self.game.onKeyRelease(event.char)

    def open(self):
        self.root.mainloop()

    def onHidePauseMenu(self):
        self.game.resume()

    def showPauseMenu(self, event):
        self.game.pause()
        self.pauseMenu.show()

    def hideBossScreen(self):
        self.bossScreen.hide()
        self.game.resume()

    def showBossScreen(self, event):
        self.game.pause()
        self.bossScreen.show()


if __name__ == "__main__":
    window = Window()
    window.open()
