import pyMarkovSimulator
from random import randint
import keyboard

nr_obs = 14
seed = 404
env = pyMarkovSimulator.pyMarkovSimulator(nr_obs, seed)

env.update_plot()
print("Program instructions:")
print(20*'-')
print("Order windows on screen as desired. ")
print("Use keys WAD to steer the robot [fowrard, left, right]")
print("Press R key to get sensor reading")
print('To END SIM, press S key')
print(20*'-')
input('Press enter when ready')
looping = True

while looping:

    env.update_plot()
    env.update_plot()

    action = randint(1,5)
    print('wait for command')
    while True:
        if keyboard.is_pressed("w"):
            env.move_step('forward')
            break
        if keyboard.is_pressed("a"):
            env.move_step('turn left')
            break
        if keyboard.is_pressed("d"):
            env.move_step('turn right')
            break
        if keyboard.is_pressed("r"):
            env.sense_step()
            break
        if keyboard.is_pressed("t"):
            looping = False
            break

    if not(looping):
        break
    

print("Simulation ended!")
