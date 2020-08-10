from interface import Interface, C, Const, Robot
from planGenerator import Generator
from simulation import ManualSimulation


Interface.setup(Const['WINDOW'], 'Plan', font_color=C.BROWN, FPS=30)

Interface.set_robot(Robot((600,600), 0))

g = Generator(Const['WINDOW'])
plans = g.load_plans()

Interface.set_plan(plans[4])

Interface.keep_track_mov(C.YELLOW)

simulation = ManualSimulation()

while Interface.running:
    pressed, events = Interface.run()
    simulation.run(pressed)
    
simulation.store()