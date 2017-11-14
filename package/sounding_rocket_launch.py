"""Needed pieces for the rocket launch simulator"""
#import scipy
import matplotlib.pyplot as plt
import numpy as np
"""----------Imported functions--------------"""
#import math.sin as sin()
"""--------------CONSTANTS-----------------"""
STANDARD_GRAVITY = 9.80665  # m/s^2
G = 6.674 * 10**-11
MASS_EARTH = 5.972 * 10**24  #kilograms
RADIUS_EARTH = 6.371 * 10**6  #meters
TIME_STEP = .01  #seconds
e = 2.71828
#MAX = 10000 # number of iterations, = 1000 second launch program
"""---------------VARIABLES---------------"""
thrust = 0.0
motor_isp = 0.0
altitude = 0.0
velocity = 0
v_x = 0.0
v_y = 0.0
angle_of_attack = 0.0
time_to_MECO = 0.0
mass_flow = 0.0
dry_mass = 0.0
wet_mass = 0.0
force_gravity = 9.81 * (wet_mass)
"""------------PROGRAMS--------------"""


def Vacuum_dV(motor_isp, wet_mass, dry_mass):
    #    print(wet_mass, dry_mass, motor_isp, STANDARD_GRAVITY)
    deltaV = STANDARD_GRAVITY * motor_isp * np.log(wet_mass / dry_mass)
    print("Total delta-V is %.2f m/s \n\n") % deltaV
    #mF = 5300 / (STANDARD_GRAVITY * motor_isp)
    #    print("Burn time is %.2f s") % mF
    return deltaV


def Mass_of_spaceship(wet_mass, mass_flow, mass_ship, i):
    """Calculates the mass of the rocket at time t

    Args:
        wet_mass (float): The mass of the ship and fuel at time t=0.
        mass_flow (float): Amount of fuel expelled per timestep
        i (int): Iterator used for Euler's Method

    Returns:
        mass_ship (float): The new mass of the ship at timestep i.

    """

    mass_ship = wet_mass - mass_flow * TIME_STEP * i
    return mass_ship


def Force_Gravity(mass_ship, altitude):
    """Calculates the force of gravity acting on the ship.

    Args:
        mass_ship (float): The mass of the ship at timestep i.
        altitude (float): The altitude of the rocket above Mean Sea level

    Returns:
        force_gravity (float): Calculated force of gravity at timestep i.
    """

    force_gravity = G * mass_ship * MASS_EARTH / ((RADIUS_EARTH + altitude)**2)
    return force_gravity


def Thrust_ship(thrust, motor_isp, mass_flow):
    """Calculates thrust from the Rocket eqation: Thrust = Isp * g0 * massflow

    Args:
        motor_isp (float): Effiency constant for the motor.
        mass_flow (float): Total mass flow of exhaust.

    Returns:
        thrust (float): Thrust force produced by the rocket.

     """
    thrust = motor_isp * STANDARD_GRAVITY * mass_flow
    return thrust


def Acceleration(thrust, force_gravity, mass_ship, force_drag):
    """Calculates the acceleration at timestep t

    Args:
        force_gravity (float): force of gravity at timestep i
        mass_ship (float): The mass of the ship at timestep i
        force_drag (float): Force of drag at timestep i

    Returns:
        acceleration (float): Acceleration vector from a=dF/dm

    """
    acceleration = (
        thrust - force_gravity - force_drag) / mass_ship  # """ - force_drag"""
    return acceleration


def Atmosphere_Density(altitude):

    h = altitude
    """CONSTANTS FOR GASSES"""
    R = 8.31432  # ideal gas constant J/(mol*K)
    M = .0289644  # molar mass of dry air, kg/mol
    """SUBSCRIPT TABLES"""
    pb = [1.2250, 0.36391, 0.08803, 0.01322, 0.00143, 0.00086, 0.000064]
    hb = [0.0, 11000, 20000, 32000, 47000, 51000, 71000]
    Tb = [288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65]
    Lb = [-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002]

    """----EQUATIONS FOR AIR DENSITY------- Taken from the Barometric formula. Accurate up to 80,000m above sea level
    If temperature step != 0
    rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb))) ** (1 + (STANDARD_GRAVITY * M) / (R * Lb))

    If temperature step == 0
    rho = pb[i] * e ** ((-1* (STANDARD_GRAVITY) * M * (h - hb[i])) / (R * Tb[i]))
    """#-------------------------------------

    if altitude < 11000:
        i = 0
        rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb[i])))**(
            1 + (STANDARD_GRAVITY * M) / (R * Lb[i]))
        return rho

    elif altitude > 11000 and altitude < 20000:
        i = 1
        rho = pb[i] * e**(
            (-1 * (STANDARD_GRAVITY) * M * (h - hb[i])) / (R * Tb[i]))
        return rho

    elif altitude > 20000 and altitude < 32000:
        i = 2
        rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb[i])))**(
            1 + (STANDARD_GRAVITY * M) / (R * Lb[i]))
        return rho

    elif altitude > 32000 and altitude < 47000:
        i = 3
        rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb[i])))**(
            1 + (STANDARD_GRAVITY * M) / (R * Lb[i]))
        return rho

    elif altitude > 47000 and altitude < 51000:
        i = 4
        rho = pb[i] * e**(
            (-1 * (STANDARD_GRAVITY) * M * (h - hb[i])) / (R * Tb[i]))
        return rho

    elif altitude > 51000 and altitude < 71000:
        i = 5
        rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb[i])))**(
            1 + (STANDARD_GRAVITY * M) / (R * Lb[i]))
        return rho

    elif altitude > 71000 and altitude < 86000:
        i = 6
        rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb[i])))**(
            1 + (STANDARD_GRAVITY * M) / (R * Lb[i]))
        return rho

    else:
        return 0


def Drag(density, velocity, reference_area):
    # drag = coefficient * density * velocity^2 * reference area / 2
    cd = .75  #drag coefficient
    A = reference_area
    drag = .5 * cd * density * A * velocity**2
    return drag


def Velocity(velocity, acceleration, i):  # F = m*a
    velocity_new = velocity + acceleration * TIME_STEP
    #v_x = velocity * np.sin(theta)
    #    v_y = velocity * np.cos(theta)

    # acceleration = (thrust - drag) / mass_ship
    return velocity_new


def Altitude(altitude, velocity, i):
    altitude = altitude + velocity * (TIME_STEP)  # * sin(pitch)
    return altitude


def Position_downrange(downrange, velocity, angle_of_attack):
    down_range = downrange + velocity * cos(angle_of_attack) * TIME_STEP
    return down_range


#def Angle_Of_Attack():
def pitch(theta, altitude, velocity):
    if velocity < 100:
        return 90

    elif velocity > 100 and velocity < 300:
        theta_max = 0


def free_fall_acceleration(force_gravity, mass_ship, force_drag):
    fall_acceleration = (-force_gravity + force_drag) / mass_ship
    return fall_acceleration


def Apogee(velocity):
    apogee = velocity**2 / (2 * STANDARD_GRAVITY)
    return apogee


def Main_simulation(thrust, motor_isp, mass_flow, dry_mass, wet_mass):
    dV = Vacuum_dV(motor_isp, wet_mass, dry_mass)

    i = 0
    altitude = 0.0
    velocity = 0.0
    force_drag = 0.0
    theta = 0.0
    mass_ship = wet_mass
    y_max = 0.0
    """----LISTS TO GRAPH----"""
    mass = []
    acceleration_rocket = []
    speed = []
    height = []
    time_passed = []
    drag = []
    dens_rho = []
    pos_x = []
    a = []
    while mass_ship > dry_mass:  # and y_max < 120000:

        mass_ship = Mass_of_spaceship(wet_mass, mass_flow, mass_ship, i)
        mass.append(mass_ship)

        force_gravity = Force_Gravity(mass_ship, altitude)

        density = Atmosphere_Density(altitude)
        dens_rho.append(density)

        force_drag = Drag(density, velocity, reference_area)
        drag.append(force_drag)

        acceleration = Acceleration(thrust, force_gravity, mass_ship,
                                    force_drag)  #, force_drag)
        acceleration_rocket.append(acceleration)

        velocity = Velocity(velocity, acceleration, i)
        speed.append(velocity)

        y_max = Apogee(velocity)
        a.append(y_max)
        #print y_max
        #if velocity > 100 and velocity < 140:
        #    print ("T + %.2f s, %.2f m/s ") % (time,velocity)

        altitude = Altitude(altitude, velocity, i)
        height.append(altitude)

        #downrange = Position_downrange(downrange, velocity)
        """This here is the time calculations"""
        time = i * TIME_STEP
        time_passed.append(time)
        i += 1

    gravity_loss = dV - velocity
    print "Final velocity: %.2f m/s \nTotal dV lost: %.2f m/s" % (velocity,
                                                                  gravity_loss)

    print "\n\n---MECO---\n\n"

    while altitude > 0:
        mass_ship = dry_mass  #- 1400

        force_gravity = Force_Gravity(mass_ship, altitude)

        density = Atmosphere_Density(altitude)
        dens_rho.append(density)

        force_drag = Drag(density, velocity, reference_area)
        drag.append(force_drag)

        acceleration = free_fall_acceleration(force_gravity, mass_ship,
                                              force_drag)
        #abc = abs(acceleration)
        acceleration_rocket.append(acceleration)

        velocity = Velocity(velocity, acceleration, i)
        speed.append(velocity)

        altitude = Altitude(altitude, velocity, i)
        height.append(altitude)
        #print ("Altitude: %.2f m, Drag: %.2f N, Acceleration: %.2f m/s, Velocity: %.2f m/s") % (altitude, force_drag, acceleration, velocity)
        #downrange = Position_downrange(downrange, velocity)

        time = i * TIME_STEP
        time_passed.append(time)

        i += 1

    print("Apogee: %.2f m") % max(height)
    print("Max Velocity: %.2f m/s") % max(speed)
    print("Min Velocity: %.2f m/s") % min(speed)
    print("Max Acceleration: %.2f m/s^2") % max(acceleration_rocket)
    print("Max Drag: %.2f N") % max(drag)
    #print("y max: %.2f m") % max(a)

    plt.subplot(4, 1, 1)
    plt.plot(time_passed, acceleration_rocket)
    plt.ylabel("Acceleration (m/s^2)")
    #plt.xlabel("Time (s)")
    plt.title("Change in Acceleration of Rocket")

    plt.subplot(4, 1, 2)
    plt.plot(time_passed, height, 'c')
    plt.ylabel("Altitude")

    plt.subplot(4, 1, 3)
    #plt.title("Drag profile for rocket launch")
    plt.plot(time_passed, drag, 'r')
    #plt.bar(height, dens_rho)
    plt.ylabel("Drag (N)")

    plt.subplot(4, 1, 4)
    plt.plot(time_passed, speed)
    plt.ylabel("Velocity (m/s)")
    plt.xlabel("Time (s)")

    plt.show()


def initialize_variables(thrust, motor_isp, mass_flow, dry_mass, wet_mass):
    #current values for falcon 9 booster
    thrust = float(
        5300)  #float(raw_input("What is the total thrust? (in newtons) "))
    motor_isp = float(
        315)  #float(raw_input("What is the motor\'s vacuum isp? "))
    mass_flow = thrust / (motor_isp * STANDARD_GRAVITY)

    dry_mass = float(
        30
    )  #127000#13600 #float(raw_input("What is the dry mass of your ship? "))
    wet_mass = float(
        40
    )  #510000#27300 #float(raw_input("What is the wet mass of your ship? "))

    reference_area = 3.14159.pi * .1**2  # pi*r^2
    return dry_mass, wet_mass, mass_flow, thrust, motor_isp, reference_area


"""----------------------------------MAIN PROGRAM----------------------------------------"""

dry_mass, wet_mass, mass_flow, thrust, motor_isp, reference_area = initialize_variables(
    thrust, motor_isp, mass_flow, dry_mass, wet_mass)

Main_simulation(thrust, motor_isp, mass_flow, dry_mass, wet_mass)

#
