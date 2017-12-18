#"""Needed pieces for the rocket launch simulator"""
#import scipy
#"""----------Imported functions--------------"""
#import math.sin as sin()
#"""--------------CONSTANTS-----------------"""
# STANDARD_GRAVITY = 9.80665  # m/s^2
# G = 6.674 * 10**-11
# MASS_EARTH = 5.972 * 10**24  #kilograms
# RADIUS_EARTH = 6.371 * 10**6  #meters
# TIME_STEP = .1  #seconds
# e = 2.71828
# #MAX = 10000 # number of iterations, = 1000 second launch program
# #---------------VARIABLES---------------
# thrust = 0.0
# motor_isp = 0.0
# altitude = 0.0
# velocity = 0
# v_x = 0.0
# v_y = 0.0
# angle_of_attack = 0.0
# time_to_MECO = 0.0
# mass_flow = 0.0
# dry_mass = 0.0
# wet_mass = 0.0
# force_gravity = 9.0665 * (wet_mass)
#------------PROGRAMS--------------


def Vacuum_dV(motor_isp, wet_mass, dry_mass):
    """Calculates the total available vacuum-dv by the Tsiakovlsy rocket equation

    Args:
        motor_isp (float): The ISP rating of the motor_isp
        wet_mass (float): Total mass of the fully fueld Rocket
        dry_mass (float): Mass of the empty Rocket

    Returns:
        deltaV (float): Total meters per second of dV available.

    """

    import numpy as np
    #    print(wet_mass, dry_mass, motor_isp, STANDARD_GRAVITY)
    deltaV = 9.0665 * motor_isp * np.log(wet_mass / dry_mass)
    print("Total delta-V is ")
    print(deltaV)
    print(" m/s \n\n")
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
    TIME_STEP = .1
    mass_ship = wet_mass - mass_flow * TIME_STEP * i

    return mass_ship


def Force_Gravity(mass_ship, altitude):
    """Calculates the force of gravity acting on the ship at altitude in meters

    Args:
        mass_ship (float): The mass of the ship at timestep i.
        altitude (float): The altitude of the rocket above Mean Sea level

    Returns:
        force_gravity (float): Calculated force of gravity at timestep i.
    """

    G = 6.674 * 10**-11
    MASS_EARTH = 5.972 * 10**24
    RADIUS_EARTH = 6.371 * 10**6  #meters
    STANDARD_GRAVITY = 9.80665  # m/s^2


    if mass_ship < 0:
        raise NameError("Mass error")

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
    STANDARD_GRAVITY = 9.80665  # m/s^2



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
    """Calculates the density of the atmosphere at any altitude for use in the drag equation"

    Args:
        altitude (float): The current altitude in meters above sea level

    Returns:
        rho (float): The density of the air in kg/m^3 at altitude
    """

    h = altitude
    # """CONSTANTS FOR GASSES"""
    R = 8.31432  # ideal gas constant J/(mol*K)
    M = .0289644  # molar mass of dry air, kg/mol
    STANDARD_GRAVITY = 9.80665  # m/s^2
    e = 2.71828


    #"""SUBSCRIPT TABLES"""
    pb = [1.2250, 0.36391, 0.08803, 0.01322, 0.00143, 0.00086, 0.000064]
    hb = [0.0, 11000, 20000, 32000, 47000, 51000, 71000]
    Tb = [288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65]
    Lb = [-0.0065, 0.0, 0.001, 0.0028, 0.0, -0.0028, -0.002]

    # ----EQUATIONS FOR AIR DENSITY------- Taken from the Barometric formula. Accurate up to 80,000m above sea level
    # If temperature step != 0
    # rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb))) ** (1 + (STANDARD_GRAVITY * M) / (R * Lb))
    #
    # If temperature step == 0
    # rho = pb[i] * e ** ((-1* (STANDARD_GRAVITY) * M * (h - hb[i])) / (R * Tb[i]))

    # The altitude model used below is based on the standard atmospheric model used in modern meteorology.
    # It takes into accout the different regression rates and properties of the thermoclines.

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
        rho = pb[i] * (Tb[i] / (Tb[i] + Lb[i] * (h - hb[i])))**(    1 + (STANDARD_GRAVITY * M) / (R * Lb[i]))
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
    """Calculates the drag force acting on the rocket.

    Args:
        density (float): From rocket.Atmosphere_Density(), Density of Atmosphere_Density
        velocity (float): From rocket.Velocity(), velocity at timestep i
        reference_area (float): Constant defined for the cross section of the Rocket

    Returns:
        drag (float): Drag force acting on the rocket in Newtons.
            Equation::drag = coefficient * density * velocity^2 * reference area / 2

    """
    cd = .75  #drag coefficient
    A = reference_area
    drag = .5 * cd * density * A * velocity**2
    return drag


def Velocity(velocity, acceleration, i):
    """Defines the velocity at timestep i.

    Args:
        velocity (float): Initial velocity = 0. Recursively updates the velocity_new
        acceleration (float): From rocket.Acceleration(), Instantaneous acceleration.
        i (int): Iterator used for Euler's Method

    Returns:
        velocity_new (float): Updated velocity. Will replace velocity for next iteration
    """

    # F = m*a
    velocity_new = velocity + acceleration * TIME_STEP
    #v_x = velocity * np.sin(theta)
    #    v_y = velocity * np.cos(theta)

    # acceleration = (thrust - drag) / mass_ship
    return velocity_new


def Altitude(altitude, velocity, i):
    """Returns the altitude at timestep i.

    Args:
        altitude (float): Altitude calculated at timestep i-1
        velocity (float): From rocket.Velocity(), The Instantaneous velocity of the rocket
        i (int): Iterator used for Euler's method

    Returns:
        altitude (float): Altitude calculated at timestep i
    """

    altitude = altitude + velocity * (TIME_STEP)  # * sin(pitch)
    return altitude


def Position_downrange(downrange, velocity, angle_of_attack):
    """ **Still under development**
    Calculates the position in the x direction from the launch site based on an ascent profile.

    Args:
        downrange (float): Position downrange calculated at timestep i-1. Initial value = 0
        velocity (float): From rocket.Velocity(): The Instantaneous velocity of the Rocket
        angle_of_attack (float): From rocket.Angle_of_attack(): Angle of the rocket from the vertical.

    Returns:
        down_range (float): Position downrange at timestep i

    """

    down_range = downrange + velocity * cos(angle_of_attack) * TIME_STEP
    return down_range


#def Angle_Of_Attack():
#def pitch(theta, altitude, velocity):
#    if velocity < 100:
#        return 90

#    elif velocity > 100 and velocity < 300:
#        theta_max = 0


def free_fall_acceleration(force_gravity, mass_ship, force_drag):
    """This function determines the downward force of gravity with respect to the
    distance of the ship to the earth.

    Args:
        force_gravity (float): The force of gravity acting on the earth
        mass_ship (float): The mass of the ship at timestep i
        force_drag (float): The drag force acting against ship

    Returns:
        fall_acceleration (float): The acceleration of the ship at timestep i.
    """

    fall_acceleration = (-force_gravity + force_drag) / mass_ship
    return fall_acceleration


def Apogee(velocity):
    """This function calculates the highest point in the rocket's trajectory as
    a function of its instantaneous velocity.

    Args:
        velocity (float): from rocket.Velocity(): Current velocity of the Rocket

    Returns:
        apogee (float): Highest predicted altitude

    """
    apogee = velocity**2 / (2 * STANDARD_GRAVITY)
    return apogee


def Main_simulation(thrust, motor_isp, mass_flow, dry_mass, wet_mass):
    """This function is the main simulation package. It calls each of the
    necessary functions to calculate the position of the rocket for the duration
    of the flight.

    Each of the returned values from each function will be stored in an array
    which will later be used to plot the results of the simulation.

    Args:
        thrust (float): Thrust force of the Rocket
        motor_isp (float): Motor efficiency number
        mass_flow (float): Mass flow per time step i
        dry_mass (float): Dry mass of the Rocket
        wet_mass (float): Mass of the Fully fueled Rocket

    Returns:
        N/A

    """

    import matplotlib.pyplot as plt
    import numpy as np


     #kilograms
    TIME_STEP = .1  #seconds
    e = 2.71828
    #MAX = 10000 # number of iterations, = 1000 second launch program
    #---------------VARIABLES---------------
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
    force_gravity = 9.0665 * (wet_mass)


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
        #This here is the time calculations
        time = i * TIME_STEP
        time_passed.append(time)
        i += 1

    gravity_loss = dV - velocity
    # print ("Final velocity: ")
    # print(velocity)
    # print(" m/s")
    # print("Total dV lost: ")
    # print("gravity_loss")
    # print(" m/s")
    #
    # print ("\n\n---MECO---\n\n")
    #

    import time as t
    while altitude > 0:
        #print('a', altitude)
        # sprint('v',velocity)
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
        print("t+",time,"s")
        time_passed.append(time)
        print(altitude)
        # break
        i += 1

    #print("Apogee: %.2f m") % max(height)
    #print("Max Velocity: %.2f m/s") % max(speed)
    #print("Min Velocity: %.2f m/s") % min(speed)
    #print("Max Acceleration: %.2f m/s^2") % max(acceleration_rocket)
    #print("Max Drag: %.2f N") % max(drag)
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
    """This function initializes the values for the rocket that will be used in the
    simulation. Specifics are given alongside the value.


    """
    #current values for falcon 9 booster
    thrust = float(
        490000)  # Motor thrust in Newtons
    motor_isp = float(
        335)  # Motor ISP
    mass_flow = thrust / (motor_isp * STANDARD_GRAVITY)

    dry_mass = float(
        17000
           )  # Dry mass in kg
    wet_mass = float(
        40000
    )  # Wet mass in kg

    reference_area = 3.14159 * .05**2  # This is the cross sectional profile of the rocket
    return dry_mass, wet_mass, mass_flow, thrust, motor_isp, reference_area


#----------------------------------MAIN PROGRAM----------------------------------------

# dry_mass, wet_mass, mass_flow, thrust, motor_isp, reference_area = initialize_variables(
#     thrust, motor_isp, mass_flow, dry_mass, wet_mass)
#
# Main_simulation(thrust, motor_isp, mass_flow, dry_mass, wet_mass)

#
