try:
    import framework32 as framework
except:
    import framework64 as framework

import math
import random


# -------------------------------------------------------
#	game parameters
# -------------------------------------------------------

class Params(object):

    class Ship(object):
        LINEAR_SPEED = 0.5
        ANGULAR_SPEED = 0.5
        DECELERATION = LINEAR_SPEED * 1.1
        ACCELERATION = LINEAR_SPEED
        LENGTH_OF_RUNWAY = 0.35
        MAX_NUMBER_OF_AIRCRAFTS = 5
        GOAL_INITIAL_POSITION_X = 0
        GOAL_INITIAL_POSITION_Y = 0

    class Aircraft(object):
        LINEAR_SPEED = 2.0
        ANGULAR_SPEED = 2.5
        MAX_FLYING_TIME = 10.0
        NECESSARY_TIME_FOR_REFUEL_AFTER_LANDING = 4.0
        ACCELERATION = LINEAR_SPEED
        DECELERATION = LINEAR_SPEED * 0.9



# -------------------------------------------------------
#	Basic Vector2 class
# -------------------------------------------------------

class Vector2(object):

    def __init__(self, *args):
        if not args:
            self.x = self.y = 0.0
        elif len(args) == 1:
            self.x, self.y = args[0].x, args[0].y
        else:
            self.x, self.y = args

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __mul__(self, coef):
        return Vector2(self.x * coef, self.y * coef)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def length(self):  # Magnitude (length) of the vector
        magnitude = math.sqrt(self.x ** 2 + self.y ** 2)
        return magnitude

    def normalized(self):  # Returns the unit vector of the given vector
        length = self.length()
        return Vector2(self.x / length, self.y / length) if length != 0 else Vector2()

# -------------------------------------------------------
#	Simple ship logic
# -------------------------------------------------------

class Ship(object):

    def __init__(self):
        self._model = None
        self._position = None
        self._angle = 0.0
        self._input = None
        self._length_of_runway = 0.0
        self._is_runway_free = False
        self._velocity_vector = None
        self._goal = None
        self._aircraft_collection = None

    def init(self):
        assert not self._model
        self._model = framework.createShipModel()
        self._position = Vector2()
        self._angle = 0.0
        self._input = {
            framework.Keys.FORWARD: False,
            framework.Keys.BACKWARD: False,
            framework.Keys.LEFT: False,
            framework.Keys.RIGHT: False
        }
        self._length_of_runway = Params.Ship.LENGTH_OF_RUNWAY
        self._is_runway_free = True
        self._velocity_vector = Vector2()
        self._aircraft_collection = []
        self._goal = Vector2(Params.Ship.GOAL_INITIAL_POSITION_X,Params.Ship.GOAL_INITIAL_POSITION_Y)
        framework.placeGoalModel(Params.Ship.GOAL_INITIAL_POSITION_X, Params.Ship.GOAL_INITIAL_POSITION_Y)

    def deinit(self):
        for aircraft in self._aircraft_collection:
            if (aircraft.is_alive):
                aircraft.deinit()
        assert self._model
        framework.destroyModel(self._model)
        self._model = None

    def update(self, dt):
        current_linear_speed = 0.0
        current_angular_speed = 0.0

        if self._input[framework.Keys.FORWARD]:
            current_linear_speed = Params.Ship.LINEAR_SPEED
        elif self._input[framework.Keys.BACKWARD]:
            current_linear_speed = -Params.Ship.LINEAR_SPEED
        else:
            if self._velocity_vector.length() > 0.0:
                deceleration = self._velocity_vector.normalized() * Params.Ship.DECELERATION * -1
                self._velocity_vector = self._velocity_vector + deceleration * dt

        if current_linear_speed != 0.0:
            if self._input[framework.Keys.LEFT]:
                current_angular_speed = math.copysign(Params.Ship.ANGULAR_SPEED, current_linear_speed)
            elif self._input[framework.Keys.RIGHT]:
                current_angular_speed = -math.copysign(Params.Ship.ANGULAR_SPEED, current_linear_speed)

        self._angle = self._angle + current_angular_speed * dt

        #Logic of ship acceleration
        if (self._input[framework.Keys.FORWARD] or self._input[framework.Keys.BACKWARD]):
            acceleration = Vector2(math.cos(self._angle), math.sin(self._angle)) * math.copysign(
                Params.Ship.ACCELERATION, current_linear_speed)
            self._velocity_vector = self._velocity_vector + acceleration * dt
            if(self._velocity_vector.length() > abs(current_linear_speed)):
                self._velocity_vector = self._velocity_vector.normalized() * abs(current_linear_speed)

        if (self._velocity_vector.length() < 0.001 and self._velocity_vector.length() > 0):
            self._velocity_vector = Vector2()

        self._position = self._position + self._velocity_vector * dt
        framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

        self.fillingLandedAircraftsWithFuel(dt)
        self.updateAircrafts(dt)

    def keyPressed(self, key):
        self._input[key] = True

    def keyReleased(self, key):
        self._input[key] = False

    def mouseClicked(self, x, y, isLeftButton):
        if isLeftButton:
            self.changeGoalThatAircraftsFollow(x, y)
        else:
            for aircraft in self._aircraft_collection:
                #If the aircraft is not in the air and does not need to be refueled and the runway is free - send aircraft to take off(init it)
                if (aircraft.is_alive == False and aircraft.necessary_time_for_refuel == 0):
                    if (self._is_runway_free == True):
                        aircraft.init(self)
                        break

    def changeGoalThatAircraftsFollow(self, x, y):
        self._goal.x = x
        self._goal.y = y
        self.notifyAircraftObserversAboutNewGoal()
        framework.placeGoalModel(x, y)

    def addAircraft(self, aircraft):
        if (len(self._aircraft_collection) < Params.Ship.MAX_NUMBER_OF_AIRCRAFTS):
            self._aircraft_collection.append(aircraft)
        else:
            print("Failed, the number of aircraft allowed on this ship has been exceeded.")

    def removeAircraft(self, aircraft):
        if (len(self._aircraft_collection) > 0):
            self._aircraft_collection.remove(aircraft)

    def notifyAircraftObserversAboutNewGoal(self):
        for aircraft in self._aircraft_collection:
            aircraft.goal_to_follow.x = self._goal.x
            aircraft.goal_to_follow.y = self._goal.y

    def updateAircrafts(self, dt):
        for aircraft in self._aircraft_collection:
            if (aircraft.is_alive == True):
                aircraft.update(self, dt)

    def fillingLandedAircraftsWithFuel(self, dt):
        for aircraft in self._aircraft_collection:
            if (aircraft.necessary_time_for_refuel > 0):
                aircraft.necessary_time_for_refuel -= dt
                aircraft.necessary_time_for_refuel = max(aircraft.necessary_time_for_refuel, 0)

    @property
    def position(self):
        return self._position

    @property
    def angle(self):
        return self._angle

    @property
    def velocity_vector(self):
        return self._velocity_vector

    @property
    def is_runway_free(self):
        return self._is_runway_free

    @is_runway_free.setter
    def is_runway_free(self, value):
        self._is_runway_free = value

    @property
    def goal(self):
        return self._goal

    @goal.setter
    def goal(self, value):
        self._goal = value


# -------------------------------------------------------
#	Aircraft logic
# -------------------------------------------------------

class Aircraft(object):
    class State:
        TAKE_OFF = 0
        CIRCLING_AROUND_GOAL = 1
        PREPARING_FOR_LANDING = 2  # If aircraft ran out of fuel it starts to line up with the ship's runway (to prepare position for landing)
        LANDING = 3
        LANDED = 4

    def __init__(self, name):
        self._name = name
        self._model = None
        self._position = None
        self._angle = 0.0
        self._goal_to_follow = Vector2()
        self._landing_preparation_position_parameter = 0.0
        self._aircraft_aimed_alignment_point = 0
        self._distance_traveled_during_take_off = 0
        self._aircraft_state = self.State.TAKE_OFF
        self._necessary_time_for_refuel = 0.0
        self._flying_time = 0.0
        self._is_alive = False
        self._max_linear_speed = 0.0
        self._max_angular_speed = 0.0
        self._current_linear_speed = 0.0
        self._current_angular_speed = 0.0

    def init(self, ship):
        assert not self._model
        self._model = framework.createAircraftModel()
        self._position = Vector2(ship.position.x, ship.position.y)
        self._angle = ship.angle
        self._goal_to_follow = Vector2(ship.goal.x, ship.goal.y)
        self._landing_preparation_position_parameter = 0.0
        self._aircraft_aimed_alignment_point = 0
        self._distance_traveled_during_take_off = 0
        self._aircraft_state = self.State.TAKE_OFF
        self._necessary_time_for_refuel = 0.0
        self._flying_time = 0.0
        self._is_alive = True
        self._max_linear_speed = Params.Aircraft.LINEAR_SPEED
        self._max_angular_speed = Params.Aircraft.ANGULAR_SPEED
        self._current_linear_speed = 0.0
        self._current_angular_speed = 0.0

    def deinit(self):
        assert self._model
        framework.destroyModel(self._model)
        self._model = None
        self._is_alive = False
        self._necessary_time_for_refuel = Params.Aircraft.NECESSARY_TIME_FOR_REFUEL_AFTER_LANDING

    def update(self, ship, dt):
        # Checks if the aircraft has reached the maximum flight time - if yes prepare it for landing
        if (self._flying_time >= Params.Aircraft.MAX_FLYING_TIME and self._aircraft_state == self.State.CIRCLING_AROUND_GOAL):
            # Determines a random position for the start of landing (to simulate a real situation)
            self._landing_preparation_position_parameter = random.uniform(3.5, 5.5)

            print("....Flight time has expired (no fuel): " + str(self._name))
            self._aircraft_state = self.State.PREPARING_FOR_LANDING

        # 0 Aircraft take off (gradually accelerating)
        if (self._aircraft_state == self.State.TAKE_OFF):
            self.takeOff(ship, dt)

        # 1 Aircraft separated from the runway, began to fly towards the goal and circle around it.
        elif (self._aircraft_state == self.State.CIRCLING_AROUND_GOAL):
            self.circlingAroundGoal(ship, dt)

        # 2 Aircraft ran out of fuel and starting to line up with the ship's runway (to prepare position for landing)
        elif (self._aircraft_state == self.State.PREPARING_FOR_LANDING):
            self.preparingForLanding(ship, dt)

        # 3 The wheels of the aircraft have touched the ship's deck (runway) and the aircraft slows down until it stops
        elif (self._aircraft_state == self.State.LANDING):
            self.landing(ship, dt)

        self._flying_time += dt

        # If the aircraft has not landed (and disappeared from the map), update its position on the screen
        if (self._aircraft_state != self.State.LANDED):
            framework.placeModel(self._model, self._position.x, self._position.y, self._angle)

    def takeOff(self, ship, dt):
        ship.is_runway_free = False
        # This does not allow aircraft to fall off the runway (left or right) -> in case this should be allowed, comment this line
        self._angle = ship.angle
        self._current_linear_speed = self._current_linear_speed + Params.Aircraft.ACCELERATION * dt
        self._current_linear_speed = min(self._current_linear_speed, self._max_linear_speed)

        aircraftVelocityVector = Vector2(math.cos(self._angle), math.sin(self._angle)) * self._current_linear_speed
        # Aircraft moves independently at its own speed as one system + moves along with the ship as another system
        self._position = self._position + ship.velocity_vector * dt
        self._position = self._position + aircraftVelocityVector * dt

        self._distance_traveled_during_take_off += aircraftVelocityVector.length() * dt

        # This means that aircraft reached the end of the deck
        if (self._distance_traveled_during_take_off >= ship._length_of_runway):
            self._aircraft_state = self.State.CIRCLING_AROUND_GOAL
            ship.is_runway_free = True
            print("++++The aircraft took off: " + str(self._name))


    def circlingAroundGoal(self, ship, dt):
        # Continues to accelerate to maximum speed
        self._current_linear_speed = self._current_linear_speed + Params.Aircraft.ACCELERATION * dt
        self._current_linear_speed = min(self._current_linear_speed, self._max_linear_speed)

        aircraftToGoalVector = self._goal_to_follow - self._position
        aircraftUnitVector = Vector2(math.cos(self._angle), math.sin(self._angle))

        angle_between_vectors = math.atan2(aircraftToGoalVector.y, aircraftToGoalVector.x) - math.atan2(aircraftUnitVector.y, aircraftUnitVector.x)

        # Normalizing the angle to be between -pi and pi
        if angle_between_vectors > math.pi:
            angle_between_vectors -= 2 * math.pi
        elif angle_between_vectors < -math.pi:
            angle_between_vectors += 2 * math.pi

        # The angular speed was reduced (*0.8) so that the aircraft would circle with a larger diameter around the goal
        self._current_angular_speed = self._max_angular_speed * 0.8
        self._angle += angle_between_vectors * self._current_angular_speed * dt



        # When aircraft is flying in a circular path, its speed will decrease due to the centripetal force acting on the plane
        if (abs(angle_between_vectors) > math.pi / 6):
            self._current_linear_speed -= Params.Aircraft.DECELERATION * dt * 3
            self._current_linear_speed = max(self._current_linear_speed, self._max_linear_speed * 0.7)
        else:
            self._current_linear_speed += Params.Aircraft.ACCELERATION * dt * 3
            self._current_linear_speed = min(self._current_linear_speed, self._max_linear_speed)

        aircraftVelocityVector = Vector2(math.cos(self._angle), math.sin(self._angle)) * self._current_linear_speed
        self._position = self._position + aircraftVelocityVector * dt

    #
    def preparingForLanding(self, ship, dt):
        self._current_angular_speed = self._max_angular_speed

        futureShipPosition = ship.position + ship.velocity_vector * dt * 25
        shipUnitVector = Vector2(math.cos(ship.angle), math.sin(ship.angle))

        # There are 3 alignment points through which the aircraft should pass during its alignment with the runway,
        # if it successfully aims through all three, it will be properly aligned with the runway and landing will be possible
        aircraftToAlignmentPointVector = (futureShipPosition - shipUnitVector * self._landing_preparation_position_parameter) - self._position

        if (self._aircraft_aimed_alignment_point == 0):
            if (aircraftToAlignmentPointVector.length() < 0.2):
                # Aircraft aimed at the first point
                self._aircraft_aimed_alignment_point = 1

        elif (self._aircraft_aimed_alignment_point == 1):
            aircraftToAlignmentPointVector = (futureShipPosition - shipUnitVector * 2) - self._position
            if (aircraftToAlignmentPointVector.length() < 0.2):
                # Aircraft aimed at the second point
                self._aircraft_aimed_alignment_point = 2

        elif (self._aircraft_aimed_alignment_point == 2):
            aircraftToAlignmentPointVector = (futureShipPosition - shipUnitVector) - self._position
            if (aircraftToAlignmentPointVector.length() < 0.2):
                # Aircraft aimed at the third point
                self._aircraft_aimed_alignment_point = 3

        elif (self._aircraft_aimed_alignment_point == 3):
            aircraftToAlignmentPointVector = (futureShipPosition - self._position)
            if (aircraftToAlignmentPointVector.length() < 0.2):
                # Aircraft touched the deck(runway) of the ship with the wheels -> landing has started
                self._aircraft_state = self.State.LANDING

        aircraftUnitVector = Vector2(math.cos(self._angle), math.sin(self._angle))
        aircraftToShipAngle = math.atan2(aircraftToAlignmentPointVector.y,aircraftToAlignmentPointVector.x) - math.atan2(aircraftUnitVector.y, aircraftUnitVector.x)

        if aircraftToShipAngle > math.pi:
            aircraftToShipAngle -= 2 * math.pi
        elif aircraftToShipAngle < -math.pi:
            aircraftToShipAngle += 2 * math.pi

        self._angle += aircraftToShipAngle * self._current_angular_speed * dt

        # When aircraft is flying in a circular path, its speed will decrease due to the centripetal force acting on the plane
        if (abs(aircraftToShipAngle) > math.pi / 6):
            if (self._aircraft_aimed_alignment_point != 0):
                self._current_linear_speed -= Params.Aircraft.DECELERATION * dt * 3
                self._current_linear_speed = max(self._current_linear_speed, self._max_linear_speed/2 * 0.7)
        else:
            if (self._aircraft_aimed_alignment_point != 0):
                self._current_linear_speed += Params.Aircraft.ACCELERATION * dt * 3
                self._current_linear_speed = min(self._current_linear_speed, self._max_linear_speed/2)

        aircraftVelocityVector = Vector2(math.cos(self._angle), math.sin(self._angle)) * self._current_linear_speed
        self._position = self._position + aircraftVelocityVector * dt


    def landing(self, ship, dt):
        self._current_linear_speed -= Params.Aircraft.DECELERATION * dt
        self._current_linear_speed = max(self._current_linear_speed, 0)

        # This does not allow aircraft to fall off the runway (left or right) -> in case this should be allowed, comment this line
        self._angle = ship.angle

        aircraftVelocityVector = Vector2(math.cos(self._angle), math.sin(self._angle)) * self._current_linear_speed

        # Aircraft moves independently at its own speed as one system + moves along with the ship as another system
        self._position = self._position + ship.velocity_vector * dt
        self._position = self._position + aircraftVelocityVector * dt

        if (self._current_linear_speed < 0.1):
            print("----The aircraft landed: " + str(self._name))
            self._aircraft_state = self.State.LANDED
            self.deinit()


    @property
    def goal_to_follow(self):
        return self._goal_to_follow

    @goal_to_follow.setter
    def goal_to_follow(self, value):
        self._goal_to_follow = value

    @property
    def necessary_time_for_refuel(self):
        return self._necessary_time_for_refuel

    @necessary_time_for_refuel.setter
    def necessary_time_for_refuel(self, value):
        self._necessary_time_for_refuel = value

    @property
    def is_alive(self):
        return self._is_alive

    @is_alive.setter
    def is_alive(self, value):
        self._is_alive = value


# -------------------------------------------------------
#	game public interface
# -------------------------------------------------------

class Game(object):
    def __init__(self):
        self._ship = Ship()

    def init(self):
        print(">>>>>>>  New Game  <<<<<<<")
        self._ship.init()
        self._ship.addAircraft(Aircraft("F-1"))
        self._ship.addAircraft(Aircraft("F-2"))
        self._ship.addAircraft(Aircraft("F-3"))
        self._ship.addAircraft(Aircraft("F-4"))
        self._ship.addAircraft(Aircraft("F-5"))
        # self._ship.addAircraft(Aircraft("F-6")) # Should throw error message and not allow adding more than 5 aircrafts

    def deinit(self):
        self._ship.deinit()

    def update(self, dt):
        self._ship.update(dt)

    def keyPressed(self, key):
        self._ship.keyPressed(key)

    def keyReleased(self, key):
        self._ship.keyReleased(key)

    def mouseClicked(self, x, y, isLeftButton):
        self._ship.mouseClicked(x, y, isLeftButton)


# -------------------------------------------------------
#	Run game
# -------------------------------------------------------

if __name__ == '__main__':
    framework.runGame(Game())
