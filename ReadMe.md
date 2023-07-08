# Ship-Aircrafts Game

![](https://github.com/Konosubasaki/shipAndAircraftsGame/blob/master/AS.gif)

#### Right click starts the take off of the plane.
#### Left click changes the position of the target.
#### Arrows are used to move the ship.

###### - The next aircraft cannot take off while the runway on the ship is busy (another aircraft is taking off).  
###### - Aircraft is considered to have taken off only when it has already passed a certain deck length during takeoff (given parameter), more precisely, when the aircraft reaches the end of the deck, it "separates" from the ship and takes off.  
###### - As long as the aircraft is in contact with the ship, that is, the wheels are on the deck (in this case, it is the time of takeoff or landing), they are considered one system. Thus, the aircraft moves independently at its own speed as one system + moves along with the ship as another system.  
###### - When aircraft is flying in a circular path, its speed will decrease due to the centripetal force acting on the plane. To simulate that in this case, if the turn angle is greater than pi/6, the aircraft moves slower (the speed is multiplied by 0.7).  
###### - Landing of the aircraft is carried out on the opposite side from the take-off point, also slowly decelerating along the length of the deck.  
###### - Before landing the aircraft (as in real life), the nose of the aircraft is aligned with the runway / deck - to ensure landing.  
###### - Data on takeoff, landing and flight time expiration ("lack of fuel") of a certain aircraft are displayed on the console.  
###### - Acceleration and deceleration of the ship at the beginning of the movement and after a stop ("Smooth movement").  
