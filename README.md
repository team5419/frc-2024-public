# Welcome to the repo for Radium the Roomba 

<p align="center">
  <img src=./images/DSC_5030.JPG width="100%" title="Radium waiting for a match to start">
</p>

---

## Code Highlights

- State machine based high-level control
- Fully automated, full speed velocity-compensated auto shoot
- Field localization using swerve odometry fused with pose from 1 camera running PhotonVision
- Automated note pickup and handoff sequence
- Driver can queue one command in advance to minimize transition time between states


## State Machines

### Superstructure
The superstructure controls the overall state of the robot, each of which contains different actions for the subsystems (eg. intaking, climbing, shooting, etc). This system takes input from the controller buttons to switch between states depending on whether they meet the requirements to do so. There are also several fail-safe states so that in the event something goes wrong, like a note gets stuck, the robot is able to recover and return back to another state.

<p align="center">
<img src=./images/superstructure.png width="650">
</p>

**States**:
- IDLE
- MOVING_TO_POS - this state handles all pre-defined arm & elevator positions, including the amp and the trap
- INTAKING
- SHOOTING
- EJECT_AND_RESET
- REACHING_FOR_CHAIN - the first step in the climbing sequence
- CLIMBING - retracts the elevator to lift the robot off the ground
- REACHING_FOR_TRAP - after the hooks handoff, the robot raises the elevator again to be able to reach the trap
- TRAP_SHOOTING  - runs the indexer backwards to shoot the note into the trap while the robot is positioned on the chain
- TRAP_RETRACTING - retracts the arm and elevator after trap shot is finished
- STOW_RESET
- INTERPOLATING - automatically tracks the speaker while driving and shooting

**Subsystems**:
- Shooter (2x falcons)
- Arm & Elevator State Machine
- Intake State Machine

### Arm & Elevator (Kinematic Chain)
This state machine controls how the arm and elevator move together. Different positions for the arm and elevator to move to are predefined in the class SuperstructureGoal for easy use. Our robot geometry requires us to move the elevator and the arm in ways to avoid crashing into each other, which is why we have specific states to home the elevator and then the arm. We also have a state for stowing the arm and elevator, which zeros the position by applying power until it hardstops.

<p align="center">
<img src=./images/arm-ele.png width="350">
</p>

**States**:
- IDLE
- MOVING
- AT_POS
- HOMING_ELE
- HOMING_ARM

**Subsystems**:
- Elevator (2x krakens) - mounted on the robot chassis
- Arm (kraken) - the arm is the pivot connected to the elevator that the shooter is mounted on


### Intake & Handoff
The robot has a series of 3 beam break sensors to detect the location of a note on the robot: in the intake, at the entry to the indexer, and at the end of the indexer before the shooter. The intake sequence state machine uses these beam breaks to determine what state it is in. It also handles the speeds at which notes are passed through so that the note always gets prepped in the same location, as well as the shooting and trap shot sequence. It has two fail safes built in, eject and reset (shoots the note instantly), and manual intake.

<p align="center">
<img src=./images/intake.png width="600">
</p>

**States**:
- IDLE
- INTAKING
- PASSING_TO_INDEXER
- NOTE_ENTERED_INDEXER
- ALIGNING_IN_INDEXER - once in this state, the note is no longer in the intake and the arm can be positioned for shooting
- NOTE_READY
- SHOOTING
- TRAP_SHOT - note comes out of the back of the indexer
- EJECT_AND_RESET - all intake rollers move at a low speed to remove the note and reset the state machine
- MANUAL_INTAKE

**Subsystems**:
- Sweepers (2x falcons) - vertically spinning star wheels that allow the robot to intake from all 4 sides
- Intake (falcon) - horizontal rollers that feed into the handoff
- Indexer (kraken) - polybelt rollers that feed the shooter


### Swerve
We chose to use CTRE's swerve API this year. To control it, we used a 1678 style state machine, barring a few commands like auto align. We use the CTRE odometry fused with vision updates from an Oranges/ Pi 5 running PhotonVision for localization. We chose to package our vision in the swerve file, as it's the only place we use our vision readouts. 

**States**
- IDLE
- LOCK - move the modules into a "x" position
- OPEN_LOOP - normal field relative driving
- PATH_FOLLOWING - state for auto
- FORCE_ORIENT - a "snap" state that takes over rotational control but keeps translation

**Hardware**
- 4x Krakens for drive
- 4x Falcons for turn
- 4x CanCoders
- Canivore
- Orange Pi 5 (5+ works too)
- OV9281 Global Shutter USB Camera
- Phoenix Pro


---

## Shoot on the move

### Quick overview
Shoot on the move can be a pretty complex function, so we wanted to ensure that it was easy to work with. We eventually decided to consolidate all the shoots on the move to one button. With the button press, the robot handles all of the auto aim, and the driver can press the shoot button. When the robot deems the aim to be within tolerance, it auto shoots and auto stows. All of our setpoints, for both swerve and superstructure, take into account our current velocity, our shot distance, and time to the target to produce a setpoint that we can feasibly track and ensure that the note gets into the speaker.

### How we did it
We continuously poll our current chassis velocity, looking a set time into the future, to calculate where we are going to be. From there, we can calculate our superstructure setpoint as if we were in the future and pass it on to our subsystems. For our chassis heading, we came up with a function that takes in our velocity and the time of flight for the note and returns a new field-relative yaw that will ensure the note still hits the speaker. To achieve a time of flight function, we characterized the shot time of our shooter based on distance. Both the superstructure setpoint generator's and swerve heading generator's look-ahead values are tuned to make sure that we don't overcompensate or look too far ahead.


