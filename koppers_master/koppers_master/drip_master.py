# 1. Select the portion to clean - 4 portions: 0, 1, 2, 3 from L(away side) to R(drain side)
# 2. Get the current section location on the pan, each section is a set length, with each step defined as a constant, tracked by odometry
# 3. Check if the section is cleanable, if it is, start cleaning # TODO: Integrate cleanable/not-cleanable map
# 4. Based on the portion selected and current section type, call cleaning skill
# 5. Move to the next section, if reached end of pan, switch to next portion and repeat from step 2 in opposite direction
# 6. End if portion 3 and end of pan is reached

# master class for the robot
class DripBot:

    def __init__(self):

        # launch the UI, sensors: voltage check, ToFs, Arm, camera

        self.location = 0 #m along the pan
        self.current_section = 0 # 0, 1, 2, 3 ...
        self.current_portion = 0 # 0, 1, 2, 3
        self.direction = 1 # 1 for forward, -1 for backward
        self.end_of_pan = False # True if end of pan is reached
        self.current_state = "idle" # idle, operating [cleaning, localization, moving]
        self.movement_step = 0.1 #m
        self.section_length = 0.5 #m
        self.start_signal = False # if received, start the robot

        self.cleaner = None # TODO: Implement cleaning skill node
        self.base_comms = None # TODO: Implement base communications node
        self.localizer = None # TODO: Implement localization skill node
        self.mover = None # TODO: Implement base linear movement node

    # Launch the robot
    def launch(self):
        # Run startup procedures - initialize sensors, actuators, etc., health check
        
        # Receive start signal
        while True:
            if self.start_signal: # TODO: Implement start signal subscription
                self.current_state = "operating"
                break

    # Run the cleaning operation
    def run(self):
        
        self.current_portion = 0
        self.direction = 0
        self.end_of_pan = False

        while (self.current_portion != 3) and (not self.end_of_pan):            

            # Call the cleaning skill node
            self.cleaner(self.current_section, self.current_portion)

            # Move to the next section
            self.mover(self.direction, self.movement_step)

            # Get the location of the robot
            self.location, self.end_of_pan = self.localizer()

            # Check if end of pan is reached
            if self.end_of_pan:
                self.current_portion += 1
                self.direction *= -1
                self.end_of_pan = False

if __name__ == "__main__":
    drip_cleaner = DripBot()
    drip_cleaner.launch()