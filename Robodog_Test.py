import time
from robot_dog import RobotDog  # import your class

def first_test():
    # Initialise robot
    dog = RobotDog()
    print("🐕 Robot Dog Booting Up...")

    # Step 1: Move forward
    print("➡️ Moving forward...")
    dog.forward()
    time.sleep(2)
    dog.stop()

    # Step 2: Move backward
    print("⬅️ Moving backward...")
    dog.backward()
    time.sleep(2)
    dog.stop()

    # Step 3: Turn left
    print("↪️ Turning left...")
    dog.turn_left()
    time.sleep(1.5)
    dog.stop()

    # Step 4: Turn right
    print("↩️ Turning right...")
    dog.turn_right()
    time.sleep(1.5)
    dog.stop()

    # Step 5: Do a victory spin (full 360° turn)
    print("🎉 Victory spin!")
    dog.turn_left()
    time.sleep(3)  # adjust timing to get one full spin
    dog.stop()

    print("✅ Test routine complete! Dog is ready.")

if __name__ == "__main__":
    first_test()
