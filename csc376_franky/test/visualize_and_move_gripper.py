
import roboticstoolbox as rtb
from simulation import RtbVisualizer
import csc376_franky


def main():
    panda_rtb_model = rtb.models.Panda()
    
    csc376_gripper = csc376_franky.Gripper("192.168.1.107")
    q_start =  panda_rtb_model.qr # Random q start for visualizer
    visualizer = RtbVisualizer(panda_rtb_model, q_start)

    input("Press enter, to move gripper in visualizer\n")
    visualizer.move_gripper(0.01, 0.1)
    visualizer.move_gripper(0.07, 0.1)

    yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
    if yes_or_else != "yes":
        print("User did not type [yes], will not run on real robot")
        return visualizer
    csc376_gripper.move(0.01, 0.1)
    csc376_gripper.move(0.07, 0.1)

    return visualizer

if __name__ == "__main__":
    visualizer = main()
    visualizer.stop() # Makes sure render thread ends
