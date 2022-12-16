# Shellby the Shell Shuffler

## 1.  Introduction

    The goal of our project was to build an interactive shell game with the Sawyer robot. The user can decide to play against the Sawyer as the shuffler or guesser, or have 2 Sawyers play against each other. Our ultimate goal was to have the shuffler and guesser win while playing against a human. 

    For our implementation of the shell game, there must be a shuffler and guesser. The shuffler will shuffle the cups while maintaining the same original configuration/placement on the table. It is important that the bean is not seen and doesn’t slide out while shuffling. The guesser will detect when the shuffler has started and stopped shuffling with vision, keeping track of which cup the bean is under, and lifting the cup corresponding to its final guess. The problems to be solved include incorporating computer vision sensing (image processing) to detect the cup for both the guesser and the shuffler, planning through the shuffler’s decision on which cups to shuffle, and actuating from shuffling the cups to revealing the final cup.    

    This project interests us because we’re creating both sides of the game. It will also be interesting to see which "player" is more successful and to see how the players fare against a human player. 

    Overall, Shellby performed well in the task of playing the shell game, demonstrating its capabilities in computer vision and object manipulation. This technology has potential applications in a wide range of fields, from manufacturing and assembly to entertainment and gaming. In a shipping and handling situation, for example, the robot could move packages from a storage facility onto a truck, sensing where to stack within the constraints of the truck container.

## 2.  Design

    Our project can be divided into two independent parts: the shuffler and the guesser.

The design criteria we considered for each subsystem are the following:

- Guesser
    - Detect cups to be moved and ignore everything else
    - Track the position of the cup containing the bean
    - Output the final bean position given the signal that the game has ended
- Shuffler
    - Obtain the relative positions of the base of the robot, camera, and the three cups
    - Move the end effector to a position over any desired cup
    - Slide the cup on the ground in any direction with the end effector such that the rim of the cup maintains contact with the ground
    - Randomly decide which cups to shuffle and execute

    For the guesser, we decided to use computer vision using a commercial webcam. The webcam would be imaging the field of view containing the shuffling cups and the computer vision program would take that image and separate and identify the individual cups. We were able to go with this design due to the cups’ distinct color and outline from the rest of the background. However, this design choice does not directly identify the location of this cup in 3D space and heavily relies on the background and lighting, making it susceptible to poor lighting and interference from the cups being very close to each other. Due to the webcam’s ability to be deployed anywhere, simplicity, and wide availability, it is efficient and durable, with robustness heavily depending on the image processing software.

    The shuffler uses pre-programmed movements in order to manipulate the cups based on the logic code. This allows for good consistency but is not a flexible implementation as it requires the cups to start in the same position whenever the game is played. We use a gripper to physically manipulate the cups’ positions, grabbing the cup and dragging it to their destinations. Because of the gripper’s small size and simplicity, it is not only able to grab cups reliably but also be able to work with other objects as well.  Also due to it only having two contact points, it allows the cup to have degrees of freedom to naturally make contact with the work surface when the gripper drags it along.

## 3.  Implementation

• Describe any hardware you used or built. Illustrate with pictures and diagrams.
• What parts did you use to build your solution?
• Describe any software you wrote in detail. Illustrate with diagrams, flow charts, and/or other
appropriate visuals. This includes launch files, URDFs, etc.
• How does your complete system work? Describe each step.

    The Sawyer robot arm used for this project is a state-of-the-art high-precision robotic manipulator with 7 joints and a gripper for picking up and moving objects. It was programmed with a controller for movement, an external camera with proprietary computer vision algorithms to track the cups, and ROS to wirelessly communicate with our computer. Also used was RViz to visualize our pathing before execution for safety.

    The cameras used are the Logitech C922 webcams providing 1080p video feed at 30 fps. Plugged directly into the computer handling the software, the camera responsible for the guesser was mounted directly above the three cups and pointed downward. The shuffler camera was mounted level with the cups and pointed such that the three cups and the robot base were contained in the field of view, for reasons we will clarify later.

    All programming was done by our team with the ROS package in Python and the RViz software. RViz is a ROS graphical interface for viewing robot movements, AR tag positioning, and much more.

    Here is a screenshot of RViz with the robot, AR tag markers, and obstacles shown:

**[INSERT RVIZ SCREENSHOT WITH ROBOT AND AR TAG MARKERS SHOWN]**

Below are logic diagrams for both the shuffler and guesser:

![Untitled](Shellby%20the%20Shell%20Shuffler%20074f832cbb2a4328a283dbf2438d9ee6/Untitled.png)

![Untitled](Shellby%20the%20Shell%20Shuffler%20074f832cbb2a4328a283dbf2438d9ee6/Untitled%201.png)

**Guesser:**

    The Guesser begins with the raw image received from the camera viewing the cups to be processed. 

    We then downsize the image from 1080p to 360p in order to allow the image processing to run quicker, then employ gaussian blur across the image reducing the details and effect from noise and other small details and defects present in the cups and environment.

    Since we are using red solo cups, we distinguish them by masking out all non-red features. We accomplish this by passing only parts of the image that lie between specific RGB values that we determined through testing into the next step. In the end we have a mask image with white where the cups are present and black for everything else.

    This mask is passed into findContour function in the OpenCV framework, which is an open-source library that includes several hundreds of computer vision algorithms. The contour outlines the blob of pixels that correspond to each cup and distinguishes the three cups from each other as well.

    We ultimately obtain a single location of the cups by finding the centroid of each of contour given by the aforementioned findContour function and finding the moment and center of mass (a.k.a averaging out the x and y coordinates of the pixels). We also use filtering at this step due to the contour function returning multiple contours around a single object when we only need one. We accomplish this by defining an error region around each centroid and merging centroid that lie within each other’s region into one. This results in a single centroid identifying each of the three cups and a hand if is present in the field of view.

****[insert screenshot of centroids and mask image]****

    With the centroids found, we continuously loop through the camera images to follow the centroids as they move. By tracking their base position and using logic of which positions are moving, we can distinguish which cups are being shuffled. The logic compares the centroids from the previous iteration, first linking the centroids that have not moved to each other as a single object. Then the moving cup is identified by looking for the centroid closest to its previous position and updating the cup position with the correct centroid and not per se the hand centroid or a centroid resulting from noise. As the shuffler shuffles through the cups, the algorithm will follow the original cup with the bean inside to the end, and make its final guess once shuffling ends signaled by a keyboard interrupt.

******************Shuffler:******************    

    The shuffler also begins with the raw image from the camera. Using the ar_tracker package and the AR tags being positioned on top of the cups, we can get the TF transformation between the robot gripper and the cups. This is done through combining the cup TF frames with the base frame (from the robot). [extrapolate]

    The controller uses PID or proportionate-integral-derivative control to travel between its current position and its desired trajectory. The proportionate term describes the actual error between the current and desired position. The integral term accounts for constant system errors and inconsistencies within the robot itself. The derivative terms follow the rate of change of the gripper's position to accelerate smoothly and slowly at the beginning and end of the robot’s movement. This is to prevent jerky behavior that would otherwise cause system instabilities.

[INSERT MORE ABOUT TF / LAUNCH FILES / CONTROLLER / MOVEIT ETC]

    The actual motion of shuffling is simply moving the 1st cup to an intermediary position, moving the 2nd cup to the 1st cup’s original position, and then finishing with the 1st cup in the 2nd cup’s original position.

The file structure of the project is visualized below and can also be seen on the GitHub repository.

**[INSERT FILE STRUCTURE FOR GIT REPO / SHELLBY]**

## 4.  Results

• How well did your project work? What tasks did it perform?
• Illustrate with pictures and at least one video.

    Shellby was successfully able to move and shuffle the cups on the table as the shuffler. Shellby was also able to reliably track the cup given the information on which cup contains the bean after multiple shuffles.

    Overall, the robot arm performed well in the task of playing the shell game, demonstrating its capabilities in computer vision and object manipulation.

## 5.  Conclusion

• Discuss your results. How well did your finished solution meet your design criteria?
• Did you encounter any particular difficulties?
• Does your solution have any flaws or hacks? What improvements would you make if you had
additional time?

    Shellby’s guesser aspect was able to meet the design criteria as we specified, being able to detect, track, and output the position of the cup containing the bean. We did have difficulty with the filtering part of the image processing as we had to use primarily trial and error in order to get a RGB value interval that ignored anything other than cups, and that interval changed with lighting and camera. We did consider a more complex solution that filtered pixels that had a higher G and B value compared to the R value, but the solution was computationally taxing and slowed down the image processing to the point where it interfered with the tracking ability.

    Shellby’s shuffler aspect ultimately wasn’t able to meet the first design criteria of identifying the relative positions of the cup, camera, and robot base. We initially attempted a vision based solution using AR tracking with AR tags located on the cups and base of the robot, but the inaccuracies and noise involved with identifying AR tags too small at too far of a distance led to significant reliability issues that impacted our ability to demonstrate a valid shuffler. Because of this we converted to a hard-coded solution where the robot is given the cup positions and plans its motion accordingly. This unfortunately leads to our solution failing whenever the cups are misaligned from their designated position. We also struggled significantly with motion planning, position and orientation constraints required by the design criteria often not working as intended and often planned extremely inefficient paths.

    Improvements we would have liked to have made include a different vision based solution where the camera based on the robot’s end effector is used as it eliminates the error associated with extrapolating the transformation between the external camera and the base of the robot (since the robot’s end effector position can be accurately derived from the joint position) with AR tags. We also considered motion planning each step of the shuffling multiple times and select the best path using criteria such as total end effector travel distance or joint angle change.

**Guesser:**

- Works reliably and is immediately deployable.
- Due to the nature of our vision processing, occasionally looses the cups when the cups come in contact and form one blob of red pixels.
    
    

**Shuffler:**

- Large inaccuracies from camera detection -> Cup TF frame
    - Better camera angle/AR tag sizing for distance
    - On-arm camera
- Hardcode the sliding motions so we don’t have to rely on motion planning
    - Position constraint
- Multiple planned motions and selecting the most “reasonable” one
    - Shortest distance traveled

## 6.  Team

• Include names and short bios of each member of your project group.
• Describe the major contributions of each team member.

Nisha Prabhakar is an EECS major interested in machine learning and computer vision. She worked on the Guesser side.

Kevin Oh is a mechanical engineering major with prior robotics experience. He primarily worked on the Guesser side, responsible for the image filtering and the cup tracking logic.

Matthew Sahim is a nuclear engineering major with research and technical project experience. He worked on the Shuffler side, primarily on the controller and movement logic.

Reena Yuan is an EECS major with software engineering experience from a variety of companies. She worked on the shuffler side, primarily on tracking the poses between gripper and the AR tags on the cups.

## 7.  Additional materials

• code, URDFs, and launch files you wrote
• CAD models for any hardware you designed
• data sheets for components used in your system
• any additional videos, images, or data from your finished solution
• links to other public sites (e.g., GitHub), if that is where your files are stored

The GitHub repository below is where all the code is stored.
