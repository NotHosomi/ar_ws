import os

with open('neg.txt','w') as f:
    
    for filename in os.listdir('/home/aaron/catkin_ws/src/opencv_interface/src/Person_Images/negative'):
        f.write('negative/' + filename + '\n')
        