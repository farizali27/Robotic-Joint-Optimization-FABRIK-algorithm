import numpy as np

#creating class Link whose attributes are length and joint angle
class Link:
    def __init__(self,length,joint_angle):
        self.length=length
        self.joint_angle=joint_angle


class RoboticArm:
    def __init__(self,links,target):
       self.links=links
       self.target=target
       self.lengths=[self.links[0].length,self.links[1].length,self.links[2].length]

    def get_points(self,angles):
        length=self.lengths
        #rotation matrix 1
        r1 = np.array([[np.cos(angles[0]), -np.sin(angles[0])],
                       [np.sin(angles[0]), np.cos(angles[0])]])
        #rotation matrix 2
        r2 = np.array([[np.cos(angles[0]+angles[1]), -np.sin(angles[0]+angles[1])],
                       [np.sin(angles[0]+angles[1]), np.cos(angles[0]+angles[1])]])
        #rotation matrix 3
        r3 = np.array([[np.cos(angles[0]+angles[1]+angles[2]), -np.sin(angles[0]+angles[1]+angles[2])],
                       [np.sin(angles[0]+angles[1]+angles[2]), np.cos(angles[0]+angles[1]+angles[2])]])
        p0=[0.0,0.0]
        #applying forward kinematics
        p1=np.dot(r1,[length[0],0.0])
        p2=np.array(p1)+np.array(np.dot(r2,[length[1],0.0]))
        p3=np.array(p2)+np.array(np.dot(r3,[length[2],0.0]))
        return [p0,p1,p2,p3]
    
    def get_angles(self,points):
        angles = [0,0,0]
        length=self.lengths
        i=0
        v1=[1.0,0.0]
        while(i<3):
            v2=np.array(points[i+1])-np.array(points[i])
            m1=np.linalg.norm(v1)
            m2=np.linalg.norm(v2)
            if np.cross(v1,v2) > 0:
                angles[i]=np.arccos(np.dot(v1,v2)/(m1*m2))
            else:
                angles[i]=-1*np.arccos(np.dot(v1,v2)/(m1*m2))
            v1=v2
            i=i+1
        return angles


    def check_reachability(self):
        #if the distance between target and base joint(origin) is less than sum of link lengths then return true
        if(np.linalg.norm(self.target)<=(23+15+2)) :
            return True
        else:
            return False

    def forward_pass(self,points):
        length=self.lengths
        i=3
        points[i]=self.target
        while(i>0):
            diff=np.array(points[i])-np.array(points[i-1])
            unit_vector=diff/np.linalg.norm(diff)
            points[i-1]=np.array(points[i])-np.array(length[i-1]*unit_vector)
            i=i-1
        return points

    def backward_pass(self,points):
        length=self.lengths
        i=0
        points[0]=[0.0,0.0]
        while(i<3):
            diff=np.array(points[i+1])-np.array(points[i])
            unit_vector=diff/np.linalg.norm(diff)
            points[i+1]=np.array(points[i])+np.array(length[i]*unit_vector)
            i=i+1
        return points


    def fabrik(self,points,tolerance,count):
        target=self.target
        #distance between end effector and target
        dist=np.linalg.norm(np.array(points[3])-np.array(target))

        #termination condition:if point 3(end effector) is close enough to the target then end iterations
        if dist<=tolerance :
            return
        
        #implementing forward pass:
        points=self.forward_pass(points)
        
        #implementing backward pass:
        points=self.backward_pass(points)

        #getting joint angles by passing points as argument
        joint_angles=self.get_angles(points)
        print("The joint angles after iteration no-{} are: {}, {}, {}".format(int(count), 180/np.pi*float(joint_angles[0]), 180/np.pi*float(joint_angles[1]), 180/np.pi*float(joint_angles[2])))
            
        #recursively calling the fabrik function
        self.fabrik(points,tolerance,count+1)

    
#main function
def main():
    #taking joint angles and target position as input
    theta1 = np.pi/180*float(input("Enter joint angle of first  link IN DEGREE (w.r.t the fixed coordinate system)\n"))
    theta2 = np.pi/180*float(input("Enter joint angle of second link IN DEGREE (w.r.t link1)\n"))
    theta3 = np.pi/180*float(input("Enter joint angle of third  link IN DEGREE (w.r.t link2)\n"))
    angles = [theta1,theta2,theta3]

    target = [float(x) for x in input("Enter coordinates of target position: \n").split()]
    #removing the z coordinate of target as it will always be zero
    target=[target[0],target[1]]

    #creating link objects
    link1 = Link(23,theta1)
    link2 = Link(15, theta2)
    link3 = Link(2, theta3)        #last digit of roll number=2

    # Passing link objects to object of RoboticArm class
    arm = RoboticArm([link1, link2, link3],target)

    #getting initial coordinates of joints by passing initial angles as argument
    points=arm.get_points(angles)

    #checking reachability of the target position
    reachibility=arm.check_reachability()

    if reachibility:
        print("yes")

        #implementing fabrik algorithm
        tolerance=0.1
        arm.fabrik(points,tolerance,1)
        
    else:
        print("no")


#calling the main function
main()