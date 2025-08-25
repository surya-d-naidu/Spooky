from servo import set_servo_angle as s
"""
intp=[input(" : ").split(" ")]

"""


temp = 0
while True:
    
    inp = input("enter servo number :")
    if inp == "": break
    elif inp == "all":
        for i in range(16):
            s(i,50)
        continue
    
    num= int (inp)
    angle = input("angle : ")
    
    if angle == "": 
        angle = temp
        print(f"angle is {temp}")
    temp = angle

    s(num,angle)
    
