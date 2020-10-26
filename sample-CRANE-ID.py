import IDpy as Ar
import math
cr = Ar.Negi39FIKFID()
#cr.setjointnum(7)
#cr.setdhparameter(0,[0.0,0.0,0.064,-0.5*math.pi])
jointangle = [-0.25*math.pi]*7 #angle rad
print jointangle
jointtau = [1.0]*7 #angle tau N*m
#calc foward kinematics
posquo = cr.getpos(jointangle)
print "[Foward kinematics] position and quatanion are"
print posquo
print ""
#calc inverse kinematics
posquo[0] -= 0.01
posquo[1] -= 0.01
hogeangle = cr.getangle(posquo,jointangle)
print "[Inverce kinematics] angle is"
print hogeangle
print ""
#calc foward kinematics
posquo = cr.getpos(hogeangle)#check ik 
print "[Foward kinematics] position and quatanion are"
print posquo
print ""
#calc inverse dynamic
forcemoment = cr.getforce(jointangle,jointtau)
print ""
print "[Inverce dynamics] forcemoment is"
print forcemoment
print ""
# calc forward dynamic
tau = cr.gettau(jointangle,forcemoment)
print "[Foward dynamics] tau is "
print tau
print ""