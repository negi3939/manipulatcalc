import IDpy as Ar
import math
cr = Ar.Negi39FIKFID()
#cr.setjointnum(7)
#cr.setdhparameter(0,[0.0,0.0,0.064,-0.5*math.pi])
jointangle = [0.1,0.2,0.3,0.4,0.5,0.6,0.7] #angle rad
jointtau = [1.1,1.2,1.3,1.4,1.5,1.6,1.7] #angle tau N*m
#calc inverse kinematics
posquo = [0.1,0.4,0.2,1.0,0.0,0.0,0.0]
hogeangle = cr.getangle(posquo)
print "[Inverce kinematics] angle is"
print hogeangle
#calc foward kinematics
posquo = cr.getpos(hogeangle)
print "[Foward kinematics] position and quatanion are"
print posquo
#calc inverse dynamic
forcemoment = cr.getforce(jointangle,jointtau)
print "[Inverce dynamics] forcemoment is"
print forcemoment
# calc forward dynamic
tau = cr.gettau(jointangle,forcemoment)
print "[Foward dynamics] tau is "
print tau