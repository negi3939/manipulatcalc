import IDpy as Ar
import math
cr = Ar.Negi39FIKFID()
#cr.setjointnum(7)
#cr.setdhparameter(0,[0.0,0.0,0.064,-0.5*math.pi])
jointangle = [0.1,0.2,0.3,0.4,0.5,0.6,0.7] #angle rad
jointtau = [1.1,1.2,1.3,1.4,1.5,1.6,1.7] #angle tau N*m
#calc inverse dynamic
forcemoment = cr.getforce(jointangle,jointtau)
print "forcemoment"
print forcemoment
# calc forward dynamic
tau = cr.gettau(jointangle,forcemoment)
print "tau"
print tau