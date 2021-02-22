import pandas as pd
import matplotlib as plt
import numpy as np
import skinematics as skin
import angular_convertion



#
#
#


acc = pd.read_csv("data/02-19-2021_rotation/acceleration.csv")
ang = pd.read_csv("data/02-19-2021_rotation/angularVeloc.csv")
ori = pd.read_csv("data/02-19-2021_rotation/orientation.csv")


singel_acc = acc.iloc[0, 1:4].values
singel_ang = ang.iloc[0, 1:4].values

#print(singel_acc,"\n", singel_ang)

#
#
#

#quat_1 = angular_convertion.convert_single("data/02-19-2021_rotation/acceleration.csv", 0)
#quat_2 = angular_convertion.convert_single("data/02-19-2021_rotation/acceleration.csv", 1)

#skin.view.orientation(quats=(quat_1*quat_2))
#skin.view.orientation(angular_convertion.convert_multiple("data/trickshot/orientation.csv"))

viewer = skin.view.Orientation_OGL(angular_convertion.convert_multiple("data/trickshot/orientation.csv"))
viewer.run(rate=10, looping=True)