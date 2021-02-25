import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import skinematics as skin
import angular_convertion



#
#
# Importing data

# acc = pd.read_csv("data/02-19-2021_rotation/acceleration.csv")
# ang = pd.read_csv("data/02-19-2021_rotation/angularVeloc.csv")
# mag = pd.read_csv("data/02-19-2021_rotation/magneticField.csv")

acc = pd.read_csv("../data/trickshot/acceleration.csv")
ang = pd.read_csv("../data/trickshot/angularVeloc.csv")
mag = pd.read_csv("../data/trickshot/magneticField.csv")

acc = acc.iloc[:, 1:4].values
ang = ang.iloc[:, 1:4].values
mag = mag.iloc[:, 1:4].values

n_acc = np.size(acc, 0)
n_ang = np.size(ang, 0)
n_mag = np.size(mag, 0)

new_acc = np.reshape(acc, (n_acc,3))
new_ang = np.reshape(ang, (n_ang,3))
new_mag = np.reshape(mag, (n_mag,3))

#print("Vanlig acc ", "\n", np.shape(new_acc), "\n\n\n\n")
#print("Vanlig ang ", "\n", np.shape(new_ang), "\n\n\n\n")



# Dataset in quaternion form
quat_acc = angular_convertion.convert_multiple("../data/trickshot/acceleration.csv")
quat_ang = angular_convertion.convert_multiple("../data/trickshot/angularVeloc.csv")
quat_mag = angular_convertion.convert_multiple("../data/trickshot/magneticField.csv")



quat_ori = angular_convertion.convert_multiple("../data/trickshot/orientation.csv")
quat_trans = angular_convertion.transform_without_timestamp("../data/transelatorisk.csv")




#
#
# Kalman Filter
def kalman_filter(lin_acc, ang_veloc, mag_ori):
    """
    Apply kalman filter to IMU data\n 
    :param lin_acc: (N,3)ndarry - Linear acceleration [m/s^2]\n
    :param ang_veloc: (N,3)ndarry - Angular Velocity    [rad/s^2]\n
    :param mag_ori: (N,3)ndarry - Magnetic Fild Orientation\n 

    Returns (N,4) Quaternion\n
    unit quaternion, describing the orientation relativ to the coordinate system spanned by the local magnetic field, and gravity
    """

    x = kalman_data = skin.imus.kalman(rate=10, acc=lin_acc, omega=ang_veloc, mag=mag_ori)
    return x



#
# 
# Visualisation, with coordinate system, but visualisation is slow
def visualise_with_coordinatesystem(quaternion, record_mp4=False):
    """
    Takes Quaternion as input and visualizes the unity vector, with the option to record the file\n
    
    :param quaternion file: Input quaternion\n
    :param bool record_mp4: Record mp4 file\n
    """    
    if record_mp4: 
        print("Recording")  
        #skin.view.orientation(quats=quaternion, out_file="../data/"+str(filename)+".mp4", title_text=title_txt)
        skin.view.orientation(quats=quaternion, out_file="../data/recording.mp4", title_text="Rotation")
    else:  
        print("NOT Recording")
        skin.view.orientation(quaternion)


                

#
#
# Visualisation without coordinate system
def visualiser(quaternion):
    """
    With OpenGL, no coordinate system, but time-accurate\n 
    Takes Quaternion as input and visualises the unity vector, accurrate speed without coordinate system\n\n
    
    :param quaternion file: Input quaternion\n
    """    

    viewer = skin.view.Orientation_OGL(quaternion)   
    viewer.run(rate=10, looping=True)



#
#
# Generates movement matrixes
def transolatoric(ang_velocity, lin_acceleration):
    """
    Translational movement\n
        :param ang_velocity(N,3): Angular Velocity\n
        :param ang_velocity(N,3): Linear Acceleration\n 

    Returns: \n
        q : ndarray(N,3)\n
            Orientation, expressed as a quaternion vector\n 
        pos : ndarray(N,3)\n
            Position in space [m]\n  
        vel : ndarray(N,3)\n
            Velocity in space [m/s]\n 
    """

    q, pos1, vel = skin.imus.analytical(omega=ang_velocity, accMeasured=lin_acceleration,rate=10)
    return q, pos1, vel
    



def main():
    
    #visualise_with_coordinatesystem(quaternion=quat_ori, record_mp4=False)
    #y = kalman_filter(lin_acc=new_acc, ang_veloc=new_ang, mag_ori=new_mag)
    #visualiser(y)
    #transolatoric(ang_velocity=new_ang, lin_acceleration=new_mag)
    
    transelatorisk = pd.read_csv("../data/transelatorisk.csv")
    
    orientation = transelatorisk.iloc[1:-1,0]
    position = transelatorisk.iloc[1:-1,1] 
    velocity = transelatorisk.iloc[1:-1,2]
    """ 
    plt.figure(3)

    plt.subplot(221)
    plt.plot(orientation)
    plt.yscale("linear")
    plt.title("Orientation")

    plt.subplot(222)
    plt.plot(position)
    plt.yscale("linear")
    plt.title("Position")

    plt.subplot(223)
    plt.plot(velocity)
    plt.yscale("linear")
    plt.title("Velocity")
    """

    fig, axs = plt.subplots(3, sharex=True, sharey=True)
    fig.suptitle('Sharing both axes')
    axs[0].plot(orientation)
    

    axs[1].plot(position)
    axs[2].plot(velocity)




    #plt.subplots_adjust(top=0.92, bottom=0.08, left=0.10, right=0.95, hspace=0.25,
    #                wspace=0.35)



    #plt.plot(orientation)
    #plt.plot(position)
    #plt.plot(velocity)

    plt.legend("Transelatoric Recording")
    plt.show()

if __name__=="__main__":
    main()