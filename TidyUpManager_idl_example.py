#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file TidyUpManager_idl_examplefile.py
 @brief Python example implementations generated from TidyUpManager.idl
 @date $Date$


"""

import omniORB
from omniORB import CORBA, PortableServer
import ogata, ogata__POA

import TidyUpManager_idl
import ManipulatorCommonInterface_DataTypes_idl
import ManipulatorCommonInterface_MiddleLevel_idl
import ManipulatorCommonInterface_Common_idl

import RGBDCamera
import OpenRTM_aist

import os, math
import keras
from keras.utils import np_utils
from keras.layers.convolutional import Conv2D, MaxPooling2D
from keras.models import Sequential
#from keras.models import Sequential
from tensorflow.keras.models import model_from_json
from keras.layers.core import Dense, Dropout, Activation, Flatten
from keras.preprocessing.image import array_to_img, img_to_array, load_img
#from keras.preprocessing.image import list_pictures
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
# import matplotlib.pyplot aplt


# Import Service implementation class
# <rtc-template block="service_impl">

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import JARA_ARM, JARA_ARM__POA
import JARA_ARM, JARA_ARM__POA

#以下list_picturesの定義
def list_pictures(directory, ext='png'):
    return [os.path.join(root, f)
            for root, _, files in os.walk(directory) for f in files
            if re.match(r'([\w]+\.(?:' + ext +'))', f.lower())]

class TidyUpManager_i (ogata__POA.TidyUpManager):
    """
    @class TidyUpManager_i
    Example class implementing IDL interface ogata.TidyUpManager
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # RETURN_VALUE tidyup(in RTC::TimedPose2D path, in RTC::TimedString kind)
    def tidyup(self, path, kind):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result



class StringNavigationCommanderService_i (ogata__POA.StringNavigationCommanderService):
    """
    @class StringNavigationCommanderService_i
    Example class implementing IDL interface ogata.StringNavigationCommanderService
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # RETURN_VALUE move(in RTC::TimedPose2D path)
    def move(self, path):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result



class Picker_i (ogata__POA.Picker):
    """
    @class Picker_i
    Example class implementing IDL interface ogata.Picker
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        camera_arg = [None] * ((len(RGBDCamera._d_TimedRGBDCameraImage) - 4) // 2)
        #self._d_camera = Img.TimedCameraImage(*camera_arg)
        self._d_camera = RGBDCamera.TimedRGBDCameraImage(*camera_arg)
        """
        """
        self._cameraIn = OpenRTM_aist.InPort("camera", self._d_camera)

        """
        """
        self._manipCommonPort = OpenRTM_aist.CorbaPort("manipCommon")
        """
        """
        self._manipMiddlePort = OpenRTM_aist.CorbaPort("manipMiddle")

        

        """
        """
        self._manipCommon = OpenRTM_aist.CorbaConsumer(interfaceType=JARA_ARM.ManipulatorCommonInterface_Common)
        """
        """
        self._manipMiddle = OpenRTM_aist.CorbaConsumer(interfaceType=JARA_ARM.ManipulatorCommonInterface_Middle)

        # initialize of configuration-data.
        # <rtc-template block="init_conf_param">
        """
        
         - Name:  debug
         - DefaultValue: 1
        """
        self._debug = [1]

        self._model = None
        
        """
        
         - Name:  gripper_close_ratio
         - DefaultValue: 0.1
        """
        self._gripper_close_ratio = [0.1]

        self._model = None

        # Set service consumers to Ports
        self._manipCommonPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._manipCommon)
        self._manipMiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._manipMiddle)
        

    # RETURN_VALUE pick(in RTC::TimedString kind)
    def pick(self, kind):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result
        #モデルとパラメータの読み込み
        if kind.data=="PET":
            print("get data;PET")
 
            self._model = model_from_json(open('model_log_pet.json', 'r').read())
            print('pet model loaded')
            self._model.compile(loss='mean_squared_error',optimizer='SGD',metrics=['accuracy'])
            self._model.load_weights('param_pet.hdf5')
            print('pet weight loaded')
        elif kind.data=="LEGO":
            print("get data;LEGO")

            self._model = model_from_json(open('model_log_lego.json', 'r').read())
            print('lego model loaded')
            self._model.compile(loss='mean_squared_error',optimizer='SGD',metrics=['accuracy'])
            self._model.load_weights('param_lego.hdf5')
            print('lego weight loaded')
        else:
            print("unexpected word")
            print(kind)
            return ogata.RETVAL_UNKNOWN_ERROR

        self._manipCommon._ptr().servoON()
        print("orochi arm servo on")
        self._manipMiddle._ptr().setSpeedJoint(30)
        print("set speed")

        self._manipMiddle._ptr().movePTPJointAbs([0, math.pi/4,math.pi/4, 0, math.pi/2, 0])
        self._manipMiddle._ptr().moveGripper(80)

        if self._cameraIn.isNew():
            data = self._cameraIn.read()
            w = data.data.cameraImage.image.width
            h = data.data.cameraImage.image.height
            cimg = np.ndarray(shape=(h, w, 3), dtype=float)
            csize = len(data.data.cameraImage.image.raw_data)

        for i in range(0,w*h):
            cimg[i//w, i%w, 0] = data.data.cameraImage.image.raw_data[i*3+0]
            cimg[i//w, i%w, 1] = data.data.cameraImage.image.raw_data[i*3+1]
            cimg[i//w, i%w, 2] = data.data.cameraImage.image.raw_data[i*3+2]
            pass

        dw = data.data.depthImage.width
        dh = data.data.depthImage.height
        dimg = np.ndarray(shape=(dh, dw), dtype=float)
        dsize = len(data.data.depthImage.raw_data)
        for i in range(dh):
            for j in range(dw):
                dimg[i, j] = data.data.depthImage.raw_data[i*dw + j]
                pass

        dimg = depth_image_to_cv2_image(dimg)
        cv2.imwrite('depth_img.png' ,dimg)
        cv2.imwrite('rgb_img.png', cimg)

        cimg = cv2.resize(cimg, (64, 64))
        dimg = cv2.resize(dimg, (64, 64))
        dimg = np.reshape(dimg, (64,64,1))
        img3 = np.c_[cimg, dimg]
        print(img3.shape)
        result = self._model.predict(np.asarray([img3/256]))
        x = result[0][0]*0.12 + 0.12
        y = result[0][1]*0.24 - 0.12
        th = result[0][2]*2*math.pi-math.pi
        print(result)
        print(x)
        print(y)
        print(th)

        z = -100.25 / 1000.0
        z_min = -130.00 / 1000.0
        s2 = math.sin(th)
        c2 = math.cos(th)
        carPos = JARA_ARM.CarPosWithElbow([[0,0,0,0],[0,0,0,0],[0,0,0,0]], 1.0, 1)
        carPos.carPos[0][0] = -c2;  carPos.carPos[0][1] = s2; carPos.carPos[0][2] =  0.0; carPos.carPos[0][3] = x;
        carPos.carPos[1][0] =  s2;  carPos.carPos[1][1] = c2; carPos.carPos[1][2] =  0.0; carPos.carPos[1][3] = y;
        carPos.carPos[2][0] =  0.0; carPos.carPos[2][1] = 0; carPos.carPos[2][2] = -1.0; carPos.carPos[2][3] = z;
        self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
            
        time.sleep(1.0)

        carPos.carPos[2][3] = z_min
            
        self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
        time.sleep(1.0)

        self._manipMiddle._ptr().moveGripper(30)
        time.sleep(1.0)
        carPos.carPos[2][3] = z
        self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
        time.sleep(1.0)
        print("done")

        self._manipMiddle._ptr().movePTPJointAbs([0, math.pi/4,math.pi/4, 0, math.pi/2, 0])

        return ogata.RETVAL_OK






        


class Droper_i (ogata__POA.Droper):
    """
    @class Droper_i
    Example class implementing IDL interface ogata.Droper
    """

    def __init__(self):
        """
        @brief standard constructor
        Initialise member variables here
        """
        pass

    # RETURN_VALUE drop(in RTC::TimedString kind)
    def drop(self, kind):
        raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

        time.sleep(3.0)
        self._manipMiddle._ptr().movePTPJointAbs([(math.pi)*3/2,0, math.pi/2, 0, math.pi/2, 0])
        self._manipMiddle._ptr().moveGripper(80)

        return RETVAL_OK


if __name__ == "__main__":
    import sys
    
    # Initialise the ORB
    orb = CORBA.ORB_init(sys.argv)
    
    # As an example, we activate an object in the Root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Create an instance of a servant class
    servant = TidyUpManager_i()

    # Activate it in the Root POA
    poa.activate_object(servant)

    # Get the object reference to the object
    objref = servant._this()
    
    # Print a stringified IOR for it
    #print orb.object_to_string(objref)

    # Activate the Root POA's manager
    poa._get_the_POAManager().activate()

    # Run the ORB, blocking this thread
    orb.run()


