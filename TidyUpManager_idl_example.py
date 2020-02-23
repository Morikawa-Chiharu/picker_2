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
        # ysuga修正
        # ここに書いてあったRTCの初期化コードのコピペは無意味

    def set_rtc(self, rtc):
        self._rtc = rtc

    # RETURN_VALUE pick(in RTC::TimedString kind)
    def pick(self, kind):
        #raise CORBA.NO_IMPLEMENT(0, CORBA.COMPLETED_NO)
        # *** Implement me
        # Must return: result

        # ysuga修正
        # ここで処理をしない．RTCのonExecuteなどでやること
        self._rtc.pick(kind)
        
        






        


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


