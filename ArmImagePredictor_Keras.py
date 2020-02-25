#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ArmImagePredictor_Keras.py
 @brief Arm Image Predictor using Keras RT Component
 @date $Date$


"""
import sys
import time
sys.path.append(".")

# Import RTM module
import RTC
import OpenRTM_aist

import TidyUpManager_idl
import ManipulatorCommonInterface_DataTypes_idl
import ManipulatorCommonInterface_MiddleLevel_idl
import ManipulatorCommonInterface_Common_idl

#import Img
import RGBDCamera




# Import Service implementation class
# <rtc-template block="service_impl">
from TidyUpManager_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import ogata, ogata__POA


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
import cv2
import pandas as pd
from sklearn.model_selection import train_test_split

#以下list_picturesの定義
def list_pictures(directory, ext='png'):
    return [os.path.join(root, f)
            for root, _, files in os.walk(directory) for f in files
            if re.match(r'([\w]+\.(?:' + ext +'))', f.lower())]

print("library imported")

# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
armimagepredictor_keras_spec = ["implementation_id", "ArmImagePredictor_Keras", 
         "type_name",         "ArmImagePredictor_Keras", 
         "description",       "Arm Image Predictor using Keras RT Component", 
         "version",           "1.0.0", 
         "vendor",            "ogata_lab", 
         "category",          "Experimental", 
         "activity_type",     "STATIC", 
         "max_instance",      "1", 
         "language",          "Python", 
         "lang_type",         "SCRIPT",
         "conf.default.debug", "1",

         "conf.__widget__.debug", "text",

         "conf.__type__.debug", "int",

         ""]
# </rtc-template>

##
# @class ArmImagePredictor_Keras
# @brief Arm Image Predictor using Keras RT Component
# 
# 
class ArmImagePredictor_Keras(OpenRTM_aist.DataFlowComponentBase):
    
    ##
    # @brief constructor
    # @param manager Maneger Object
    # 
    def __init__(self, manager):


        OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)
        camera_arg = [None] * ((len(RGBDCamera._d_TimedRGBDCameraImage) - 4) // 2)

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
        self._pickerPort = OpenRTM_aist.CorbaPort("picker")

        """
        """
        self._TidyUpManager = Picker_i()
        # ここでRTCの参照を設定する
        self._TidyUpManager.set_rtc(self)
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
        
        # </rtc-template>


         
    ##
    #
    # The initialize action (on CREATED->ALIVE transition)
    # formaer rtc_init_entry() 
    # 
    # @return RTC::ReturnCode_t
    # 
    #
    def onInitialize(self):
        # Bind variables and configuration variable
        self.bindParameter("debug", self._debug, "1")
        
        # Set InPort buffers
        self.addInPort("camera",self._cameraIn)
        
        # Set OutPort buffers
        
        # Set service provider to Ports
        self._pickerPort.registerProvider("TidyUpManager", "ogata::Picker", self._TidyUpManager)
        
        # Set service consumers to Ports
        self._manipCommonPort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._manipCommon)
        self._manipMiddlePort.registerConsumer("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._manipMiddle)
        
        # Set CORBA Service Ports
        self.addPort(self._manipCommonPort)
        self.addPort(self._manipMiddlePort)
        self.addPort(self._pickerPort)
        
        return RTC.RTC_OK
    
    ###
    ## 
    ## The finalize action (on ALIVE->END transition)
    ## formaer rtc_exiting_entry()
    ## 
    ## @return RTC::ReturnCode_t
    #
    ## 
    #def onFinalize(self):
    #
    #    return RTC.RTC_OK
    
    ###
    ##
    ## The startup action when ExecutionContext startup
    ## former rtc_starting_entry()
    ## 
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onStartup(self, ec_id):
    #
    #    return RTC.RTC_OK
    
    ###
    ##
    ## The shutdown action when ExecutionContext stop
    ## former rtc_stopping_entry()
    ##
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onShutdown(self, ec_id):
    #
    #    return RTC.RTC_OK
    
    ##
    #
    # The activated action (Active state entry action)
    # former rtc_active_entry()
    #
    # @param ec_id target ExecutionContext Id
    # 
    # @return RTC::ReturnCode_t
    #
    #
    def onActivated(self, ec_id):
    
        return RTC.RTC_OK
    
    ##
    #
    # The deactivated action (Active state exit action)
    # former rtc_active_exit()
    #
    # @param ec_id target ExecutionContext Id
    #
    # @return RTC::ReturnCode_t
    #
    #
    def onDeactivated(self, ec_id):
    
        return RTC.RTC_OK
    
    ##
    #
    # The execution action that is invoked periodically
    # former rtc_active_do()
    #
    # @param ec_id target ExecutionContext Id
    #
    # @return RTC::ReturnCode_t
    #
    #
    def onExecute(self, ec_id):
    
        return RTC.RTC_OK
    
    ###
    ##
    ## The aborting action when main logic error occurred.
    ## former rtc_aborting_entry()
    ##
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onAborting(self, ec_id):
    #
    #    return RTC.RTC_OK
    
    ###
    ##
    ## The error action in ERROR state
    ## former rtc_error_do()
    ##
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onError(self, ec_id):
    #
    #    return RTC.RTC_OK
    
    ###
    ##
    ## The reset action that is invoked resetting
    ## This is same but different the former rtc_init_entry()
    ##
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onReset(self, ec_id):
    #
    #    return RTC.RTC_OK
    
    ###
    ##
    ## The state update action that is invoked after onExecute() action
    ## no corresponding operation exists in OpenRTm-aist-0.2.0
    ##
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##

    ##
    #def onStateUpdate(self, ec_id):
    #
    #    return RTC.RTC_OK
    
    ###
    ##
    ## The action that is invoked when execution context's rate is changed
    ## no corresponding operation exists in OpenRTm-aist-0.2.0
    ##
    ## @param ec_id target ExecutionContext Id
    ##
    ## @return RTC::ReturnCode_t
    ##
    ##
    #def onRateChanged(self, ec_id):
    #
    #    return RTC.RTC_OK

    def pick(self, kind):
        # ysuga 修正
        # この部分，Picker_iのpickメソッドからコピペした．
        # インデントが間違っているかもしれないので修正すること
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

        time.sleep(3.0)

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
            

        z = -40.25 / 1000.0
        z_min = -60.00 / 1000.0
        s2 = math.sin(th)
        c2 = math.cos(th)
        carPos = JARA_ARM.CarPosWithElbow([[0,0,0,0],[0,0,0,0],[0,0,0,0]], 1.0, 1)
        carPos.carPos[0][0] = -c2;  carPos.carPos[0][1] = s2; carPos.carPos[0][2] =  0.0; carPos.carPos[0][3] = x;
        carPos.carPos[1][0] =  s2;  carPos.carPos[1][1] = c2; carPos.carPos[1][2] =  0.0; carPos.carPos[1][3] = y;
        carPos.carPos[2][0] =  0.0; carPos.carPos[2][1] = 0; carPos.carPos[2][2] = -1.0; carPos.carPos[2][3] = z;
        self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
                
        time.sleep(1.0)

        carPos.carPos[2][3] = z_min

        time.sleep(1.0)
        carPos.carPos[2][3] = z_min
            
        self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
        time.sleep(1.0)

        self._manipMiddle._ptr().moveGripper(45)
        time.sleep(1.0)
        self._manipMiddle._ptr().moveGripper(35)
        time.sleep(1.0)

        carPos.carPos[2][3] = z
        self._manipMiddle._ptr().movePTPCartesianAbs(carPos)
        time.sleep(1.0)

        self._manipMiddle._ptr().movePTPJointAbs([0, math.pi/4,math.pi/4, 0, math.pi/2, 0])
        print("pick done")
        time.sleep(1.0)

        if kind.data=="PET":
            
            print("start moving to pet box")
            self._manipMiddle._ptr().movePTPJointAbs([(math.pi)*3/4,0, math.pi/2, 0, math.pi/2, 0])
            time.sleep(1.0)
            self._manipMiddle._ptr().moveGripper(85)
            time.sleep(1.0)
            self._manipMiddle._ptr().moveGripper(90)

            print('pet dropped')
        elif kind.data=="LEGO":
            print("start moving to lego box")
            self._manipMiddle._ptr().movePTPJointAbs([-(math.pi)*3/4,0, math.pi/2, 0, math.pi/2, 0])
            time.sleep(1.0)
            self._manipMiddle._ptr().moveGripper(85)
            time.sleep(1.0)
            self._manipMiddle._ptr().moveGripper(90)

            
            print('lego dropped')
        else:
            print("unexpected word")
            print(kind)
            return ogata.RETVAL_UNKNOWN_ERROR

        time.sleep(1.0)
        self._manipMiddle._ptr().movePTPJointAbs([0, math.pi/4,math.pi/4, 0, math.pi/2, 0])
        print("done")

        return ogata.RETVAL_OK
                

def depth_image_to_cv2_image(v1):
    h, w = v1.shape
    max_value = 0.64 # np.max(v1)
    min_value = 0.62 # np.min(v1)
    v2 = (v1 - min_value) / (max_value - min_value)
    img = np.ones([h, w, 1])

    for i in range(w):
        for j in range(h):
            img[j, i ,0] = (1.0-v2[j,i]) * 255.0 if v2[j, i] > 0.0 else 0
    return img



def ArmImagePredictor_KerasInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=armimagepredictor_keras_spec)
    manager.registerFactory(profile,
                            ArmImagePredictor_Keras,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    ArmImagePredictor_KerasInit(manager)

    # Create a component
    comp = manager.createComponent("ArmImagePredictor_Keras")

def main():
    mgr = OpenRTM_aist.Manager.init(sys.argv)
    mgr.setModuleInitProc(MyModuleInit)
    mgr.activateManager()
    mgr.runManager()

if __name__ == "__main__":
    main()

