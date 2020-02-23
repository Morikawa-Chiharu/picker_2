#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file ArmImagePredictor_KerasTest.py
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

# Import Service implementation class
# <rtc-template block="service_impl">
from TidyUpManager_idl_example import *

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
import ogata, ogata__POA


# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
armimagepredictor_kerastest_spec = ["implementation_id", "ArmImagePredictor_KerasTest", 
		 "type_name",         "ArmImagePredictor_KerasTest", 
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
# @class ArmImagePredictor_KerasTest
# @brief Arm Image Predictor using Keras RT Component
# 
# 
class ArmImagePredictor_KerasTest(OpenRTM_aist.DataFlowComponentBase):
	
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_camera = OpenRTM_aist.instantiateDataType(Img.CameraImage)
		"""
		"""
		self._cameraOut = OpenRTM_aist.OutPort("camera", self._d_camera)

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
		self._manipCommon = ManipulatorCommonInterface_Common_i()
		"""
		"""
		self._manipMiddle = ManipulatorCommonInterface_Middle_i()
		

		"""
		"""
		self._TidyUpManager = OpenRTM_aist.CorbaConsumer(interfaceType=ogata.Picker)

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
		
		# Set OutPort buffers
		self.addOutPort("camera",self._cameraOut)
		
		# Set service provider to Ports
		self._manipCommonPort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", self._manipCommon)
		self._manipMiddlePort.registerProvider("JARA_ARM_ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", self._manipMiddle)
		
		# Set service consumers to Ports
		self._pickerPort.registerConsumer("TidyUpManager", "ogata::Picker", self._TidyUpManager)
		
		# Set CORBA Service Ports
		self.addPort(self._manipCommonPort)
		self.addPort(self._manipMiddlePort)
		self.addPort(self._pickerPort)
		
		return RTC.RTC_OK
	
	#	##
	#	# 
	#	# The finalize action (on ALIVE->END transition)
	#	# formaer rtc_exiting_entry()
	#	# 
	#	# @return RTC::ReturnCode_t
	#
	#	# 
	#def onFinalize(self):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The startup action when ExecutionContext startup
	#	# former rtc_starting_entry()
	#	# 
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onStartup(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The shutdown action when ExecutionContext stop
	#	# former rtc_stopping_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onShutdown(self, ec_id):
	#
	#	return RTC.RTC_OK
	
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
	
	#	##
	#	#
	#	# The aborting action when main logic error occurred.
	#	# former rtc_aborting_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onAborting(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The error action in ERROR state
	#	# former rtc_error_do()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onError(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The reset action that is invoked resetting
	#	# This is same but different the former rtc_init_entry()
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onReset(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The state update action that is invoked after onExecute() action
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#

	#	#
	#def onStateUpdate(self, ec_id):
	#
	#	return RTC.RTC_OK
	
	#	##
	#	#
	#	# The action that is invoked when execution context's rate is changed
	#	# no corresponding operation exists in OpenRTm-aist-0.2.0
	#	#
	#	# @param ec_id target ExecutionContext Id
	#	#
	#	# @return RTC::ReturnCode_t
	#	#
	#	#
	#def onRateChanged(self, ec_id):
	#
	#	return RTC.RTC_OK
	



def ArmImagePredictor_KerasTestInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=armimagepredictor_kerastest_spec)
    manager.registerFactory(profile,
                            ArmImagePredictor_KerasTest,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    ArmImagePredictor_KerasTestInit(manager)

    # Create a component
    comp = manager.createComponent("ArmImagePredictor_KerasTest")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

