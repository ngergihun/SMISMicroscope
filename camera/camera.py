# Camera packages
from ids_peak import ids_peak
from ids_peak_ipl import ids_peak_ipl
from ids_peak import ids_peak_ipl_extension
import numpy as np
import copy

import logging

class BaseCamera:
    def __init__(self) -> None:
        self.image = []

    def open_device(self):
        raise NotImplementedError
    
    def close_device(self):
        raise NotImplementedError
    
    def start_acquisition(self):
        raise NotImplementedError
    
    def stop_acquisition(self):
        raise NotImplementedError
    
    def set_fps(self):
        raise NotImplementedError


class IDS_Camera(BaseCamera):
    def __init__(self) -> None:
        super().__init__()

        self.__device = None
        self.__nodemap_remote_device = None
        self.__datastream = None

        self.__acquisition_running = False

        self.__image_converter = ids_peak_ipl.ImageConverter()
        # initialize peak library
        ids_peak.Library.Initialize()

        self.name = None
        self.pixelformat = ids_peak_ipl.PixelFormatName_RGBa8
        self.fps_limit = 50

    def open_device(self):
        try:
            # Create instance of the device manager
            device_manager = ids_peak.DeviceManager.Instance()
            device_manager.Update()
            # Return if no device was found
            if device_manager.Devices().empty():
                logging.critical("Error", "No device found!")
                return False
            # Open the first openable device in the managers device list
            for device in device_manager.Devices():
                if device.IsOpenable():
                    self.__device = device.OpenDevice(ids_peak.DeviceAccessType_Control)
                    break
            self.name = self.__device.DisplayName()
            # Return if no device could be opened
            if self.__device is None:
                logging.critical("Device could not be opened!")
                return False
            
            # Open standard data stream
            datastreams = self.__device.DataStreams()
            if datastreams.empty():
                logging.critical("Device has no DataStream!")
                self.__device = None
                return False
            self.__datastream = datastreams[0].OpenDataStream()

            # Get nodemap of the remote device for all accesses to the genicam nodemap tree
            self.__nodemap_remote_device = self.__device.RemoteDevice().NodeMaps()[0]

            return True
        
        except ids_peak.Exception as e:
            logging.critical("Exception" + str(e))

        return False

    def close_device(self):
        """
        Stop acquisition if still running and close datastream and nodemap of the device
        """
        # Stop Acquisition in case it is still running
        self.__stop_acquisition()

        # If a datastream has been opened, try to revoke its image buffers
        if self.__datastream is not None:
            try:
                for buffer in self.__datastream.AnnouncedBuffers():
                    self.__datastream.RevokeBuffer(buffer)
            except Exception as e:
                logging.critical("Exception:" + str(e))

    def start_acquisition(self):
        """Start Acquisition on camera to receive images."""
        # Check that a device is opened and that the acquisition is NOT running. If not, return.
        if self.__device is None:
            return False
        if self.__acquisition_running is True:
            return True

        try:
            # Lock critical features to prevent them from changing during acquisition
            self.__nodemap_remote_device.FindNode("TLParamsLocked").SetValue(1)

            image_width = self.__nodemap_remote_device.FindNode("Width").Value()
            image_height = self.__nodemap_remote_device.FindNode("Height").Value()
            input_pixel_format = ids_peak_ipl.PixelFormat(self.__nodemap_remote_device.FindNode("PixelFormat").CurrentEntry().Value())

            # Pre-allocate conversion buffers to speed up first image conversion
            # while the acquisition is running
            self.__image_converter = ids_peak_ipl.ImageConverter()
            self.__image_converter.PreAllocateConversion(input_pixel_format, self.pixelformat,image_width, image_height)

            # Start acquisition on camera
            self.__datastream.StartAcquisition()
            self.__nodemap_remote_device.FindNode("AcquisitionStart").Execute()
            self.__nodemap_remote_device.FindNode("AcquisitionStart").WaitUntilDone()

        except Exception as e:
            logging.error("Exception: " + str(e))
            return False

        self.__acquisition_running = True

        return True
    
    def stop_acquisition(self):
        """Stop acquisition timer and stop acquisition on camera"""

        # Check that a device is opened and that the acquisition is running. If not, return.
        if self.__device is None or self.__acquisition_running is False:
            return

        # Otherwise try to stop acquisition
        try:
            remote_nodemap = self.__device.RemoteDevice().NodeMaps()[0]
            remote_nodemap.FindNode("AcquisitionStop").Execute()

            # Stop and flush datastream
            self.__datastream.KillWait()
            self.__datastream.StopAcquisition(ids_peak.AcquisitionStopMode_Default)
            self.__datastream.Flush(ids_peak.DataStreamFlushMode_DiscardAll)

            self.__acquisition_running = False

            # Unlock parameters after acquisition stop
            if self.__nodemap_remote_device is not None:
                try:
                    self.__nodemap_remote_device.FindNode("TLParamsLocked").SetValue(0)
                except Exception as e:
                    logging.info("Exception" + str(e))

        except Exception as e:
            logging.info("Exception" + str(e))

    def set_fps(self,fps):
        # Get the maximum framerate possible, limit it to the configured fps_limit. If the limit can't be reached, set
        # acquisition interval to the maximum possible framerate
        try:
            max_fps = self.__nodemap_remote_device.FindNode("AcquisitionFrameRate").Maximum()
            target_fps = min(max_fps, fps)
            self.__nodemap_remote_device.FindNode("AcquisitionFrameRate").SetValue(target_fps)
        except ids_peak.Exception:
            # AcquisitionFrameRate is not available. Unable to limit fps. Print warning and continue on.
            logging.warning("Warning: Unable to limit fps, since the AcquisitionFrameRate." +
                            "Program will continue without limit.")

    def set_singleshot(self):
        if self.__acquisition_running:
            self.stop_acquisition()
        
        self.__nodemap_remote_device.FindNode("AcquisitionMode").SetCurrentEntry("SingleFrame")
        self.__nodemap_remote_device.FindNode("TriggerSelector").SetCurrentEntry("ExposureStart")
        self.__nodemap_remote_device.FindNode("TriggerSource").SetCurrentEntry("Software")
        self.__nodemap_remote_device.FindNode("TriggerMode").SetCurrentEntry("On")

        # Get the payload size for correct buffer allocation
        payload_size = self.__nodemap_remote_device.FindNode("PayloadSize").Value()
        print(payload_size)
        buffer_count_max = self.__datastream.NumBuffersAnnouncedMinRequired()
        print(buffer_count_max)
        for i in range(buffer_count_max):
            buffer = self.__datastream.AllocAndAnnounceBuffer(payload_size)
            self.__datastream.QueueBuffer(buffer)

        self.start_acquisition()
        self.__nodemap_remote_device.FindNode("TriggerSoftware").Execute()

        try:
            self.capture_image()
        except:
            pass

    def set_continuous(self):
        if self.__acquisition_running:
            self.stop_acquisition()

        self.__nodemap_remote_device.FindNode("AcquisitionMode").SetCurrentEntry("Continuous")
        self.__nodemap_remote_device.FindNode("TriggerSelector").SetCurrentEntry("ExposureStart")
        self.__nodemap_remote_device.FindNode("TriggerMode").SetCurrentEntry("Off")

        expvalue = self.__nodemap_remote_device.FindNode("ExposureTime").Value()
        print(expvalue)
        newfps = round(1000000/expvalue,1)
        self.set_fps(newfps)

        # Get the payload size for correct buffer allocation
        payload_size = self.__nodemap_remote_device.FindNode("PayloadSize").Value()
        print(payload_size)
        buffer_count_max = self.__datastream.NumBuffersAnnouncedMinRequired()
        print(buffer_count_max)
        for i in range(buffer_count_max):
            buffer = self.__datastream.AllocAndAnnounceBuffer(payload_size)
            self.__datastream.QueueBuffer(buffer)
        # start acquisition again
        self.start_acquisition()
        self.__nodemap_remote_device.FindNode("AcquisitionStart").Execute()
        self.__nodemap_remote_device.FindNode("AcquisitionStart").WaitUntilDone()

        try:
            self.get_image()
        except:
            pass
    
    def set_exposuretime(self, value):
        """ Sets the exposuretime in microseconds for timed exposure mode"""
        self.__nodemap_remote_device.FindNode("ExposureTime").SetValue(value)

    def get_limits(self):
        """ Get the limits and parameters of the specific device """
        self.image_width = self.__nodemap_remote_device.FindNode("Width").Value()
        self.image_height = self.__nodemap_remote_device.FindNode("Height").Value()
        
        self.min_exposure_time = self.__nodemap_remote_device.FindNode("ExposureTime").Minimum()
        self.max_exposure_time = self.__nodemap_remote_device.FindNode("ExposureTime").Maximum()

    def get_image(self):
        """
        Gets the raw image from the buffer and convert it to a numpy array image from the 
        """
        try:
            # Get buffer from device's datastream
            buffer = self.__datastream.WaitForFinishedBuffer(5000)
            # Create IDS peak IPL image for debayering and convert it to RGBa8 format
            ipl_image = ids_peak_ipl_extension.BufferToImage(buffer)
            converted_ipl_image = self.__image_converter.Convert(ipl_image, self.pixelformat)
            # Queue buffer so that it can be used again
            self.__datastream.QueueBuffer(buffer)
            # Get raw image data from converted image and construct a QImage from it
            np_image = converted_ipl_image.get_numpy_3D()
            # Make an extra copy of the image to make sure that memory is copied and can't get overwritten later on
            self.image = np.ascontiguousarray(copy.deepcopy(np_image), dtype="uint8")

            return self.image

        except ids_peak.Exception as e:
            logging.error("Exception: " + str(e))

    def capture_image(self):
        self.__nodemap_remote_device.FindNode("AcquisitionStart").Execute()
        self.__nodemap_remote_device.FindNode("AcquisitionStart").WaitUntilDone()
        self.__nodemap_remote_device.FindNode("TriggerSoftware").Execute()
        return self.get_image()

    def __del__(self):
        self.destroy_all()

    def destroy_all(self):
        # Stop acquisition
        self.stop_acquisition()
        # Close device and peak library
        self.close_device()
        ids_peak.Library.Close()