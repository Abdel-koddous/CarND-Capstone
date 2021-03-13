import os

import cv2
import numpy as np
import rospy
import tensorflow as tf
from styx_msgs.msg import TrafficLight

# Before executing script on Udacity server - upgrade to TF2
# python -m pip install --upgrade pip
# pip install --upgrade tensorflow==2



class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        self.current_light = TrafficLight.UNKNOWN

        self.saved_model_path = "/home/workspace/my_models/my_tl_mobnet/export/saved_model" 
        rospy.logwarn( "----------------- Make sure to migrate to TF2 for this saved_model to be loaded properly ... " )
        rospy.logwarn( "----------------- For this run the 2 following commands before running the project app " )
        rospy.logwarn( "----------------- >> python -m pip install --upgrade pip " )
        rospy.logwarn( "----------------- >> pip install --upgrade tensorflow==2 " )
        rospy.logwarn( "----------------- Loading TF2 Object detection model, might take a while... " )
        self.loaded = tf.saved_model.load(self.saved_model_path)
        rospy.logwarn( "----------------- Object detection model loaded successfully -------------------- " )


    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        # return TrafficLight.RED
        # TODO implement light color prediction
        # creating an image object  
        img_np = np.array(image) 

        # convert np array to tensor
        input_tensor = tf.convert_to_tensor(img_np)

        # The model expects a batch of images, so add an axis with `tf.newaxis`.
        input_tensor = input_tensor[tf.newaxis, ...]


        detections = self.loaded(input_tensor)

        num_detections = int(detections.pop('num_detections'))

        # detection_classes should be ints.
        detections_dict = {key: value[0, :num_detections].numpy() for key, value in detections.items()}


        # detection_classes should be ints.
        detections_dict['detection_classes'] = detections_dict['detection_classes'].astype(np.int64)

        label_id_offset = 1

        # DEBUG - can do it in a cleaner way :0
        tl_classes = {3: 'green', 2: 'red'}
        top_classes_prediction = list(detections_dict['detection_classes']+label_id_offset)[:5] 
        #print(top_classes_prediction)
        for i in range(len(top_classes_prediction)):
            if top_classes_prediction[i] == 2:
                top_classes_prediction[i] = 'green'
            elif top_classes_prediction[i] == 3:
                top_classes_prediction[i] = 'red'


        #print("--------->", image_path, "<-----------")
        #print( top_classes_prediction ) 
        #print(detections_dict['detection_scores'][:5], '\n' )

        # basic red tl logic
        if top_classes_prediction[0] == 'red' and detections_dict['detection_scores'][0] >= 0.60:
            #print("-------------> RED TRAFFIC LIGHT <----------------\n")
            self.current_light = TrafficLight.RED
            #rospy.logwarn( "----------------- Taffic light is RED !!! -------------------- " )
            self.display_predictions_scores( top_classes_prediction, detections_dict['detection_scores'] )
        else:
            #print("No red traffic is detected\n")
            self.current_light = TrafficLight.GREEN
            #rospy.logwarn( "----------------- You're good to go !!! --------: {0} - {1} ".format(top_classes_prediction[0], detections_dict['detection_scores'][0]) )
            self.display_predictions_scores( top_classes_prediction, detections_dict['detection_scores'] )

        return self.current_light
    
    def display_predictions_scores(self, classes, scores):
        N = len(classes)
        display_string = ""
        for i in range(N):
            display_string += str(i+1) + "." + classes[i] + " : " + str(scores[i]) + "  -  "
        
        rospy.logwarn( "Predictions: {0} ".format(display_string) )

