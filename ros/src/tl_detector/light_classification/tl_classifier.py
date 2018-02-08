#This code has been adapted from
#https://github.com/ColinShaw/self-driving-car-capstone

import rospy
import cv2
import time, os
import numpy as np
import tensorflow as tf
from keras import backend as K
from keras import layers
from keras.models import load_model
from keras.applications.vgg16 import preprocess_input
from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        sess = tf.Session(config=config)
        K.set_session(sess)
        full_path = os.path.realpath(__file__)
        path, filename = os.path.split(full_path)
        self.model = load_model(path+'/model/udacity_vgg_fine_tuning_combined.h5')

        # https://github.com/fchollet/keras/issues/3517
        self.get_output = K.function([self.model.layers[0].input, K.learning_phase()],
                                     [self.model.layers[-1].output])

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_NEAREST)
        image = np.expand_dims(image, axis=0)
        image = image.astype(dtype=np.float64, copy=False)
        image = image / 255.0
        start_time = time.time()
        pred = self.get_output([image, 0])[0]
        pred = np.argmax(pred)

        # pred 0 == green, 1 == unknown, 2 == red, 3 == yellow
        if pred == 0 or pred == 1:
            state = TrafficLight.GREEN
        else:
            state = TrafficLight.RED

        return state

