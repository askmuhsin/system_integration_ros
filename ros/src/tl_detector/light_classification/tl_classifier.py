from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy
import yaml
import cv2
import os

MAX_IMAGE_WIDTH = 300
MAX_IMAGE_HEIGHT = 300

class TLClassifier(object):
    def __init__(self):
        self.model_graph = None
        self.session = None
        self.image_counter = 0
        self.classes = {1: TrafficLight.RED,
                        2: TrafficLight.YELLOW,
                        3: TrafficLight.GREEN,
                        4: TrafficLight.UNKNOWN}

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        self.load_model(self.get_model_path())               ## <<ROUTE 1>>

    def load_model(self, model_path):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        self.model_graph = tf.Graph()
        with tf.Session(graph=self.model_graph, config=config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

    def get_model_path(self):
        return os.path.dirname(os.path.realpath(__file__)) + self.config['detection_model']

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        class_index, probability = self.predict(image)          ## <<ROUTE 2>>
        if class_index is not None:
            rospy.logdebug("class: %d, probability: %f", class_index, probability)
        return class_index

    def predict(self, image_np, min_score_thresh=0.5):
        image_tensor = self.model_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.model_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.model_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.model_graph.get_tensor_by_name('detection_classes:0')
        image_np = self.process_image(image_np)                 ## <<ROUTE 3>>

        (boxes, scores, classes) = self.session.run(
        [detection_boxes, detection_scores, detection_classes],
        feed_dict={image_tensor: np.expand_dims(image_np, axis=0)})

        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes = np.squeeze(boxes)

        for i, box in enumerate(boxes):
            if scores[i] > min_score_thresh:
                light_class = self.classes[classes[i]]
                # rospy.loginfo("Traffic Light: %d", light_class)
                return light_class, scores[i]
            else:
                return None, None

    def process_image(self, image):
        image = cv2.resize(image, (MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
