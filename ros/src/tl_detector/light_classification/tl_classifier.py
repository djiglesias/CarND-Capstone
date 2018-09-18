from styx_msgs.msg import TrafficLight
import rospy
import tensorflow as tf
import numpy as np
import cv2
import glob
import os, os.path
from functools import partial


class TLClassifier(object):
    def __init__(self):
        # TODO load classifier
        self.DETECTION_THRESHOLD = 0.9
        self.COCO_TL_NUM = 10
        # MODELCHUNK_BASE_DIR = rospy.get_param('~model_path')
        self.readsize = 1024
        #
        # # rospy.logwarn(MODEL_BASE_DIR)
        # PATH_TO_MODEL = MODELCHUNK_BASE_DIR + '/rfcn_resnet101_coco_2018_01_28/frozen_inference_graph.pb'
        # # reassemble the model from chunks. credit goes to team vulture for this idea
        # output = open(PATH_TO_MODEL, 'wb')
        # frozen_model_path = (MODELCHUNK_BASE_DIR + '/rfcn_resnet101_coco_2018_01_28/')
        # chunks = glob.glob(frozen_model_path + 'chunk*')
        # chunks.sort()
        # for filename in chunks:
        #     with open(filename, 'rb') as fileobj:
        #         for chunk in iter(partial(fileobj.read, self.readsize), ''):
        #             output.write(chunk)
        # output.close()

        #
        # self.detection_graph = tf.Graph()
        # with self.detection_graph.as_default():
        #     od_graph_def = tf.GraphDef()
        #     with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
        #         serialized_graph = fid.read()
        #         od_graph_def.ParseFromString(serialized_graph)
        #         tf.import_graph_def(od_graph_def, name='')

        tf.reset_default_graph()
        config = tf.ConfigProto(
            # gpu_options=tf.GPUOptions(per_process_gpu_memory_fraction=0.4),
            # device_count={'GPU': 1}
        )
        model = os.path.join(os.getcwd(), 'light_classification/frozen_inference_graph.pb')
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(model, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        # self.session = tf.Session(config=config, graph=self.detection_graph)

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(config=config, graph=self.detection_graph)
        rospy.loginfo("Model recreated to run for TL detection")

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # TODO implement light color prediction
        with self.detection_graph.as_default():
            image_expanded = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num) = self.sess.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d],
                                                          feed_dict={self.image_tensor: image_expanded})
            # rospy.logwarn(scores[0][0])

        if int(num) == 0:
            return TrafficLight.UNKNOWN
        else:
            valid = []
            for i in range(int(num)):
                if int(classes[0][i]) == self.COCO_TL_NUM and scores[0][i] > self.DETECTION_THRESHOLD:
                    valid.append(i)
            if len(valid) == 0:
                return TrafficLight.UNKNOWN

            h, w = image.shape[0], image.shape[1]
            # As the scores are already sorted, we take the first box from the valid boxes
            ymin = int(h * boxes[0][valid[0]][0])  # ymin
            xmin = int(w * boxes[0][valid[0]][1])  # xmin
            ymax = int(h * boxes[0][valid[0]][2])  # ymax
            xmax = int(w * boxes[0][valid[0]][3])  # xmax

            # To figure out which area of traffic light is brighter i.e. lighted ON
            cropped_image = image[ymin:ymax, xmin:xmax]

            return self.red_yellow_green(cropped_image)

        # return TrafficLight.UNKNOWN

    def red_yellow_green(self, rgb_image):
        h, w = rgb_image.shape[0], rgb_image.shape[1]

        if h > w:  # if traffic lights are vertical (topmost red)
            one_half_rgb_image = rgb_image[0:int(h / 3), :]  # red area
            two_half_rgb_image = rgb_image[int(h / 3):int(2 * h / 3), :]  # yellow area
            three_half_rgb_image = rgb_image[int(2 * h / 3):h, :]  # green area
        else:  # if traffic lights are horizontal like in Japan (rightmost red)
            one_half_rgb_image = rgb_image[:, int(2 * w / 3):w]  # red area
            two_half_rgb_image = rgb_image[:, int(w / 3):int(2 * w / 3)]  # yellow area
            three_half_rgb_image = rgb_image[:, 0:int(w / 3)]  # green area

        hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        sum_bright_full = float(np.sum(hsv[:, :, 2]))  # float casting is just for ratio calculation
        hsv_one_half = cv2.cvtColor(one_half_rgb_image, cv2.COLOR_RGB2HSV)
        hsv_two_half = cv2.cvtColor(two_half_rgb_image, cv2.COLOR_RGB2HSV)
        hsv_three_half = cv2.cvtColor(three_half_rgb_image, cv2.COLOR_RGB2HSV)
        ratio_red = np.sum(hsv_one_half[:, :, 2]) / sum_bright_full
        ratio_yellow = np.sum(hsv_two_half[:, :, 2]) / sum_bright_full
        ratio_green = np.sum(hsv_three_half[:, :, 2]) / sum_bright_full
        rospy.logwarn(
            '0(Red): ' + str(ratio_red) + ', 1(Yellow):' + str(ratio_yellow) + ', 2(Green):' + str(ratio_green))
        # state --> TrafficLight.UNKNOWN=4 , GREEN=2, YELLOW=1, RED=0

        # If the lights are off then it is expected that brightness is similar in all three halfs.
        # But if one of them is highlighted and then experimentally I found that it is always >35% of brigthness vs. overall.
        if ratio_red >= ratio_yellow and ratio_red >= ratio_green and ratio_red >= 0.35:
            return TrafficLight.RED  # Red
        elif ratio_green >= ratio_yellow and ratio_green >= 0.35:
            return TrafficLight.GREEN  # Yellow
        elif ratio_yellow >= 0.35:
            return TrafficLight.YELLOW  # Green
        else:
            return TrafficLight.UNKNOWN  # traffic light switched off / could be considered green as well :)