import cv2
import numpy as np
import math
from styx_msgs.msg import TrafficLight


YELLOW = 'yellow'
GREEN = 'green'
RED = 'red'

def standardize_input(image_in):
    ## DONE: Resize image and pre-process so that all "standard" images are the same size
    standard_im = np.copy(image_in)
    standard_im_resized = cv2.resize(standard_im, (32, 32))
    return standard_im_resized


label_to_one_hot_encode_dic = {'red': [1, 0, 0], 'yellow': [0, 1, 0], 'green': [0, 0, 1]}


def one_hot_encode(label_in):
    ## DONE: Create a one-hot encoded label that works for all classes of traffic lights
    one_hot_encoded = []
    one_hot_encoded = label_to_one_hot_encode_dic[label_in]
    return one_hot_encoded


def standardize(image_list_in):
    # Empty image data array
    standard_list = []

    # Iterate through all the image-label pairs
    for item in image_list_in:
        image = item[0]
        label = item[1]

        # Standardize the image
        standardized_im = standardize_input(image)

        # One-hot encode the label
        one_hot_label = one_hot_encode(label)

        # Append the image, and it's one hot encoded label to the full, processed list of image data
        standard_list.append((standardized_im, one_hot_label))

    return standard_list


def count_label(image_list_in, label_in):
    count = 0
    for item in image_list_in:
        label = item[1]
        if label is label_in:
            count = count+1
    return count


def create_feature(rgb_image_in, debug=False, channel_to_select=2):
    ## DONE: Convert image to HSV color space
    feature = []
    hsv_f = cv2.cvtColor(rgb_image_in, cv2.COLOR_RGB2HSV)

    ## DONE: Create and return a feature value and/or vector

    # HSV channels
    # h = hsv[:,:,0]
    # s = hsv[:,:,1]
    v_f = hsv_f[:, :, channel_to_select]

    v_thresh_min = 45
    v_thresh_max = 255
    # v[(v >= v_thresh_min) & (v <= v_thresh_max)] = 1
    v_f[v_f <= v_thresh_min] = 0

    # crop the image (mainly from left and right)
    v_crop_value = 3
    h_crop_value = 5

    v_cropped = np.copy(v_f)
    v_cropped[:v_crop_value, :] = 0
    v_cropped[32 - v_crop_value:, :] = 0
    v_cropped[:, :h_crop_value] = 0
    v_cropped[:, 32 - h_crop_value:] = 0

    # break the image horizontally into 3 parts
    # such that, 3 lights are in 3 different zones
    # return sum of value as a feature
    top_v = v_cropped[v_crop_value:v_crop_value + 9, :]
    middle_v = v_cropped[v_crop_value + 9:v_crop_value + 18, :]
    bottom_v = v_cropped[v_crop_value + 18:32 - v_crop_value, :]

    feature_top = top_v.ravel().sum()
    feature_middle = middle_v.ravel().sum()
    feature_bottom = bottom_v.ravel().sum()
    feature = [feature_top, feature_middle, feature_bottom]
    if debug:
        f, axis_arr = plt.subplots(2, 3)
        axis_arr[0][0].imshow(rgb_image_in)
        axis_arr[0][0].set_title("Original")

        axis_arr[0][1].imshow(v, cmap='gray')
        axis_arr[0][1].set_title("Selected Channel")

        axis_arr[0][2].imshow(v_cropped, cmap='gray')
        axis_arr[0][2].set_title("Cropped")

        axis_arr[1][0].imshow(top_v, cmap='gray')
        axis_arr[1][0].set_title("Top")

        axis_arr[1][1].imshow(middle_v, cmap='gray')
        axis_arr[1][1].set_title("Middle")

        axis_arr[1][2].imshow(bottom_v, cmap='gray')
        axis_arr[1][2].set_title("Bottom")
        print(feature)
    return feature


def second_largest(numbers):
    count = 0
    m1 = m2 = -math.inf
    for x in numbers:
        count += 1
        if x > m2:
            if x >= m1:
                m1, m2 = x, m1
            else:
                m2 = x
    return m2 if count >= 2 else None


def confidence_of_channel(feature):
    max_feature = max(feature)
    second_feature = second_largest(feature)
    sum_features = sum(feature)
    diff_percent = 0
    if sum_features is not 0:
        diff_percent = (max_feature - second_feature) / sum_features
    return diff_percent


# (Optional) Add more image analysis and create more features
feature_index_to_label_dic = {0: 'red', 1: 'yellow', 2: 'green'}


## Takes multiple channel and sums the feature vector
def feature_aggregator_v2(rbg_image_in, debug=False, selected_channels=[1, 2]):
    feature = [0, 0, 0]
    sum_features = 1
    max_diff_percent = 0
    for selected_channel in selected_channels:
        feature_curr = create_feature(rbg_image_in, debug, selected_channel)
        diff_percent = confidence_of_channel(feature)

        # print( max_feature , second_feature, sum_features, diff_percent)
        if diff_percent > 0.9:
            feature = feature_curr
            break
        else:
            feature[0] += feature_curr[0]
            feature[1] += feature_curr[1]
            feature[2] += feature_curr[2]

    # feature = create_feature(rbg_image_in, debug, selected_channel)
    label_str = feature_index_to_label_dic[feature.index(max(feature))]
    if debug:
        print(feature)

    return label_str


def feature_aggregator(rbg_image_in, debug=False, selected_channels=[1, 2]):
    feature = [0, 0, 0]
    for selected_channel in selected_channels:
        feature_curr = create_feature(rbg_image_in, debug, selected_channel)

        feature[0] += feature_curr[0]
        feature[1] += feature_curr[1]
        feature[2] += feature_curr[2]

    label_str = feature_index_to_label_dic[feature.index(max(feature))]
    if debug:
        print(feature)
    return label_str

class CVClassifier:
    def get_classification(rgb_image_in, debug=False, selected_channels=[1, 2]):
        ## DONE: Extract feature(s) from the RGB image and use those features to
        ## classify the image and output a one-hot encoded label
        predicted_label_str = feature_aggregator(rgb_image_in, debug, selected_channels)
        predicted_label = one_hot_encode(predicted_label_str)
        if (debug):
            print(predicted_label_str)
            print(predicted_label)
        if class_name == RED:
            self.current_light = TrafficLight.RED
        elif class_name == GREEN:
            self.current_light = TrafficLight.GREEN
        elif class_name == YELLOW:
            self.current_light = TrafficLight.YELLOW
        self.image_np_deep = image
        return predicted_label