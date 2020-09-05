import matplotlib.colors
import matplotlib.pyplot as plt
import numpy as np
from pcl_helper import *


def rgb_to_hsv(rgb_list):
    rgb_normalized = [1.0*rgb_list[0]/255, 1.0*rgb_list[1]/255, 1.0*rgb_list[2]/255]
    hsv_normalized = matplotlib.colors.rgb_to_hsv([[rgb_normalized]])[0][0]
    return hsv_normalized


def compute_color_histograms(cloud, using_hsv=False):

    # Compute histograms for the clusters
    point_colors_list = []

    # Step through each point in the point cloud
    for point in pc2.read_points(cloud, skip_nans=True):
        rgb_list = float_to_rgb(point[3])
        if using_hsv:
            point_colors_list.append(rgb_to_hsv(rgb_list) * 255)
        else:
            point_colors_list.append(rgb_list)

    # Populate lists with color values
    channel_1_vals = []
    channel_2_vals = []
    channel_3_vals = []

    for color in point_colors_list:
        channel_1_vals.append(color[0])
        channel_2_vals.append(color[1])
        channel_3_vals.append(color[2])
    
    # TODO: Compute histograms
    bin_nums = 32
    hist1 = np.histogram(channel_1_vals, bins=bin_nums, range=(0,256))
    hist2 = np.histogram(channel_2_vals, bins=bin_nums, range=(0,256))
    hist3 = np.histogram(channel_3_vals, bins=bin_nums, range=(0,256))

    # TODO: Concatenate and normalize the histograms
    histogram = np.concatenate((hist1[0], hist2[0], hist3[0])).astype(np.float64)	
    hist_norm = histogram/np.sum(histogram)

    ## Generate random features for demo mode.  
    ## Replace normed_features with your feature vector
    ##normed_features = np.random.random(96) 
    return hist_norm#normed_features 


def compute_normal_histograms(normal_cloud):
    norm_x_vals = []
    norm_y_vals = []
    norm_z_vals = []

    for norm_component in pc2.read_points(normal_cloud,
                                          field_names = ('normal_x', 'normal_y', 'normal_z'),
                                          skip_nans=True):
        norm_x_vals.append(norm_component[0])
        norm_y_vals.append(norm_component[1])
        norm_z_vals.append(norm_component[2])

    # TODO: Compute histograms of normal values (just like with color)
    bin_nums = 32
    hist1 = np.histogram(norm_x_vals, bins=bin_nums, range=(0,256))
    hist2 = np.histogram(norm_y_vals, bins=bin_nums, range=(0,256))
    hist3 = np.histogram(norm_z_vals, bins=bin_nums, range=(0,256))

    # TODO: Concatenate and normalize the histograms
    histogram = np.concatenate((hist1[0], hist2[0], hist3[0])).astype(np.float64)	
    hist_norm = histogram/np.sum(histogram)

    ## Generate random features for demo mode.  
    ## Replace normed_features with your feature vector
    ## normed_features = np.random.random(96)

    return hist_norm#normed_features
