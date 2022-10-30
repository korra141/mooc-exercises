#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def get_steer_matrix_left_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_left_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                    using the masked left lane markings (numpy.ndarray)
    """
    
    steer_matrix_left_lane = np.zeros(shape)
    
    A2 = np.zeros((int(shape[0]),int(shape[1]/2)))  
    l2 = np.tril_indices(int(shape[0]), k=-4, m=int(shape[1]/2))
    val2 = -2.5
    A2[l2]=val2
    val2 = -2.5
    
    val = -1
    A = np.zeros( (int(3*shape[0]/4),int(shape[1]/2) ))   
     
    l = np.tril_indices(int(3*shape[0]/4), k=-4, m=int(shape[1]/2))
    A[l] = val
     
    steer_matrix_left_lane[:int(3*shape[0]/4), :int(shape[1]/2)] = np.fliplr(A)
    steer_matrix_left_lane[int(3*shape[0]/4):, :int(shape[1]/2)] = val
    
    steer_matrix_left_lane[:,:int(shape[1]/2)] += A2[l2]
    
    steer_matrix_left_lane[0:4, int(shape[1]/2):] = -4
    

    return steer_matrix_left_lane

# In[ ]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK


def get_steer_matrix_right_lane_markings(shape):
    """
        Args:
            shape: The shape of the steer matrix (tuple of ints)
        Return:
            steer_matrix_right_lane: The steering (angular rate) matrix for Braitenberg-like control 
                                     using the masked right lane markings (numpy.ndarray)
    """
    
    steer_matrix_right_lane = np.zeros(shape) 
    A = np.zeros((int(3*shape[0]/4),int(shape[1]/2)))
    val2 = 2
    A2 = np.zeros((int(shape[0]),int(shape[1]/2)))  
    l2 = np.tril_indices(int(shape[0]), k=-4, m=int(shape[1]/2))
    A2[l2]=val2
  
    
    val = 1
    
    
    l = np.tril_indices(int(3*shape[0]/4), k=-4, m=int(shape[1]/2))
    l2 = np.tril_indices(int(shape[0]), k=-4, m=int(shape[1]/2))
    A[l] = val
    A2[l2]=val2
    
    steer_matrix_right_lane[:int(3*shape[0]/4),int(shape[1]/2):]   = A
    steer_matrix_right_lane[int(3*shape[0]/4):,int(shape[1]/2):]  = val
    
    steer_matrix_right_lane[:,int(shape[1]/2):] += A2[l2]
    
    steer_matrix_right_lane[0:4,:int(shape[1]/2)] = 3

    return steer_matrix_right_lane

# In[ ]:


# The function written in this cell will actually be ran on your robot (sim or real). 
# Put together the steps above and write your DeltaPhi function! 
# DO NOT CHANGE THE NAME OF THIS FUNCTION, INPUTS OR OUTPUTS, OR THINGS WILL BREAK

import cv2
import numpy as np


def detect_lane_markings(image):
    """
        Args:
            image: An image from the robot's camera in the BGR color space (numpy.ndarray)
        Return:
            left_masked_img:   Masked image for the dashed-yellow line (numpy.ndarray)
            right_masked_img:  Masked image for the solid-white line (numpy.ndarray)
    """
    
    h, w, _ = image.shape
    
    imgrgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
# Convert the image to HSV for any color-based filtering
    imghsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# Most of our operations will be performed on the grayscale version
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    
    #PARAMS
    
    sigma = 2
    
    threshold = 95
    
    white_lower_hsv = np.array([0, 0, 100])         # CHANGE ME
    white_upper_hsv = np.array([200,90,255])   # CHANGE ME
    yellow_lower_hsv = np.array([20, 85, 85])        # CHANGE ME
    yellow_upper_hsv = np.array([30, 255, 255])
    
    #DEFINE MASKS
    
    # Smooth the image using a Gaussian kernel
    img_gaussian_filter = cv2.GaussianBlur(img,(0,0), sigma)

    sobelx = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,1,0)
    sobely = cv2.Sobel(img_gaussian_filter,cv2.CV_64F,0,1)

    # Compute the magnitude of the gradients
    Gmag = np.sqrt(sobelx*sobelx + sobely*sobely)
    
    mask_mag = (Gmag > threshold)
    
    mask_sobelx_pos = (sobelx > 0)
    mask_sobelx_neg = (sobelx < 0)
    mask_sobely_pos = (sobely > 0)
    mask_sobely_neg = (sobely < 0)
    

    mask_white = cv2.inRange(imghsv, white_lower_hsv, white_upper_hsv)
    mask_yellow = cv2.inRange(imghsv, yellow_lower_hsv, yellow_upper_hsv)

    mask_left = np.ones(sobelx.shape)
    mask_left[:,int(np.floor(w/3)):w + 1] = 0
    mask_right = np.ones(sobelx.shape)
    mask_right[:,0:int(np.floor(2*w/3))] = 0
    
    
    mask_left_edge =  mask_left * mask_mag * mask_sobelx_neg * mask_sobely_neg * mask_yellow
    mask_right_edge = mask_right * mask_mag * mask_sobelx_pos * mask_sobely_neg * mask_white
    
    return (mask_left_edge, mask_right_edge)
