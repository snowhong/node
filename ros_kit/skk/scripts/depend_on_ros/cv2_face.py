import cv2
import numpy
import pprint
import random
import math

def training_and_test_data(file):
    lines = file.readlines()
    training_data, testing_data = split_test_training_data()

if __name__ == "__main__":
    csv = open('faces.csv', 'r')
