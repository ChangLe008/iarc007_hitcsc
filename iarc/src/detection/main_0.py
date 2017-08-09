# encoding:utf-8
#
# Written by Paral
#
# Started at 2017.02.05


import numpy as np
import tensorflow as tf

import time
import random

import cv2
from PIL import Image

import tf_class
from tf_utils import *

with tf.device('/cpu:0'):
	with tf.Session() as sess:

		model = tf_class.IARC_FCN()

		init = tf.global_variables_initializer()
		sess.run(init)

		saver = tf.train.Saver()
		save_path = '/home/hitcsc/catkin_ws/src/iarc/src/detection/model/model.ckpt'
		print('Param path:')
		print(save_path)

		saver.restore(sess, save_path)
		print('Finished reloading param.')


		#prediction_run(sess, model, 'E:\\iarc\\pic\\2\\', num = 793, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\3\\', num = 891, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\4\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\6\\', num = 1798, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\7\\', num = 1798, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\8\\', num = 541, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\9\\', num = 1799, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\10\\', num = 1444, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\11\\', num = 3902, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\12\\', num = 4757, save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\13\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\14\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\15\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\16\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\17\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\18\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\19\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\28\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\29\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\30\\', save_ans = 0, pic_mode = 'jpg')
		prediction_run(sess, model, 'E:\\iarc\\pic\\31\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\32\\', save_ans = 0, pic_mode = 'jpg')
		#prediction_run(sess, model, 'E:\\iarc\\pic\\Contest1\\Contest', num = 13762, save_ans = 1, pic_mode = 'jpg')
