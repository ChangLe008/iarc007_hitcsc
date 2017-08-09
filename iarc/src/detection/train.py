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
import argparse

import tf_class
from tf_utils import *

if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Training mode')
	parser.add_argument('--new','-n', action='store', default='n', help='y for building a new model')
	args = parser.parse_args()

	with tf.Session() as sess:

		MAX_STEPS = 40000

		model = tf_class.IARC_FCN()

		#summary_dir = 'E:\\iarc\\sum'
		#train_writer = tf.summary.FileWriter(summary_dir + '\\train', sess.graph)
		#valid_writer = tf.summary.FileWriter(summary_dir + '\\valid', sess.graph)

		init = tf.global_variables_initializer()
		sess.run(init)

		saver = tf.train.Saver()
		save_path = 'E:\\iarc\\tf_code\\model\\model.ckpt'
		print('Param path:')
		print(save_path)

		if args.new == 'n':
			saver.restore(sess, save_path)
			print('Finished reloading param.')

		else:
			print('Building a new model.')
			print('The former params will be dumped.')

		t_all = 0
		[train_image, train_label] = input_data(model.num_classes)
		#train_image = train_image[120:-1, 0:, 0:, 0:]
		#train_label = train_label[120:-1, 0:, 0:]
		data_num = train_image.shape[0]
		list_x = list(range(data_num))
		for i in range(MAX_STEPS):
			if i%data_num == 0:
				random.shuffle(list_x)
				list_c = 0

			'''
			t_i = train_image[list_x[i%data_num], 0:, 0:, 0:]
			t_l = train_label[list_x[i%data_num], 0:, 0:]
			t_i = np.concatenate((t_i,t_i), axis=0)
			t_l = np.concatenate((t_l,t_l), axis=0)
			t_i = t_i.reshape(2, 480, 640, 3)
			t_l = t_l.reshape(2, 480, 640)
			'''
			list_start = list_c%data_num
			if (list_c+50)%data_num == 0:
				list_end = data_num
			else:
				list_end = (list_c+50)%data_num
			#print(list_start, list_end)
			#t_i = train_image[list_x[list_start:list_end], 0:, 0:, 0:]
			#t_l = train_label[list_x[list_start:list_end], 0:, 0:]
			#t_i = t_i.reshape(50, 480, 640, 3)
			#t_l = t_l.reshape(50, 480, 640)
			list_c += 50


			time1 = time.time()
			if i > 1000:
				model.learning_rate = 1e-4
			for j in range(4):
				t_i = train_image[list_x[j*100:(j+1)*100], 0:, 0:, 0:]
				t_l = train_label[list_x[j*100:(j+1)*100], 0:, 0:]
				t_i = t_i.reshape(100, 480, 640, 3)
				t_l = t_l.reshape(100, 480, 640)
				for loop in range(1):
					sess.run(model.train_op, feed_dict = {model.x:t_i,model.y:t_l})

			t_all += time.time() - time1

			if i%10 == 9:
				#t_i = t_i[0, 0:, 0:, 0:]
				#t_l = t_l[0, 0:, 0:]
				#t_i = t_i.reshape(1, 480, 640, 3)
				#t_l = t_l.reshape(1, 480, 640)
				cross_entropy = 0
				for j in range(4):
					t_i = train_image[j*100:(j+1)*100, 0:, 0:, 0:]
					t_l = train_label[j*100:(j+1)*100, 0:, 0:]
					t_i = t_i.reshape(100, 480, 640, 3)
					t_l = t_l.reshape(100, 480, 640)
					cross_entropy += sess.run(model.cross_entropy, feed_dict = {model.x:t_i,model.y:t_l})
				print(i%100+1, cross_entropy/400, i+1)
				f = open('loss.txt','a+')
				wr = str(i) + ' ' + str(cross_entropy) + '\r\n'
				f.write(wr)
				f.close()
				#valid_writer.add_summary(summary, i)
				#print(i%2000+1, sess.run(model.cross_entropy, feed_dict = {model.x:train_image,model.y:train_label}), i+1, list_x[i%data_num])
			if i % 100 == 99:
				#print(t_all)
				#print(2000*4/t_all)
				save_path = saver.save(sess, save_path)
				t_all = 0