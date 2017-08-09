# encoding:utf-8
#
# Written by Paral
#
# Started at 2017.02.05

import tensorflow as tf
import numpy as np
from math import *

class IARC_FCN:

	def __init__(self):
		self.description = 'FCN-8s'
		self.num_classes = 2
		self.x = tf.placeholder(tf.float32, shape = (None,480,640,3))
		self.y = tf.placeholder(tf.int32, shape = (None,480,640))
		self.learning_rate = 1e-3

		self.global_fcn_build()

		print('Finished init.')
		


	def global_fcn_build(self):
		print('Building model.')

		self.batch_num = tf.shape(self.x)[0]

		self.poool = self._max_pool_layer(self.x, 'fcn_poool')
		#self.pooool = self._max_pool_layer(self.poool, 'fcn_pooool')
		#self.pooool, self.pooool_relu = self._conv_layer(self.poool, 3, 8, 'fcn_pooool', kernel_size=7, strides_size=2)
		self.pooool = self._max_pool_layer(self.poool, 'fcn_pooool')

		self.conv3, self.relu3 = self._conv_layer(self.pooool, 3, 8, 'fcn_conv3_2')
		#self.batch3 = self._batch_normalization_layer(self.conv3, 'fcn_batch3')
		#self.pool3, self.pool_relu3 = self._conv_layer(self.batch3, 8, 8, 'fcn_pool3', kernel_size=2, strides_size=2)
		self.pool3 = self._max_pool_layer(self.conv3, 'fcn_pool3')

		self.conv4, self.relu4 = self._conv_layer(self.pool3, 8, 8, 'fcn_conv4_2')
		#self.batch4 = self._batch_normalization_layer(self.conv4, 'fcn_batch4')
		#self.pool4, self.pool_relu4 = self._conv_layer(self.batch4, 8, 8, 'fcn_pool4', kernel_size=2, strides_size=2)
		self.pool4 = self._max_pool_layer(self.conv4, 'fcn_pool4')

		self.conv5, self.relu5 = self._conv_layer(self.pool4, 8, 16, 'fcn_conv5_2')
		#self.batch5 = self._batch_normalization_layer(self.conv5, 'fcn_batch5')
		#self.pool5, self.pool_relu5 = self._conv_layer(self.batch5, 8, 16, 'fcn_pool5', kernel_size=2, strides_size=2)
		self.pool5 = self._max_pool_layer(self.conv5, 'fcn_pool5')

		self.conv_add, self.relu_add = self._conv_layer(self.pool5, 16, 32, 'fcn_conv_add')
		#self.batch_add = self._batch_normalization_layer(self.conv_add, 'fcn_batch_add')

		self.downsample6 = self._downsampling_layer(self.conv_add, 32, 32, 'fcn_downsampling6')
		self.dropout6 = self._dropout_layer(self.downsample6, 'fcn_dropout6')

		self.fc7 = self._fc_layer(self.dropout6, 32, 64, 'fc7')
		self.dropout7 = self._dropout_layer(self.fc7, 'fcn_dropout7')

		self.fc8 = self._fc_layer(self.dropout7, 64, 128, 'fcn_fc8')
		self.dropout8 = self._dropout_layer(self.fc8, 'fcn_dropout8')

		self.score = self._score_layer(self.dropout8, 128, 'fcn_score32')

		self.upscore16 = self._deconv_layer(self.score, 30, 40, 2, 'fcn_upscore16')

		self.score16 = self._score_layer(self.pool4, 8, 'fcn_score16')
		self.add16 = self._add_layer(self.upscore16, self.score16, 'fcn_add16')
		
		self.upscore8 = self._deconv_layer(self.add16, 60, 80, 2, 'fcn_upscore8')

		self.score8 = self._score_layer(self.pool3, 8, 'fcn_score8')
		self.add8 = self._add_layer(self.upscore8, self.score8, 'fcn_add8')

		self.out_image = self._deconv_layer(self.add8, 120, 160, 2, 'fcn_out')

		self.ans = tf.nn.softmax(self.out_image, name = 'softmax')
		#tf.contrib.deprecated.histogram_summary('softmax', self.ans)

		self.y_p = self._avg_pool_layer(tf.reshape(tf.to_float(self.y, name='ToFloat'), [self.batch_num,480,640,1]), 'fcn_pool_y', i_ksize=[1,4,4,1], i_strides=[1,4,4,1])
		self.y_p = tf.to_int32(tf.reshape(tf.round(self.y_p), [self.batch_num,120,160]))

		self.logits = self.logit(self.y_p, self.out_image)

		self.cross_entropy = self.loss(self.logits)
		self.train_op = self.train(self.cross_entropy, self.learning_rate)

		self.ans_slice = tf.slice(self.ans, [0, 0, 0, 1], [1, 120, 160, self.num_classes-1])

		#self.ans_filt1, self.ans_filt_relu1 = self._filt_layer(self.ans_slice, 'ans_filt1', 1)
		self.ans_filt, self.ans_filt_relu = self._filt_layer(self.ans_slice, 'ans_filt2', 2)

		#self.merged = tf.contrib.deprecated.merge_all_summaries()

		print('Finished Building Network.')


	def _conv_layer(self, bottom, in_channels, out_channels, name, kernel_size=3, strides_size=1):
		with tf.name_scope(name) as scope:
			weights = tf.Variable(tf.truncated_normal([kernel_size,kernel_size,in_channels,out_channels],stddev = 0.1), name = 'filter_weights')
			biases = tf.Variable(tf.zeros([out_channels]), name = 'conv_biases')
			conv = tf.nn.convolution(bottom, weights, strides = [strides_size,strides_size], padding = 'SAME') + biases
			relu = tf.nn.relu(conv, name='relu')

		return conv, relu


	def _batch_normalization_layer(self, bottom, name, relu=False, is_training=False):
		with tf.name_scope(name) as scope:
			batch_norm = tf.contrib.layers.batch_norm(bottom,scale=True,center=True,is_training=is_training,scope=name)

		if relu:
			batch_norm = tf.nn.relu(batch_norm, name='relu')

		return batch_norm


	def _max_pool_layer(self, bottom, name, i_ksize=[1,2,2,1], i_strides=[1,2,2,1]):
		with tf.name_scope(name) as scope:
			pool = tf.nn.max_pool(bottom, ksize = i_ksize, strides = i_strides, padding = 'SAME', name = 'max_pooling')

		return pool


	def _avg_pool_layer(self, bottom, name, i_ksize=[1,2,2,1], i_strides=[1,2,2,1]):
		with tf.name_scope(name) as scope:
			pool = tf.nn.avg_pool(bottom, ksize = i_ksize, strides = i_strides, padding = 'SAME', name = 'max_pooling')

		return pool


	def _fc_layer(self, bottom, in_channels, out_channels, name):
		with tf.name_scope(name) as scope:
			weights = tf.Variable(tf.truncated_normal([1,1,in_channels,out_channels],stddev = 0.1), name = 'filter_weights')
			biases = tf.Variable(tf.zeros([out_channels]), name = 'fc_biases')
			fc = tf.nn.relu(tf.nn.convolution(bottom, weights, padding = 'SAME', strides = [1,1]) + biases, name = 'relu')

		return fc


	def _downsampling_layer(self, bottom, in_channels, out_channels, name):
		with tf.name_scope(name) as scope:
			weights = tf.Variable(tf.truncated_normal([5,5,in_channels,out_channels],stddev = 0.1), name = 'filter_weights')
			biases = tf.Variable(tf.zeros([out_channels]), name = 'downsample_biases')
			downsample = tf.nn.relu(tf.nn.convolution(bottom, weights, padding = 'SAME', strides = [1,1]) + biases, name = 'relu')

		return downsample


	def _score_layer(self, bottom, in_channels, name):
		with tf.name_scope(name) as scope:
			weights = tf.Variable(tf.truncated_normal([1,1,in_channels,self.num_classes],stddev = 0.1), name = 'filter_weights')
			biases = tf.Variable(tf.zeros([self.num_classes]), name = 'score_biases')
			score = tf.nn.convolution(bottom, weights, padding = 'SAME', strides = [1,1]) + biases

		return score


	def _add_layer(self, bottom_1, bottom_2, name):
		with tf.name_scope(name) as scope:
			add = tf.add(bottom_1, bottom_2)

		return add


	def _deconv_layer(self, bottom, out_height, out_width, strides_xy, name, batch_num=None):
		if batch_num is None:
			batch_num = self.batch_num

		with tf.name_scope(name) as scope:
			weights = self.get_deconv_filter(name, f_shape=[strides_xy*2,strides_xy*2,self.num_classes,self.num_classes])
			upscore = tf.nn.conv2d_transpose(bottom, weights, [batch_num, out_height, out_width, self.num_classes], [1,strides_xy,strides_xy,1], padding='SAME', data_format='NHWC', name=None)

		return upscore


	def _dropout_layer(self, bottom, name):
		with tf.name_scope(name) as scope:
			dropout = tf.nn.dropout(bottom, 0.5)

		return dropout


	def get_deconv_filter(self, f_name, f_shape):
		width = f_shape[0]
		heigh = f_shape[0]
		f = ceil(width/2.0)
		c = (2 * f - 1 - f % 2) / (2.0 * f)
		bilinear = np.zeros([f_shape[0], f_shape[1]])
		for x in range(width):
			for y in range(heigh):
				value = (1 - abs(x / f - c)) * (1 - abs(y / f - c))
				bilinear[x, y] = value
		weights = np.zeros(f_shape)
		for i in range(f_shape[2]):
			weights[:, :, i, i] = bilinear
		#print(bilinear)
		init = tf.constant_initializer(value=weights, dtype=tf.float32)
		var = tf.get_variable(name = f_name, initializer=init, shape=weights.shape)

		return var


	def logit(self, y, prediction):
		with tf.name_scope('logits'):
			logits = tf.nn.sparse_softmax_cross_entropy_with_logits(labels = y, logits = prediction, name = "sparse_softmax_cross_entropy")

		return logits


	def loss(self, logits):
		with tf.name_scope('loss'):
			cross_entropy = tf.reduce_sum(logits)
		
		return cross_entropy


	def train(self, loss, learning_rate):
		with tf.name_scope('train_op'):
			optimizer = tf.train.AdamOptimizer(learning_rate)
			train_op = optimizer.minimize(loss)
		
		return train_op


	def _filt_layer(self, bottom, name, type, batch_num=None):
		if batch_num is None:
			batch_num = self.batch_num

		with tf.name_scope(name) as scope:
			if type == 3:
				weights = self.get_filt_filter(name, [5,5,self.num_classes,self.num_classes], type)
			else:
				weights = self.get_filt_filter(name, [5,5,self.num_classes-1,self.num_classes-1], type)
			init = tf.constant_initializer(value=0, dtype=tf.float32)
			#biases = tf.Variable(tf.zeros([1]), name = 'conv_biases')
			if name[0] == 'l':
				biases = tf.get_variable(initializer=init, shape=[self.num_classes-1], name='local_biases'+str(type))
			else:
				biases = tf.get_variable(initializer=init, shape=[1], name='biases'+str(type))
			conv = tf.nn.convolution(bottom, weights, strides = [1,1], padding = 'SAME') + biases
			relu = tf.nn.relu(conv, name='relu')

		return conv, relu

	def get_filt_filter(self, f_name, f_shape, type):
		width = f_shape[0]
		heigh = f_shape[0]
		f = ceil(width/2.0)
		c = (2 * f - 1 - f % 2) / (2.0 * f)
		bilinear = np.zeros([f_shape[0], f_shape[1]])
		for x in range(width):
			for y in range(heigh):
				if type == 1:
					value = (1 - abs(x / f - c)) * (1 - abs(y / f - c))
					bilinear[x, y] = value
				else:
					bilinear[x, y] = 1
		weights = np.zeros(f_shape)
		for i in range(f_shape[2]):
			weights[:, :, i, i] = bilinear
		#print('Cautions!')
		#print(bilinear)
		init = tf.constant_initializer(value=weights, dtype=tf.float32)
		var = tf.get_variable(name=f_name+str(type), initializer=init, shape=weights.shape)

		return var

