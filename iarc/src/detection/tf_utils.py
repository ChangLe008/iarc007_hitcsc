# encoding:utf-8
#
# Written by Paral
#
# Started at 2017.02.05

import numpy as np

import cv2
from PIL import Image

import time
import os

from skimage import data,color,morphology,measure,feature

def input_data(classtype, rootpath_label=r'E:\iarc\tf_code\train_image', rootpath_image=r'E:\iarc\pic'):

	new = 1

	if new:

		childlist_label = os.listdir(rootpath_label)

		for child_dir in childlist_label:
			if child_dir == 'F':
				continue
			if child_dir[0] == '%':
				continue
			childpath_label = os.path.join('%s\%s' % (rootpath_label, child_dir))
			childpath_image = os.path.join('%s\%s' % (rootpath_image, child_dir))
			secchildlist_label = os.listdir(childpath_label)
			for secchild_dir in secchildlist_label:
				secchildpath_label = os.path.join('%s\%s' % (childpath_label, secchild_dir))
				secchildpath_image = os.path.join('%s\%s' % (childpath_image, secchild_dir))

				secchildpath_image = secchildpath_image[0:-3] + 'jpg'

				print(secchildpath_label)

				image = cv2.imread(secchildpath_image)
				image = np.asarray(image)
				image = image.astype('float32')
				#image = image/255
				image = image.reshape(1, 480, 640, 3)

				label = cv2.imread(secchildpath_label)
				label = np.asarray(label)
				label = label.astype('int32')
				label = label[0:, 0:, 0]
				label = label.reshape(1, 480, 640)

				if 'np_label' in dir():
					np_image = np.concatenate((np_image, image), axis = 0)
					np_label = np.concatenate((np_label, label), axis = 0)
				else:
					np_image = image
					np_label = label

		if classtype == 2:
			np_label = np.ceil(np_label/5)
		elif classtype == 3:
			np_label = np.ceil(np_label/2)

		np.save('np_label.npy', np_label)
		np.save('np_image.npy', np_image)

	else:
		np_label = np.load('np_label.npy')
		np_image = np.load('np_image.npy')

	print('Find ', np_label.shape[0], ' images')
	return np_image, np_label


def prediction_run(sess, model, num=1200, save_ans=False, pic_mode='jpg'):
	cap = cv2.VideoCapture(0)
	lower_green = np.array([35, 60, 46])
	upper_green = np.array([77, 255, 255])

	lower_red_1 = np.array([160, 60, 100])
	upper_red_1 = np.array([180, 255, 255])
	lower_red_2 = np.array([0, 60, 100])
	upper_red_2 = np.array([10, 255, 255])

	region_bias = [2, 2, -2, -2]
	bbox = [-1, -1, -1, -1]

	time_all = 0
	time_all2 = 0
	while 1:	
		ret, im = cap.read()
		ret, img = cap.read()		
		im_to_pre = np.asarray(im)
		im_to_pre = im_to_pre.astype('float32')
		im_to_pre = im_to_pre.reshape(1, 480, 640, 3)

		time1 = time.time()
		time2 = time.time()

		#ans, ans_center = sess.run([model.ans,model.ans_filt_relu], feed_dict={model.x:im_to_pre})
		ans_center = sess.run(model.ans_filt_relu, feed_dict={model.x:im_to_pre})
		time_all += time.time() - time1
		ans_center = ans_center[0, 0:, 0:, 0]
		ans_center[ans_center<=20] = 0
		ans_center[ans_center>20] = 255
		ans_center = ans_center.astype('uint8')
		#cv2.imshow('center', ans_center)
		#ans = ans[0, 0:, 0:, 1]
		#cv2.imshow('ans', ans)


		#edgs = feature.canny(ans_center2)#, sigma=3, low_threshold=10, high_threshold=20)
		#chull = morphology.convex_hull_object(edgs)
		#cv2.imshow('chull', chull*1*255)
		#pos = measure.regionprops(chull.astype('uint8')*255)

		label_image = measure.label(ans_center)

		for region in measure.regionprops(label_image):
			bbox[0] = 4 * (region.bbox[0] - 1)
			bbox[1] = 4 * (region.bbox[1] - 1)
			bbox[2] = 4 * (region.bbox[2] + 1) + 3
			bbox[3] = 4 * (region.bbox[3] + 1) + 3

			if bbox[0] < 0:
				bbox[0] = 0
			if bbox[1] < 0:
				bbox[1] = 0
			if bbox[2] > 479:
				bbox[2] = 479
			if bbox[3] > 639:
				bbox[3] = 639
			cv2.rectangle(img, (bbox[1],bbox[0]), (bbox[3],bbox[2]), (255,255,0), 3)
			roi = im[bbox[0] : bbox[2], bbox[1] : bbox[3]]
			#cv2.imshow('roi', roi)
			biases = [bbox[1], bbox[0]]
			biases = np.asarray(biases)
			biases = np.concatenate((biases,biases,biases,biases), axis = 0)
			biases = biases.reshape(4,2)
			#print(biases)

			frame_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
			# Find shades of green
			green = cv2.inRange(frame_hsv, lower_green, upper_green)
			# Blur the threshholded image to make it more accurate
			green_blur = cv2.GaussianBlur(green, (3, 3), 0)
			green_erode = cv2.erode(green_blur, None, iterations = 3)
			green_dilate = cv2.dilate(green_erode, None, iterations = 3)
			green_dilate0 = green_dilate.copy()
			green_dilate1 = green_dilate.copy()
			green_dilate2 = green_dilate.copy()
			
			# Find shades of red
			red_1 = cv2.inRange(frame_hsv, lower_red_1, upper_red_1)
			red_2 = cv2.inRange(frame_hsv, lower_red_2, upper_red_2)
			red = cv2.add(red_1, red_2)
			# Blur the threshholded image to make it more accurate
			red_blur = cv2.GaussianBlur(red, (3, 3), 0)
			red_erode = cv2.erode(red_blur, None, iterations = 3)
			red_dilate = cv2.dilate(red_erode, None, iterations = 3)
			red_dilate0 = red_dilate.copy()
			red_dilate1 = red_dilate.copy()
			red_dilate2 = red_dilate.copy()

			contours_g, hierarchy_g = cv2.findContours(green_dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			contours_r, hierarchy_r = cv2.findContours(red_dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
			if len(contours_g) > 0:
				# Draw each contour
				for index_g in range(len(contours_g)):
					if cv2.contourArea(contours_g[index_g]) > 200:
						rect_g = cv2.minAreaRect(contours_g[index_g])
						((center_x_g, center_y_g), (width_g, height_g), angle_g) = rect_g
						'''
						if (width_g / height_g) < 3.3 and (width_g / height_g) > 0.3:
							box_g = np.int0(cv2.boxPoints(rect_g))
							box_g = box_g + biases
							cv2.drawContours(im, [box_g], -1, (0, 255, 0), 2)
						'''
						if (width_g / height_g) < 3.3 and (width_g / height_g) > 0.3:
							box_g = np.int0(cv2.cv.BoxPoints(rect_g)) 
							#box_g_init = box_g.copy()
							box_g_init = np.int0(cv2.cv.BoxPoints(rect_g))
							#box_g = box_g + biases
							#print box_g[0]
							#print box_g[1],
							#print box_g[2],
							#print box_g[3]
							#dis10 = pow((box_g[1, 0] - box_g[0, 0]), 2) + pow((box_g[1, 1] - box_g[0, 1]),2)
							#dis12 = pow((box_g[1, 0] - box_g[2, 0]), 2) + pow((box_g[1, 1] - box_g[2, 1]),2)
							dis10 = np.absolute(box_g[1, 0] - box_g[0, 0]) + np.absolute(box_g[1, 1] - box_g[0, 1])
							dis12 = np.absolute(box_g[1, 0] - box_g[2, 0]) + np.absolute(box_g[1, 1] - box_g[2, 1])
							px0 = 0
							px1 = 0
							px2 = 0

							if (dis10 < dis12):

								if box_g[1, 0] < 20:
									box_g[1, 0] = 20
								if box_g[1, 0] > bbox[3]-bbox[1]-21:
									box_g[1, 0] = bbox[3]-bbox[1]-21
								if box_g[1, 1] < 20:
									box_g[1, 1] = 20
								if box_g[1, 1] > bbox[2]-bbox[0]-21:
									box_g[1, 1] = bbox[2]-bbox[0]-21
								green_dilate1[box_g[1, 1]-20 : box_g[1, 1]+20, box_g[1, 0]-20 : box_g[1, 0]+20]
								px1 = np.sum(green_dilate1)
								
								if box_g[0, 0] < 20:
									box_g[0, 0] = 20
								if box_g[0, 0] > bbox[3]-bbox[1]-21:
									box_g[0, 0] = bbox[3]-bbox[1]-21
								if box_g[0, 1] < 20:
									box_g[0, 1] = 20
								if box_g[0, 1] > bbox[2]-bbox[0]-21:
									box_g[0, 1] = bbox[2]-bbox[0]-21
								green_dilate0[box_g[0, 1]-20 : box_g[0, 1]+20, box_g[0, 0]-20 : box_g[0, 0]+20]
								px0 = np.sum(green_dilate0)

								if (px1 < px0) :
									direction_x = box_g_init[1, 0] - box_g_init[0, 0]
									direction_y = box_g_init[1, 1] - box_g_init[0, 1]
								else:
									direction_x = box_g_init[0, 0] - box_g_init[1, 0]
									direction_y = box_g_init[0, 1] - box_g_init[1, 1]
								#print"px1 = %d" % px1,
								#print "px0 = %d" % px0
								#print(direction_x)
								#print(direction_y)

							else:
								if box_g[1, 0] < 20:
									box_g[1, 0] = 20
								if box_g[1, 0] > bbox[3]-bbox[1]-21:
									box_g[1, 0] = bbox[3]-bbox[1]-21
								if box_g[1, 1] < 20:
									box_g[1, 1] = 20
								if box_g[1, 1] > bbox[2]-bbox[0]-21:
									box_g[1, 1] = bbox[2]-bbox[0]-21
								green_dilate1[box_g[1, 1]-20 : box_g[1, 1]+20, box_g[1, 0]-20 : box_g[1, 0]+20]
								px1 = np.sum(green_dilate1)
								
								if box_g[2, 0] < 20:
									box_g[2, 0] = 20
								if box_g[2, 0] > bbox[3]-bbox[1]-21:
									box_g[2, 0] = bbox[3]-bbox[1]-21
								if box_g[2, 1] < 20:
									box_g[2, 1] = 20
								if box_g[2, 1] > bbox[2]-bbox[0]-21:
									box_g[2, 1] = bbox[2]-bbox[0]-21
								green_dilate2[box_g[2, 1]-20 : box_g[2, 1]+20, box_g[2, 0]-20 : box_g[2, 0]+20]
								px2 = np.sum(green_dilate2)

								if (px1 < px2) :
									direction_x = box_g_init[1, 0] - box_g_init[2, 0]
									direction_y = box_g_init[1, 1] - box_g_init[2, 1]
								else:
									direction_x = box_g_init[2, 0] - box_g_init[1, 0]
									direction_y = box_g_init[2, 1] - box_g_init[1, 1]
								#print "px1 = %d" % px1,
								#print "px2 = %d" % px2
								#print(direction_x)
								#print(direction_y)
							
							#vector_x = direction_x / np.sqrt(direction_x * direction_x + direction_y * direction_y)
							#vector_y = direction_y / np.sqrt(direction_x * direction_x + direction_y * direction_y)

							vector_x = direction_x
							vector_y = direction_y
							end_x_g = vector_x
							end_y_g = vector_y
							if end_x_g < -320:
								end_x_g = -320
							if end_x_g > 319:
								end_x_g = 319
							if end_y_g < -240:
								end_y_g = -240
							if end_y_g > 239:
								end_y_g = 239
							box_g_init = box_g_init
							cv2.line(im, (320, 240), (320+end_x_g, 240+end_y_g), (0, 255, 0), 2)
							cv2.drawContours(im, [box_g_init], -1, (0, 255, 0), 2) 
			if len(contours_r) > 0:
				# Draw each contour
				for index_r in range(len(contours_r)):
					if cv2.contourArea(contours_r[index_r]) > 200:
						rect_r = cv2.minAreaRect(contours_r[index_r])
						((center_x_r, center_y_r), (width_r, height_r), angle_r) = rect_r
						if (width_r / height_r) < 3.3 and (width_r / height_r) > 0.3:
							box_r = np.int0(cv2.cv.BoxPoints(rect_r))
							box_r_init = box_r.copy()
							#box_r = box_r + biases
							#print(box_r)

							#dis10 = pow((box_r[1, 0] - box_r[0, 0]), 2) + pow((box_r[1, 1] - box_r[0, 1]),2)
							#dis12 = pow((box_r[1, 0] - box_r[2, 0]), 2) + pow((box_r[1, 1] - box_r[2, 1]),2)
							dis10 = np.absolute(box_r[1, 0] - box_r[0, 0]) + np.absolute(box_r[1, 1] - box_r[0, 1])
							dis12 = np.absolute(box_r[1, 0] - box_r[2, 0]) + np.absolute(box_r[1, 1] - box_r[2, 1])
							px0 = 0
							px1 = 0
							px2 = 0

							if (dis10 < dis12):

								if box_r[1, 0] < 20:
									box_r[1, 0] = 20
								if box_r[1, 0] > bbox[3]-bbox[1]-21:
									box_r[1, 0] = bbox[3]-bbox[1]-21
								if box_r[1, 1] < 20:
									box_r[1, 1] = 20
								if box_r[1, 1] > bbox[2]-bbox[0]-21:
									box_r[1, 1] = bbox[2]-bbox[0]-21
								#print(red_dilate)
								red_dilate1[box_r[1, 1]-20 : box_r[1, 1]+20, box_r[1, 0]-20 : box_r[1, 0]+20]
								px1 = np.sum(red_dilate1)
								
								if box_r[0, 0] < 20:
									box_r[0, 0] = 20
								if box_r[0, 0] > bbox[3]-bbox[1]-21:
									box_r[0, 0] = bbox[3]-bbox[1]-21
								if box_r[0, 1] < 20:
									box_r[0, 1] = 20
								if box_r[0, 1] > bbox[2]-bbox[0]-21:
									box_r[0, 1] = bbox[2]-bbox[0]-21
								red_dilate0[box_r[0, 1]-20 : box_r[0, 1]+20, box_r[0, 0]-20 : box_r[0, 0]+20]
								px0 = np.sum(red_dilate0)

								if (px1 < px0) :
									direction_x = box_r_init[1, 0] - box_r_init[0, 0]
									direction_y = box_r_init[1, 1] - box_r_init[0, 1]
								else:
									direction_x = box_r_init[0, 0] - box_r_init[1, 0]
									direction_y = box_r_init[0, 1] - box_r_init[1, 1]
								#print"px1 = %d" % px1,
								#print "px0 = %d" % px0
								#print(direction_x)
								#print(direction_y)

							else:
								if box_r[1, 0] < 20:
									box_r[1, 0] = 20
								if box_r[1, 0] > bbox[3]-bbox[1]-21:
									box_r[1, 0] = bbox[3]-bbox[1]-21
								if box_r[1, 1] < 20:
									box_r[1, 1] = 20
								if box_r[1, 1] > bbox[2]-bbox[0]-21:
									box_r[1, 1] = bbox[2]-bbox[0]-21
								red_dilate1[box_r[1, 1]-20 : box_r[1, 1]+20, box_r[1, 0]-20 : box_r[1, 0]+20]
								px1 = np.sum(red_dilate1)
								
								if box_r[2, 0] < 20:
									box_r[2, 0] = 20
								if box_r[2, 0] > bbox[3]-bbox[1]-21:
									box_r[2, 0] = bbox[3]-bbox[1]-21
								if box_r[2, 1] < 20:
									box_r[2, 1] = 20
								if box_r[2, 1] > bbox[2]-bbox[0]-21:
									box_r[2, 1] = bbox[2]-bbox[0]-21
								red_dilate2[box_r[2, 1]-20 : box_r[2, 1]+20, box_r[2, 0]-20 : box_r[2, 0]+20]
								px2 = np.sum(red_dilate2)

								if (px1 < px2) :
									direction_x = box_r_init[1, 0] - box_r_init[2, 0]
									direction_y = box_r_init[1, 1] - box_r_init[2, 1]
								else:
									direction_x = box_r_init[2, 0] - box_r_init[1, 0]
									direction_y = box_r_init[2, 1] - box_r_init[1, 1]
								#print "px1 = %d" % px1,
								#print "px2 = %d" % px2
								#print(direction_x)
								#print(direction_y)
							
							#vector_x = direction_x / np.sqrt(direction_x * direction_x + direction_y * direction_y)
							#vector_y = direction_y / np.sqrt(direction_x * direction_x + direction_y * direction_y)

							vector_x = direction_x
							vector_y = direction_y
							end_x_r = vector_x
							end_y_r = vector_y
							if end_x_r < -320:
								end_x_r = -320
							if end_x_r > 319:
								end_x_r = 319
							if end_y_r < -240:
								end_y_r = -240
							if end_y_r > 239:
								end_y_r = 239
							#box_r_init = box_r_init + biases
							cv2.line(im, (320, 240), (320+end_x_r, 240+end_y_r), (0, 0, 255), 2)
							cv2.drawContours(im, [box_r_init], -1, (0, 0, 255), 2) 

			if len(contours_g) == 0 and len(contours_r) == 0 and region.area > 5:
				if (region.bbox[3]-region.bbox[1])/(region.bbox[2]-region.bbox[0]) > 1/2.5 and (region.bbox[3]-region.bbox[1])/(region.bbox[2]-region.bbox[0]) < 2.5:
					if region.bbox[3] < 158 and region.bbox[2] < 118 and region.bbox[1] > 1 and region.bbox[0] > 1:
						cv2.rectangle(im, (4*region.bbox[1],4*region.bbox[0]), (4*region.bbox[3]+3,4*region.bbox[2]+3), (0,0,0), 3)
			
			
		cv2.imshow('im', im)

		time_all2 += time.time() - time2
		cv2.waitKey(1)
		#if save_ans:
		#	save_path_im = 'E:\\iarc\\pic_relabel\\' + path[-3:-1] + '\\im' + str(i+1) + '.png'
		#	save_path_img = 'E:\\iarc\\pic_relabel\\' + path[-3:-1] + '\\img' + str(i+1) + '.png'
		#	print(save_path_im)
		#	cv2.imwrite(save_path_im, im)
		#	cv2.imwrite(save_path_img, img)
	print(time_all)
	print(time_all2)
	print(num/time_all)
	print(num/time_all2)

