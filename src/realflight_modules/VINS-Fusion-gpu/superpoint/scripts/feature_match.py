import cv2
import copy
import numpy as np 
from time import time
import pycuda.driver as cuda

#from utils.feature_process import PointTracker
#from utils.feature_process import SuperPointFrontend_torch, SuperPointFrontend
from utils.feature_process_trt import PointTracker
from utils.feature_process_trt import SuperPointFrontend_TensorRT

# cuda.init()
# cfx = cuda.Device(0).make_context()

run_time = 0.0
match_time = 0.0

myjet = np.array([[0.        , 0.        , 0.5       ],
                  [0.        , 0.        , 0.99910873],
                  [0.        , 0.37843137, 1.        ],
                  [0.        , 0.83333333, 1.        ],
                  [0.30044276, 1.        , 0.66729918],
                  [0.66729918, 1.        , 0.30044276],
                  [1.        , 0.90123457, 0.        ],
                  [1.        , 0.48002905, 0.        ],
                  [0.99910873, 0.07334786, 0.        ],
                  [0.5       , 0.        , 0.        ]])


class VisualTracker:
	def __init__(self, opts, cams):

		self.curframe_ = {
				'PointID': [],
				'keyPoint': np.zeros((3,0)),
				'descriptor': np.zeros((256,0)),
				'unPts': np.zeros((3,0)),
				'image': None,
				}

		self.preframe_ = {
				'PointID': [],
				'keyPoint': np.zeros((3,0)),
				'descriptor': np.zeros((256,0)),
				'unPts': np.zeros((3,0)),
				'image': None
				}
		self.rightframe_={
				'PointID': [],
				'keyPoint': np.zeros((3,0)),
				'descriptor': np.zeros((256,0)),
				'unPts': np.zeros((3,0)),
				'image': None
		}
	
		self.camera = cams[0]
		if cams[1] != None:
			self.camera_right = cams[1]
		self.new_frame = None
		self.new_frame_right = None
		self.allfeature_cnt = 0
		
		self.cuda = opts.cuda
		self.scale = opts.scale
		self.max_cnt = opts.max_cnt
		self.nms_dist = opts.nms_dist
		self.nn_thresh = opts.nn_thresh
		self.no_display = opts.no_display
		self.width = opts.W // opts.scale
		self.height = opts.H // opts.scale
		self.conf_thresh = opts.conf_thresh
		self.weights_path = opts.weights_path
		self.stereo = opts.stereo
		self.use_sp_in_stereo = opts.use_sp_in_stereo
		self.trt_engine_path = opts.trt_engine_path

		# SuperPointFrontend_torch SuperPointFrontend
		# self.SuperPoint_Ghostnet = SuperPointFrontend_torch(
		# 	weights_path = self.weights_path, 
		# 	nms_dist = self.nms_dist,
		# 	conf_thresh = self.conf_thresh,
		# 	cuda = self.cuda
		# 	)
		self.SuperPoint_Ghostnet = SuperPointFrontend_TensorRT(
			engine_path = self.trt_engine_path, 
			nms_dist = self.nms_dist,
			conf_thresh = self.conf_thresh,
			)

	
		self.tracker = PointTracker(nn_thresh=self.nn_thresh)

		self.conf_thresh_type = 0
		self.pts_velocity = np.zeros((2,0))
		self.pts_right_velocity = np.zeros((2,0))
		self.cur_un_pts_map = {}
		self.prev_un_pts_map = {}
		self.cur_un_right_pts_map = {}
		self.prev_un_right_pts_map = {}
		self.cur_time = 0
		self.pre_time = 0
		self.right_extract_success = False

	#返回数据
	def getTrackRes(self):

		cur_un_pts = copy.deepcopy(self.curframe_['unPts'])
		cur_pts = copy.deepcopy(self.curframe_['keyPoint'] * self.scale)
		ids = copy.deepcopy(self.curframe_['PointID'])
		pts_velocity = copy.deepcopy(self.pts_velocity)

		return cur_un_pts, cur_pts, ids, pts_velocity

	def getRightTrackRes(self):

		cur_un_right_pts = copy.deepcopy(self.rightframe_['unPts'])
		cur_right_pts = copy.deepcopy(self.rightframe_['keyPoint'] * self.scale)
		right_ids = copy.deepcopy(self.rightframe_['PointID'])
		pts_right_velocity = copy.deepcopy(self.pts_right_velocity)

		return cur_un_right_pts, cur_right_pts, right_ids, pts_right_velocity
	
	#去畸变
	def undistortedLineEndPoints(self, cur_pts, camera):

		cur_un_pts = copy.deepcopy(cur_pts)

		for i in range(cur_un_pts.shape[1]):
			b = camera.liftProjective(cur_pts[:2,i])
			cur_un_pts[0,i] = b[0] / b[2]
			cur_un_pts[1,i] = b[1] / b[2]
			cur_un_pts[2,i] = 1

		return cur_un_pts

	#计算当前特征点的速度
	# def calPtsVelocity(self):

	# 	self.pts_velocity = np.zeros((2,0))
	# 	if self.preframe_['unPts'].shape[1] != 0:
	# 		dt = self.cur_time - self.pre_time#当前帧与前一帧时间差
	# 		#构建字典加速查找
	# 		prePtsDict = {value: index for index, value in enumerate(self.preframe_['PointID'])}
	# 		for i in range(self.curframe_['unPts'].shape[1]):
	# 			cur_id = self.curframe_['PointID'][i]
	# 			j = prePtsDict.get(cur_id, -1)
	# 			if j != -1:
	# 				v_xy = (self.curframe_['unPts'][:2, i] - self.preframe_['unPts'][:2, j]) / dt
	# 				v_xy = v_xy.reshape((2,1))
	# 				self.pts_velocity = np.append(self.pts_velocity, v_xy, axis = 1)
	# 			else:
	# 				self.pts_velocity = np.append(self.pts_velocity, np.zeros((2,1)), axis = 1)
	# 	else:
	# 		for i in range(self.curframe_['unPts'].shape[1]):
	# 			self.pts_velocity = np.append(self.pts_velocity, np.zeros((2,1)), axis = 1)

		#ids:特征点id  pts：当前帧去畸变后的2D特征点   cur_id_pts：当前帧特征点id到归一化坐标的映射表  pre_id_pts：上一帧帧特征点id到归一化坐标的映射表
	def calPtsVelocity(self, ids, pts, pre_id_pts):
		pts_velocity = np.zeros((2,0))
		cur_id_pts = {value: pts[:2, index] for index, value in enumerate(ids)}#构建当前帧特征点ID到归一化坐标的映射
		if len(pre_id_pts) != 0:
			dt = self.cur_time - self.pre_time
			for i in range(len(ids)):
				pre_pts = pre_id_pts.get(ids[i], None)
				if pre_pts is not None:
					v_xy = (pts[:2, i] - pre_pts) / dt
					v_xy = v_xy.reshape((2, 1))
					pts_velocity = np.append(pts_velocity, v_xy, axis = 1)
					#print("velocity of {} is ({}, {})".format(ids[i], v_xy[0], v_xy[1]))
				else:
					pts_velocity = np.append(pts_velocity, np.zeros((2,1)), axis = 1)
		else:
			for i in range(len(ids)):
				pts_velocity = np.append(pts_velocity, np.zeros((2,1)), axis = 1)
		return pts_velocity, cur_id_pts
	
	def precompile_numba(self):
		dummy_image = np.random.rand(self.height, self.width).astype(np.float32)
		self.SuperPoint_Ghostnet.run(dummy_image)
		print("==> Precompile for numba function Successfully.")
	#适配双目
	def readImage(self, new_img, cur_time):

		assert(new_img[0].ndim==2 and new_img[0].shape[0]==self.height and new_img[0].shape[1]==self.width), "Frame: provided image has not the same size as the camera model or image is not grayscale"
		self.new_frame = new_img[0]
		self.cur_time = cur_time

		first_image_flag = False

		if not self.curframe_['PointID']:
			self.curframe_['PointID'] = []
			self.curframe_['keyPoint'] = np.zeros((3,0))
			self.curframe_['descriptor'] = np.zeros((256,0))
			self.curframe_['unPts'] = np.zeros((3,0))

			self.curframe_['image'] = self.new_frame
			self.preframe_['image'] = self.new_frame
			first_image_flag = True

		else:
			self.curframe_['PointID'] = []
			self.curframe_['keyPoint'] = np.zeros((3,0))
			self.curframe_['descriptor'] = np.zeros((256,0))
			self.curframe_['unPts'] = np.zeros((3,0))

			self.curframe_['image'] = self.new_frame
		
		######################### 提取关键点和描述子 ############################
		print('*'*10 + " current frame " + '*'*10)
		start_time = time()
		self.curframe_['keyPoint'], self.curframe_['descriptor'], heatmap = self.SuperPoint_Ghostnet.run(self.new_frame, conf_thresh=0.015)
		self.conf_thresh_type = 0

		run_time = ( time()-start_time )*1000.0
		print("left image superpoint run time is :{}ms".format(run_time))

		keyPoint_size = self.curframe_['keyPoint'].shape[1]
		print("current keypoint size is :", keyPoint_size)

		if keyPoint_size < self.max_cnt-50:
			self.curframe_['keyPoint'], self.curframe_['descriptor'], heatmap = self.SuperPoint_Ghostnet.run(self.new_frame, conf_thresh=0.01)
			self.conf_thresh_type = 1
			keyPoint_size = self.curframe_['keyPoint'].shape[1]
			print("next keypoint size is ", keyPoint_size)

		

		for _ in range(keyPoint_size):
			if first_image_flag == True:
				self.curframe_['PointID'].append(self.allfeature_cnt)
				self.allfeature_cnt = self.allfeature_cnt+1
			else:
				self.curframe_['PointID'].append(-1)
		
		##################### 开始处理匹配的特征点 ###############################
		if self.preframe_['keyPoint'].shape[1] > 0:
			start_time = time()
			feature_matches = self.tracker.nn_match_two_way( 
									self.curframe_['descriptor'], 
									self.preframe_['descriptor'], 
									self.nn_thresh
							).astype(int)
			global match_time
			match_time = (time()-start_time) * 1000.0
			print("left match time is :{}ms".format(match_time))
			print("match size is :", feature_matches.shape[1])
			######################## 保证匹配得到的lineID相同 #####################
			start_time = time()
			for k in range(feature_matches.shape[1]):
				self.curframe_['PointID'][feature_matches[0,k]] = self.preframe_['PointID'][feature_matches[1,k]]
			run_time = ( time()-start_time )*1000.0
			print("left image 保证匹配得到的lineID相同 run time is :{}ms".format(run_time))

			################### 将跟踪的点与没跟踪的点进行区分 #####################
			start_time = time()
			vecPoint_new = np.zeros((3,0))
			vecPoint_tracked = np.zeros((3,0))
			PointID_new = []
			PointID_tracked = []
			Descr_new = np.zeros((256,0))
			Descr_tracked = np.zeros((256,0))

			for i in range(keyPoint_size):
				if self.curframe_['PointID'][i] == -1 :#新增特征点(未被匹配的特征点)
					self.curframe_['PointID'][i] = self.allfeature_cnt
					self.allfeature_cnt = self.allfeature_cnt+1
					vecPoint_new = np.append(vecPoint_new, self.curframe_['keyPoint'][:,i:i+1], axis=1)
					PointID_new.append(self.curframe_['PointID'][i])
					Descr_new = np.append(Descr_new, self.curframe_['descriptor'][:,i:i+1], axis=1)
				else:
					vecPoint_tracked = np.append(vecPoint_tracked, self.curframe_['keyPoint'][:,i:i+1], axis=1)
					PointID_tracked.append(self.curframe_['PointID'][i])
					Descr_tracked = np.append(Descr_tracked, self.curframe_['descriptor'][:,i:i+1], axis=1)
			run_time = ( time()-start_time )*1000.0
			print("left image 将跟踪的点与没跟踪的点进行区分 run time is :{}ms".format(run_time))
			########### 跟踪的点特征少于150了，那就补充新的点特征 ###############
			start_time = time()
			diff_n = self.max_cnt - vecPoint_tracked.shape[1]
			if diff_n > 0:#新增的特征点数量大于需要补充的特征点数量,只push diff_n个特征点
				if vecPoint_new.shape[1] >= diff_n:
					for k in range(diff_n):
						vecPoint_tracked = np.append(vecPoint_tracked, vecPoint_new[:,k:k+1], axis=1)
						PointID_tracked.append(PointID_new[k])
						Descr_tracked = np.append(Descr_tracked, Descr_new[:,k:k+1], axis=1)
				else:#否则将新增特征点全部push到vecPoint_tracked
					for k in range(vecPoint_new.shape[1]):
						vecPoint_tracked = np.append(vecPoint_tracked, vecPoint_new[:,k:k+1], axis=1)
						PointID_tracked.append(PointID_new[k])
						Descr_tracked = np.append(Descr_tracked, Descr_new[:,k:k+1], axis=1)
			
			self.curframe_['keyPoint'] = vecPoint_tracked
			self.curframe_['PointID'] = PointID_tracked
			self.curframe_['descriptor'] = Descr_tracked
			run_time = ( time()-start_time )*1000.0
			print("left image 跟踪的点特征少于150了,那就补充新的点特征 run time is :{}ms".format(run_time))
		#经过以上操作keyPoint的排序为[<成功关联的特征点>---<新增的特征点>]
		#去畸变
		start_time = time()
		self.curframe_['unPts']  = self.undistortedLineEndPoints(self.curframe_['keyPoint'] * self.scale, self.camera)
		run_time = ( time()-start_time )*1000.0
		print("left image 去畸变 run time is :{}ms".format(run_time))
		#计算特征点速度
		start_time = time()
		self.pts_velocity, self.cur_un_pts_map= self.calPtsVelocity(self.curframe_["PointID"], self.curframe_['unPts'], self.prev_un_pts_map)
		run_time = ( time()-start_time )*1000.0
		print("left image 计算特征点速度 run time is :{}ms".format(run_time))

		#################################### 处理右目 ####################################
		if new_img[1] is not None:
			self.new_frame_right = new_img[1]
			self.rightframe_['PointID'] = []
			self.rightframe_['keyPoint'] = np.zeros((3,0))
			self.rightframe_['descriptor'] = np.zeros((256,0))
			self.rightframe_['unPts'] = np.zeros((3,0))
			self.cur_un_right_pts_map.clear()

			self.rightframe_['image'] = self.new_frame_right

			if self.curframe_['keyPoint'].shape[1] > 0:
				if self.use_sp_in_stereo:
					#采用superpoint进行左右目匹配，提取特征点和描述子
					if self.conf_thresh_type == 0:
						right_conf_thresh = 0.015
					else:
						right_conf_thresh = 0.01

					right_start_time = time()
					self.rightframe_['keyPoint'], self.rightframe_['descriptor'], right_heatmap = self.SuperPoint_Ghostnet.run(self.new_frame_right, conf_thresh = right_conf_thresh)
					keyPoint_size = self.rightframe_['keyPoint'].shape[1]
					print("right image superpoint run time is :{}ms".format((time() - right_start_time)*1000.))

					#print("right img shape is {}:{}, keypoint size is {}, descriptor size is {}".format(self.new_frame_right.shape[0], self.new_frame_right.shape[1], keyPoint_size, self.rightframe_['descriptor'].shape[0]))
					if self.rightframe_['descriptor'].shape[0] != 256:
						self.right_extract_success = False
						print("error: right frame's descriptor's dim is {}, the frame stamp is {}".format(self.rightframe_['descriptor'].shape[0], cur_time))
						# bad_image = (np.dstack((self.new_frame_right, self.new_frame_right, self.new_frame_right)) * 255.).astype('uint8')
						# cv2.imshow('feature detector window',bad_image)
						# cv2.waitKey(0)
					else:
						self.right_extract_success = True

					if self.right_extract_success: #如果右目描述子没有问题
						for _ in range(keyPoint_size):
							self.rightframe_['PointID'].append(-1)

						#左右目特征点描述子匹配
						start_time = time()
						feature_matches = self.tracker.nn_match_two_way( 
												self.rightframe_['descriptor'], 
												self.curframe_['descriptor'], 
												self.nn_thresh
										).astype(int)
						match_time = (time()-start_time) * 1000.0
						print("right match time is :{}ms".format(match_time))
						print("match size is :", feature_matches.shape[1])

						start_time = time()
						for k in range(feature_matches.shape[1]):
							self.rightframe_['PointID'][feature_matches[0, k]] = self.curframe_['PointID'][feature_matches[1, k]]
					
						#仅保留成功匹配的点
						vecPoint_tracked = np.zeros((3,0))
						PointID_tracked = []
						Descr_tracked = np.zeros((256,0))
						for i in range(keyPoint_size):
							if self.rightframe_['PointID'][i] != -1:
								vecPoint_tracked = np.append(vecPoint_tracked, self.rightframe_['keyPoint'][:, i:i+1], axis = 1)
								PointID_tracked.append(self.rightframe_['PointID'][i])
								Descr_tracked = np.append(Descr_tracked, self.rightframe_['descriptor'][:, i:i+1], axis = 1)

						self.rightframe_['keyPoint'] = vecPoint_tracked
						self.rightframe_['PointID'] = PointID_tracked
						self.rightframe_['descriptor'] = Descr_tracked
						run_time = ( time()-start_time )*1000.0
						print("right image 分配ID+跟踪与未跟踪特征点分离 run time is :{}ms".format(run_time))
				else:
					#采用光流追踪
					
					lk_params = dict(winSize=(15, 15),maxLevel=2)
					#光流
					preImage = (self.curframe_['image'] * 255.).astype('uint8')
					nextImage = (self.preframe_['image'] * 255.).astype('uint8')
					prePts = copy.deepcopy(self.curframe_['keyPoint'][:2, :].astype(np.float32))
					prePts = prePts.T.reshape(self.curframe_['keyPoint'].shape[1], 1, 2)
					lk_start_time = time()
					p1, st, err = cv2.calcOpticalFlowPyrLK(preImage, nextImage, prePts, None, **lk_params)
					print("right image lk process time is {}ms".format((time() - lk_start_time)*1000.))
					for i in range(len(self.curframe_['PointID'])):
						if st[i] == 1:
							self.rightframe_['PointID'].append(self.curframe_['PointID'][i])
							right_pt = np.ones((3, 1))
							right_pt[0] = p1[i, 0, 0]
							right_pt[1] = p1[i, 0 ,1]
							self.rightframe_['keyPoint'] = np.append(self.rightframe_['keyPoint'], right_pt, axis = 1)
					
					self.right_extract_success = True
				if self.right_extract_success:
					#去畸变
					start_time = time()
					self.rightframe_['unPts'] = self.undistortedLineEndPoints(self.rightframe_['keyPoint'] * self.scale, self.camera_right)
					run_time = ( time()-start_time )*1000.0
					print("right image 去畸变 run time is :{}ms".format(run_time))
					#计算特征点速度
					start_time = time()
					self.pts_right_velocity, self.cur_un_right_pts_map= self.calPtsVelocity(self.rightframe_['PointID'], self.rightframe_['unPts'], self.prev_un_right_pts_map)
					run_time = ( time()-start_time )*1000.0
					print("right image 计算特征点速度 run time is :{}ms".format(run_time))
			self.prev_un_right_pts_map = copy.deepcopy(self.cur_un_right_pts_map)

		########### 绘制跟踪可视化结果 ###############
		#self.trackShow(self.preframe_, self.curframe_, heatmap)
		if self.stereo:
			self.trackShow(self.curframe_, self.rightframe_, heatmap)

		self.pre_time = self.cur_time
		self.prev_un_pts_map = copy.deepcopy(self.cur_un_pts_map)
		self.preframe_ = copy.deepcopy(self.curframe_)

	def trackShow(self, image0, image1, heatmap):
		draw_feature_matches = []
		if not self.no_display :	
			prePtsDict = {value: index for index, value in enumerate(image0['PointID'])}
			out1 = (np.dstack((image0['image'], image0['image'], image0['image'])) * 255.).astype('uint8')
			for i in range(len(image0['PointID'])):
				pts1 = (int(round(image0['keyPoint'][0,i]))-3, int(round(image0['keyPoint'][1,i]))-3)
				pts2 = (int(round(image0['keyPoint'][0,i]))+3, int(round(image0['keyPoint'][1,i]))+3)
				pt2 = (int(round(image0['keyPoint'][0,i])), int(round(image0['keyPoint'][1,i])))
				cv2.rectangle(out1, pts1, pts2, (0,255,0))
				cv2.circle(out1, pt2, 2, (255, 0, 0), -1)
				# cv2.putText(out1, str(self.curframe_['PointID'][i]), pt2, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX , 0.3, (0, 0, 255), lineType=5)
			cv2.putText(out1, 'pre_image Point', (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), lineType=16)

			out2 = (np.dstack((image1['image'], image1['image'], image1['image'])) * 255.).astype('uint8')
			for i in range(len(image1['PointID'])):
				j = prePtsDict.get(image1['PointID'][i], -1)
				if j != -1:
					draw_feature_matches.append((i,j))
				pts1 = (int(round(image1['keyPoint'][0,i]))-3, int(round(image1['keyPoint'][1,i]))-3)
				pts2 = (int(round(image1['keyPoint'][0,i]))+3, int(round(image1['keyPoint'][1,i]))+3)
				pt2 = (int(round(image1['keyPoint'][0,i])), int(round(image1['keyPoint'][1,i])))
				cv2.rectangle(out2, pts1, pts2, (0,255,0))
				cv2.circle(out2, pt2, 2, (0, 0, 255), -1)
				# cv2.putText(out2, str(self.forwframe_['PointID'][i]), pt2, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX, 0.3, (0, 0, 255), lineType=5)
			cv2.putText(out2, 'cur_image Point', (4, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), lineType=16)

			min_conf = 0.001
			heatmap[heatmap < min_conf] = min_conf
			heatmap = -np.log(heatmap)
			heatmap = (heatmap - heatmap.min()) / (heatmap.max() - heatmap.min() + .00001)
			out3 = myjet[np.round(np.clip(heatmap*10, 0, 9)).astype('int'), :]
			out3 = (out3*255).astype('uint8')

			out = np.hstack((out1, out2, out3))
			out = cv2.resize(out, (3*self.width, self.height))
			for i in range(len(draw_feature_matches)):
				pt1 = (int(round(image0['keyPoint'][0, draw_feature_matches[i][1]])), int(round(image0['keyPoint'][1, draw_feature_matches[i][1]])))
				pt2 = (int(round(image1['keyPoint'][0, draw_feature_matches[i][0]])) + self.width, int(round(image1['keyPoint'][1, draw_feature_matches[i][0]])))
				cv2.line(out, pt1, pt2, (0, 255, 0), 1)

			cv2.namedWindow("feature detector window",1)
			# cv2.resizeWindow("feature detector window", 640*3, 480)
			cv2.imshow('feature detector window',out)
			cv2.waitKey(1)