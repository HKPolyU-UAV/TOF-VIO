from rosbag import Bag

with Bag('output2019-10-19-13-05-14.bag', 'w') as Y:
	for topic, msg, t in Bag('2019-10-19-13-05-14.bag'):
    		if   topic == '/mavros/vision_pose/pose':
        		Y.write('/gt', msg, t)
		elif topic == '/image_mono8':
			Y.write('/image_nir', msg, t)
		elif topic == '/image_mono_cam':
			Y.write('/monocam_image', msg, t)
		elif topic == '/mavros/imu/data':
			Y.write('/imu', msg, t)
    		else:
        		Y.write(topic, msg, t)
