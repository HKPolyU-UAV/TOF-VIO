from rosbag import Bag

with Bag('output.bag', 'w') as Y:
	for topic, msg, t in Bag('2019-09-22-23-53-41.bag'):
    		if   topic == '/mavros/vision_pose/pose':
        		Y.write('/gt', msg, t)
		elif topic == '/image_mono8':
			Y.write('/image_nir', msg, t)
		elif topic == '/image_mono_cam':
			Y.write('/monocam_image', msg, t)
		elif topic == 'mavros/imu/data':
			Y.write('/imu', msg, t)
    		else:
        		Y.write(topic, msg, t)
