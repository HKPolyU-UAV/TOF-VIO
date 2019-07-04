//        pcl::KdTreeFLANN<PointT> kdtree;
//        if(!icp_init)
//        {
//            prev_pose.setIdentity();
//            prevCloud = currCloud;
//            icp_init = 1;
//            return;
//        }else
//        {
//            //down sample to 1000 pts
//            kdtree.setInputCloud(prevCloud);
//            t_cb.toc("creat kdtree");
//        }

//        tic_toc_ros tt;
//        for(size_t i=0; i<3000; i++)
//        {
//            PointT searchPoint=currCloud->at(i);
//            if(searchPoint.z==searchPoint.z)
//            {
//                //        std::vector<int> pointIdxRadiusSearch;
//                //        std::vector<float> pointRadiusSquaredDistance;
//                std::vector<int> pointIdxNKNSearch(10);
//                std::vector<float> pointNKNSquaredDistance(10);
//                float radius = 3;
//                //        std::cout << "Neighbors within radius search at (" << searchPoint.x
//                //                  << " " << searchPoint.y
//                //                  << " " << searchPoint.z
//                //                  << ") with radius=" << radius << std::endl;
//                if ( kdtree.nearestKSearch (searchPoint, 10, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//                {
//                    //          for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//                    //            std::cout << "    "  <<   prev_cloud->points[ pointIdxRadiusSearch[i] ].x
//                    //                      << " " << prev_cloud->points[ pointIdxRadiusSearch[i] ].y
//                    //                      << " " << prev_cloud->points[ pointIdxRadiusSearch[i] ].z
//                    //                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//                }
//            }

//        }
//tt.toc("One Loop NNS");

//inline void buildpc_1000(const Mat& depth_img, const Mat& intensity_img, const Mat& intr_rect_ir, CloudTPtr& cloud)
//{
//    cloud = CloudT::Ptr(new CloudT);
//    int w = depth_img.cols;
//    int h = depth_img.rows;
//    size_t pcsize=w*h;

//    double cx = intr_rect_ir.at<double>(0,2);
//    double cy = intr_rect_ir.at<double>(1,2);
//    double fx_inv = 1.0 / intr_rect_ir.at<double>(0,0);
//    double fy_inv = 1.0 / intr_rect_ir.at<double>(1,1);

//    int count = 0;
//    PointT pt;
//    for(count=0; count<1000; )
//    {
//        size_t idx=rand() % pcsize;
//        int v = idx/w;
//        int u = idx%w;
//        float z = depth_img.at<float>(v, u);
//        if(z!=0) {
//            pt.x = (float)(z * ((u - cx) * fx_inv));
//            pt.y = (float)(z * ((v - cy) * fy_inv));
//            pt.z = z;
//            //CAMERA TO ENU
//            //            pt.y = -(float)(z * ((u - cx) * fx_inv));
//            //            pt.z = -(float)(z * ((v - cy) * fy_inv));
//            //            pt.x = z;
//            pt.intensity = (float)intensity_img.at<unsigned char>(v,u);
//            cloud->push_back(pt);
//            count++;
//        }
//    }
//    cloud->width = count;
//    cloud->height = 1;
//    cloud->is_dense = true;
//}


//    void synccallback(const sensor_msgs::ImageConstPtr& imagePtr_depth, const sensor_msgs::ImageConstPtr& imagePtr_intensity)
//    {
//        tic_toc_ros t_cb;
//        t_cb.toc("call back");
//        cv_bridge::CvImagePtr d_raw_image = cv_bridge::toCvCopy(imagePtr_depth, imagePtr_depth->encoding);
//        cv_bridge::CvImagePtr i_raw_image  = cv_bridge::toCvCopy(imagePtr_intensity,  imagePtr_intensity->encoding);

//        Mat d_image,i_image;
//        cv::undistort(i_raw_image->image, i_image, cameraMatrix, distCoeffs);
//        i_image.convertTo(i_image,CV_8UC1);
//        cv::equalizeHist(i_image,i_image);
//        cv::GaussianBlur(i_image,i_image, Size(3,3), 0);
//        cv::medianBlur(i_image,i_image,3);
//        //cv::bilateralFilter(i_image,i_image,5,100,100);

//        cv::undistort(d_raw_image->image, d_image, cameraMatrix, distCoeffs);
//        cv::imshow("undis_d", d_image);
//        cv::imshow("undis_i", i_image);
//        cv::waitKey(2);
//        t_cb.toc("undistort");

//        CloudTPtr curr_cloud;
//        buildpc_1000(d_image,i_image,cameraMatrix,curr_cloud);
//        sensor_msgs::PointCloud2 output;
//        //cout << pcptr->size() << endl;
//        pcl::toROSMsg(*curr_cloud,output);
//        output.header.frame_id="royale_camera_optical_frame";
//        output.header.stamp = imagePtr_depth->header.stamp;
//        pub1000pc.publish(output);


//        pcl::KdTreeFLANN<PointT> kdtree;
//        if(!icp_init)
//        {
//            prev_pose.setIdentity();
//            prev_cloud = curr_cloud;
//            icp_init = 1;
//            return;
//        }else
//        {
//            //down sample to 1000 pts
//            kdtree.setInputCloud(prev_cloud);
//            t_cb.toc("creat kdtree");
//        }

//        tic_toc_ros tt;
//        for(size_t i=0; i<1000; i++)
//        {
//            PointT searchPoint=curr_cloud->at(i);
//            //        std::vector<int> pointIdxRadiusSearch;
//            //        std::vector<float> pointRadiusSquaredDistance;
//            std::vector<int> pointIdxNKNSearch(10);
//            std::vector<float> pointNKNSquaredDistance(10);
//            float radius = 3;
//            //        std::cout << "Neighbors within radius search at (" << searchPoint.x
//            //                  << " " << searchPoint.y
//            //                  << " " << searchPoint.z
//            //                  << ") with radius=" << radius << std::endl;
//            if ( kdtree.nearestKSearch (searchPoint, 10, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
//            {
//                //          for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
//                //            std::cout << "    "  <<   prev_cloud->points[ pointIdxRadiusSearch[i] ].x
//                //                      << " " << prev_cloud->points[ pointIdxRadiusSearch[i] ].y
//                //                      << " " << prev_cloud->points[ pointIdxRadiusSearch[i] ].z
//                //                      << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
//            }
//        }
//        tt.toc("One Loop NNS");

//        tic_toc_ros tt2;
//        for(size_t i=0; i<1000; i++)
//        {
//            PointT searchPoint=curr_cloud->at(i);
//            double u,v;
//            reprojection(searchPoint,u,v);

//            if(u>224 || v>171 || u<0 ||v<0)
//            {
//                cout << searchPoint.x << " " << searchPoint.y << " " << searchPoint.z << endl;
//                cout << "u" << u <<"  v:" << v << endl;
//            }
//        }
//        tt2.toc("One Loop NNS");
//    }


//        for (size_t i = 0; i<pairs.size(); i+=15)
//        {
//            PointT pt_from, pt_to;
//            pt_from = get<0>(pairs[i]);
//            pt_to   = get<1>(pairs[i]);
//            cout << "[" << pt_from.x <<","<<pt_from.y<<"," <<pt_from.z<<"]" << " mit "
//                 << "[" << pt_to.x <<","<<pt_to.y<<"," <<pt_to.z<<"]"<<endl;
//        }


//void remove_nan(CloudTPtr& in, CloudTPtr& out)
//{
//    output.reset();
//    int count=0;
//    for(int i=0; i< in->size(); i++)
//    {
//        PointT pt=in->at(i);
//        if(pt.x==pt.x && pt.y==pt.y && pt.z==pt.z && pt.intensity==pt.intensity)
//        {
//            out->push_back(pt);
//            count++;
//        }
//    }
//    out->width = count;
//    out->height = 1;
//}
