#include <string>
#include <vector>
#include <cstring>

#include "sfm_twoview_reconstruction.h"
#include "sfm_datastructures.h"
#include "keypoint_matching.h"
#include "sfm_utils.h"
#include "matrix.h"

using namespace std;

int loc(int x, int y)
{
	return (x - 1) * 4 + y - 1;
}


//int i,j,l,k,l1,l2,t,t1,ii,jj,kk;
//double x,y,d[16],p[10][12],u[10000],s[1200],v[144],r,a[1200];
v3_t d3;
int main(int argc, char **argv)
{
	vector<string> input;
	vector<double> focals;

	/// 输入为文件路径，以及每个图像对应的焦距的值
	input.push_back("./kermit000.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit001.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit006.jpg");
	focals.push_back(655.787);
/*	input.push_back("./kermit002.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit005.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit008.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit009.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit007.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit003.jpg");
	focals.push_back(655.787);
	input.push_back("./kermit004.jpg");
	focals.push_back(655.787);*/

	KeypointMatching km;
	SFMReconstruction recon;

	/// 提取特征点匹配关系
	km.process(input, focals);

	/// 双视图相机位置恢复
	CameraData cam1, cam2;
	sfm_twoview_reconstruction(km, 0, 1, &cam1, &cam2);

	/// 保存到SFMReconstruction结构中
	recon.cameras.push_back(cam1);
	recon.image_reconstructed.push_back(0);
	recon.cameras.push_back(cam2);
	recon.image_reconstructed.push_back(1);

	/// TODO: 在已经计算得到双视图相机姿态的情况下，重建三维空间点
	// write your code here!
	ImageData * im[10];
	double p[10][12];
	for (int i = 0 ; i < 3 ; ++i) im[i] = &km.m_images[i];
	for (int i0 = 0 ; i0 < im[0]->keypoints.size() ; ++i0)
		for (int i1 = 0 ; i1 < im[1]->keypoints.size() ; ++i1)
		{
			if ((im[0]->trackids[i0] > -1)&&(im[0]->trackids[i0]==im[1]->trackids[i1]))
			{
				double a[16] , u[16] , s[16] , vt[16];
				recon.cameras[0].get_projection_matrix(p[0]);
				recon.cameras[1].get_projection_matrix(p[1]);
				double x = im[0]->keypoints[i0].pt.x;
				double y = im[0]->keypoints[i0].pt.y;
				for (int k = 1; k <= 4; ++k)
					a[loc(2, k)] = x * p[0][loc(3, k)] - p[0][loc(1, k)];
				for (int k = 1; k <= 4; ++k)
					a[loc(1, k)] = y * p[0][loc(3, k)] - p[0][loc(2, k)];
				x = im[1]->keypoints[i1].pt.x;
				y = im[1]->keypoints[i1].pt.y;
				for (int k = 1; k <= 4; ++k)
					a[loc(4, k)] = x * p[1][loc(3, k)] - p[1][loc(1, k)];
				for (int k = 1; k <= 4; ++k)
					a[loc(3, k)] = y * p[1][loc(3, k)] - p[1][loc(2, k)];
				dgesvd_driver(4 , 4 , a , u , s , vt);
				double temp = vt[loc(4 , 4)];
				v3_t point = {vt[loc(4, 1)] / temp, vt[loc(4, 2)] / temp, vt[loc(4, 3)] / temp};
				v3_t color = {255 , 255 , 255};
				recon.points.push_back(point);
				recon.colors.push_back(color);
				recon.trackid_for_points.push_back(im[0]->trackids[i0]);
			}
		}

		for (int k = 2 ; k < 3 ; ++k)
		{
			double a0[1200];
			memset(a0 , 0 , sizeof(a0));
			int count = 0;


			for (int i = 0 ; i < im[k]->keypoints.size() ; ++i)
			{
				for (int j = 0 ; j < recon.trackid_for_points.size() ; ++j)
				{
					 if ((im[k]->trackids[i] > -1)&&(im[k]->trackids[i]==recon.trackid_for_points[j]))
					{
						 double x = im[k]->keypoints[i].pt.x;
						 double y = im[k]->keypoints[i].pt.y;
						 double X = recon.points[j].p[0];
						 double Y = recon.points[j].p[1];
						 double Z = recon.points[j].p[2];
						 a0[loc(2*count+1,5)] = -X;
						 a0[loc(2*count+1,6)] = -Y;
						 a0[loc(2*count+1,7)] = -Z;
						 a0[loc(2*count+1,8)] = -1;
						 a0[loc(2*count+1,9)] = y*X;
						 a0[loc(2*count+1,10)] = y*Y;
						 a0[loc(2*count+1,11)] = y*Z;
						 a0[loc(2*count+1,12)] = y;
						 a0[loc(2*count+2,1)] = X;
						 a0[loc(2*count+2,2)] = Y;
						 a0[loc(2*count+2,3)] = Z;
						 a0[loc(2*count+2,4)] = 1;
						 a0[loc(2*count+2,9)] = -x*X;
						 a0[loc(2*count+2,10)] = -x*Y;
						 a0[loc(2*count+2,11)] = -x*Z;
						 a0[loc(2*count+2,12)] = -x;
						 ++count;
					}
					if (count >= 50) break;
				 }
				 if (count >= 50) break;
			}
         
			double u0[10000] , s0[1200] , vt0[144];
			dgesvd_driver(100 , 12 , a0 , u0 , s0 , vt0);
			for (int i = 0 ; i < 12 ; ++i) p[k][i] = vt0[loc(12,i+1)];
			for (int pr = 0 ; pr < k-1 ; ++pr)
			{
				for (int i0 = 0 ; i0 < im[pr]->keypoints.size() ; ++i0)
					for (int i1 = 0 ; i1 < im[k]->keypoints.size() ; ++i1)
					{
						if ((im[pr]->trackids[i0] != -1)&&(im[k]->trackids[i1]==im[pr]->trackids[i0]))
						{
							double a[16] , u[16] , s[16] , vt[16];
							double x = im[pr]->keypoints[i0].pt.x;
							double y = im[pr]->keypoints[i0].pt.y;
							for (int l = 1; l <= 4; ++l)
								a[loc(2, l)] = x * p[pr][loc(3, l)] - p[pr][loc(1, l)];
							for (int l = 1; l <= 4; ++l)
								a[loc(1, l)] = y * p[pr][loc(3, l)] - p[pr][loc(2, l)];
							x = im[k]->keypoints[i1].pt.x;
							y = im[k]->keypoints[i1].pt.y;
							for (int l = 1; l <= 4; ++l)
								a[loc(4, l)] = x * p[k][loc(3, l)] - p[k][loc(1, l)];
							for (int l = 1; l <= 4; ++l)
								a[loc(3, l)] = y * p[k][loc(3, l)] - p[k][loc(2, l)];
							dgesvd_driver(4 , 4 , a , u , s , vt);
							double temp = vt[loc(4 , 4)];
							v3_t point = {vt[loc(4, 1)] / temp, vt[loc(4, 2)] / temp, vt[loc(4, 3)] / temp};
							printf("%lf %lf %lf\n" , point.p[0] , point.p[1] , point.p[2]);
							if (point.p[0] > 10 || point.p[1] > 10 || point.p[2] > 10) continue;
							v3_t color = {255 , 255 , 255};
							recon.points.push_back(point);
							recon.colors.push_back(color);
							recon.trackid_for_points.push_back(im[k]->trackids[i1]);
						}
					}
			}
		}
			dump_reconstruction_to_ply("d:\\", "testFSY.ply", recon);
			printf("Done\n");
			system("pause");
			return 0;
}
