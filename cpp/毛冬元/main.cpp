#include <string>
#include <vector>

#include "sfm_twoview_reconstruction.h"
#include "sfm_datastructures.h"
#include "keypoint_matching.h"
#include "sfm_utils.h"
#include "matrix.h"

using namespace std;

int i,j,l,k,l1,l2,t,t1,jj,kk;
double x,y,d[16],p[10][12],u[22500],s[12],v[144],r,a[1800];
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
  /*  input.push_back("./kermit002.jpg");
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
		ImageData &i1 = km.m_images[0];
		ImageData &i2 = km.m_images[1];
		l1=i1.keypoints.size();
		l2=i2.keypoints.size();
		recon.cameras[0].get_projection_matrix(p[0]);
		recon.cameras[1].get_projection_matrix(p[1]);
		for(i=0;i!=l1;i++)
			for(j=0;j!=l2;j++)
				if ((i1.trackids[i]!=-1)&&(i1.trackids[i]==i2.trackids[j])){
					x=i1.keypoints[i].pt.x;
					y=i1.keypoints[i].pt.y;
					for (k=0;k!=4;k++)
						d[k]=x*p[0][8+k]-p[0][k];
					for (k=0;k!=4;k++)
						d[4+k]=y*p[0][8+k]-p[0][4+k];
					x=i2.keypoints[j].pt.x;
					y=i2.keypoints[j].pt.y;
					for(k=0;k!=4;k++)
						d[8+k]=x*p[1][8+k]-p[1][k];
					for(k=0;k!=4;k++)
						d[12+k]=y*p[1][8+k]-p[1][4+k];
					dgesvd_driver(4,4,d,u,s,v);
					r=v[15];
					d3.p[0]=v[12]/r;
					d3.p[1]=v[13]/r;
					d3.p[2]=v[14]/r;
					v3_t color={rand()%256,rand()%256,rand()%256};
					recon.points.push_back(d3);
					recon.colors.push_back(color);
					recon.trackid_for_points.push_back(i1.trackids[i]);
				}
    /// 将重建结果写入演示文件
	for(k=2;k!=3;k++){
		i1 = km.m_images[k];
		l1=i1.keypoints.size();
		l2=recon.trackid_for_points.size();
		t=0;
		t1=0;
		for(i=0;i!=l1;i++)
			for(j=0;j!=l2;j++)
				if ((i1.trackids[i]!=-1)&&(i1.trackids[i]==recon.trackid_for_points[j])){
					x=i1.keypoints[i].pt.x;
					y=i1.keypoints[i].pt.y;
					a[t1]=0;a[t1+1]=0;a[t1+2]=0;a[t1+3]=0;
					a[t1+4]=-recon.points[j].p[0];a[t1+5]=-recon.points[j].p[1];a[t1+6]=-recon.points[j].p[2];a[t1+7]=-1;
					a[t1+8]=y*recon.points[j].p[0];a[t1+9]=y*recon.points[j].p[1];a[t1+10]=y*recon.points[j].p[2];a[t1+11]=y;
					t1+=12;
					t++;
					if(t==150)
						goto xxx;
					a[t1]=recon.points[j].p[0];a[t1+1]=recon.points[j].p[1];a[t1+2]=recon.points[j].p[2];a[t1+3]=1;
					a[t1+4]=0;a[t1+5]=0;a[t1+6]=0;a[t1+7]=0;
					a[t1+8]=-x*recon.points[j].p[0];a[t1+9]=-x*recon.points[j].p[1];a[t1+10]=-x*recon.points[j].p[2];a[t1+11]=-x;
					t1+=12;
					t++;
					if(t==150)
						goto xxx;
				}
xxx:	dgesvd_driver(t,12,a,u,s,v);
		t1=12*11;
		for(i=0;i!=12;i++)
			p[k][i]=v[t1+i];
		for(jj=0;jj<k;jj++){
			i2 = km.m_images[jj];
			l2=i2.keypoints.size();
			for(i=0;i!=l1;i++)
				for(j=0;j!=l2;j++)
					if ((i1.trackids[i]!=-1)&&(i1.trackids[i]==i2.trackids[j])){
						x=i1.keypoints[i].pt.x;
						y=i1.keypoints[i].pt.y;
						for (kk=0;kk!=4;kk++)
							d[kk]=x*p[k][8+kk]-p[k][kk];
						for (kk=0;kk!=4;kk++)
							d[4+kk]=y*p[k][8+kk]-p[k][kk+4];
						x=i2.keypoints[j].pt.x;
						y=i2.keypoints[j].pt.y;
						for (kk=0;kk!=4;kk++)
							d[8+kk]=x*p[jj][8+kk]-p[jj][kk];
						for (kk=0;kk!=4;kk++)
							d[12+kk]=y*p[jj][8+kk]-p[jj][kk+4];
						dgesvd_driver(4,4,d,u,s,v);
						r=v[15];
						d3.p[0]=v[12]/r;
						d3.p[1]=v[13]/r;
						d3.p[2]=v[14]/r;
						v3_t color={rand()%256,rand()%256,rand()%256};
						recon.points.push_back(d3);
						recon.colors.push_back(color);
						if(k<=4)
							recon.trackid_for_points.push_back(i1.trackids[i]);
					}
		}
	}
		
    dump_reconstruction_to_ply("d:\\", "test.ply", recon);

    return 0;
}