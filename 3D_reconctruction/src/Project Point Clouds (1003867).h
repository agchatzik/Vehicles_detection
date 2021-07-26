
//#include <math.h>
#include <iostream>
//#include <vector>
//#include <string>
//#include <iterator>
//#include <algorithm>
//#include <array>
//#include <VVRScene/scene.h>
#include <VVRScene/canvas.h>
#include <VVRScene/utils.h>
#include <opencv2/opencv.hpp>
#include <GeoLib.h>
#include <MathGeoLib.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>
#include <ctime>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <chrono>

#define DIMENSIONS 3

#define FLAG_SHOW_FIRST_PC		 1
#define FLAG_SHOW_SECOND_PC		 2
#define FLAG_SHOW_CONNECT_PC     4
#define FLAG_ÏUTER_PRODUCT_NORMALS		 8
#define FLAG_SHOW_ORIGINAL_CLOUDSET	16
#define FLAG_COLOUR_PC		32
#define FLAG_TRIANGLE_NORMALS 64

#define FLAG_TASK_1 128
#define FLAG_TASK_2 256
#define FLAG_TASK_3 512

#define FLAG_SEGMENT_PC 1024
#define FLAG_SHOW_TRIANGULATION 2048

#define FLAG_TASK_4 4096
#define FLAG_TASK_5 8192
//#define FLAG_SHOW_AABB      32

//#define DEFAULT 0.535353

using namespace vvr;
using namespace cv;
using namespace std;
using namespace Eigen;
using namespace chrono;

void Task_2_CalculateMatching2(const cv::Mat& leftImage, const cv::Mat& rightImage, std::vector<vec> &vertices, std::vector<vec> &vertices2);

typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> TransformType;
typedef std::vector<Eigen::Vector3d>                PointsType;
TransformType computeRigidTransform(const PointsType& src, const PointsType& dst);

//std::vector<cv::Point3f> PCloud;
//std::vector<cv::Point3f> PCloud;


/**
 * A node of a KD-Tree
 */
struct KDNode
{
	vec split_point;
	int axis;
	int level;
	AABB aabb;
	KDNode *child_left;
	KDNode *child_right;
	KDNode() : child_left(NULL), child_right(NULL) {}
	~KDNode() { delete child_left; delete child_right; }
};

/**
 * KD-Tree wrapper. Holds a ptr to tree root.
 */
class KDTree
{
public:
	KDTree(VecArray &pts);
	~KDTree();
	std::vector<KDNode*> getNodesOfLevel(int level);
	int depth() const { return m_depth; }
	const KDNode* root() const { return m_root; }
	const VecArray &pts;

private:
	static int makeNode(KDNode *node, VecArray &pts, const int level);
	static void getNodesOfLevel(KDNode *node, std::vector<KDNode*> &nodes, int level);

private:
	KDNode *m_root;
	int m_depth;
};
/**
 * Function object to compare 2 3D-vecs in the specified axis.
 */
struct VecComparator {
	unsigned axis;
	VecComparator(unsigned axis) : axis(axis % 3) {}
	virtual inline bool operator() (const vec& v1, const vec& v2) {
		return (v1.ptr()[axis] < v2.ptr()[axis]);
	}
};

namespace vvr {
	

	std::string depth_dir = "../resources/depth_images/";
	std::string RGB_dir = "../resources/RGB_images/";

	Ptr<StereoBM> stereo_matcher = StereoBM::create(16);

	//Our Block Matching class
	
	bool run_first_task = true;
	bool run_second_task = false;
	bool run_third_task = false;
	bool run_fourth_task = false;
	bool run_fifth_task = false;

	std::vector<vec> m_vertices;
	std::vector<vec> m_vertices2;
	vector<vec> m_vertices_before;
	vector<Vec3b> RGB_colour_before;

	vec cOm1;
	KDTree *m_KDTree;
	KDTree *m_KDTree_sobel;
	KDTree *m_KDTree_original;
	std::vector<vvr::LineSeg3D> Connecting_Lines;

	int single_image_size = 307200;
	std::vector<cv::Mat> RGB_images;
	std::vector<cv::Mat> depth_images;
	cv::Mat depth_image;
	cv::Mat depth_image2;
	int index = 0;
	int choice = index;
	int choice2;
	cv::Mat RGB_image;
	cv::Mat RGB_image2;

	vector<cv::Vec3b> RGB_colour;
	vector<cv::Vec3b> RGB_colour2;

	vector<unsigned> vertices_with_triangles;
	vector<unsigned> indices_of_triangles_per_point;

	
	void CalculatePointCloud(const cv::Mat& disparity, std::vector<vec> &vertices);
	void Find_Nearest_and_connect_lines(std::vector<vec> &vertices, std::vector<vec> &vertices2);
	void Sobel_filter(const cv::Mat& Image, cv::Mat& Filtered_Image);
	int xGradient(cv::Mat image, int x, int y);
	int yGradient(cv::Mat image, int x, int y);
	void ICP(std::vector<vec> &vertices, std::vector<vec> &vertices2, Matrix3d &R_whole, Vector3d &T_whole);
	void Sobel_ICP(const int choice1,vector<Mat> &Sobel_images,std::vector<vec> &vertices_first, std::vector<vec> &vertices_second, Matrix3d&vectorR, Vector3d &vectorT);
	void Task_2_CalculateMatching(const cv::Mat& leftImage, const cv::Mat& rightImage);
	void Compute_center_of_mass(std::vector<vec> &vertices, vec &cOm);	
	void KDTree_Nearest(const vec& test_pt, const KDNode* root, const KDNode **nn, float *best_dist);
	void Remove_dublicates(std::vector<vec> &vertices, std::vector<vec> &non_dublicate_vertices, vector<Vec3b> &non_dublicate_colours);
	vec calcNormal(const vec &p1, const vec &p2, const vec &p3);
	void Apply_median_filter(const cv::Mat& Image, cv::Mat& Filtered_Image);
	ushort Median_filter(const cv::Mat image, int x, int y);
	float angle_of_vectors(vec& u, vec& v);
	void NearestK(const int k, const vec& test_pt, const KDNode* root, const KDNode **knn, float *best_dist);
	float compute_curvature(const vec &p, vec& normal);
	void computeCovarianceMatrix(std::vector<vec> points, Eigen::MatrixXd &covarianceMatrix);
	void TriangulateMesh(const std::vector<vec>& vertices, vvr::Mesh*& mesh);
	void Colour_connected_mesh_vetices(vvr::Mesh* mesh, vector<Point3D> &segemnted_vertices);
	bool FindAdjacentTriangle(vvr::Triangle &tri_i, vector<vvr::Triangle> &tris, vector<unsigned>& tri_adj_index);
	float computeMean(std::vector<float> numbers);
	float computeVariance(float mean, std::vector<float> numbers);
	void compute_Normals(const std::vector<vec>& m_vertices, vector<LineSeg3D>& PointCloudNormals);
	void Covariance_Normals(std::vector<vec> m_vertices, vector<LineSeg3D>& PointCloudNormals);
	void cuvature_estimation(std::vector<vec> all_the_normals, vector<float>& pointCloud_curvature);

	void read_image(int, void*) {

		cv::imshow("meeting_small", RGB_images[index]);
	}

	class PointCloud3DScene : public vvr::Scene
	{
	public:
		PointCloud3DScene();
		const char* getName() const { return "PointCloud 3D Scene"; }
		void keyEvent(unsigned char key, bool up, int modif) override;
		virtual void mousePressed(int x, int y, int modif) override;

	private:

		void drawCloud(const std::vector<vec> & vertices, vector<Vec3b> color);
		void drawCloud_Coloured(const std::vector<vec> & vertices,vvr::Colour Colour);
		void draw() override;
		void printKeyboardShortcuts();
		void Task1();
		void Task2();
		void Task3();
		void Task4();
		void Task5();
		void reset() override;
		
	private:

	
		std::vector<Point3D> segemnted_vertices;

		vector<LineSeg3D> TriangleNormals;
		vector<float> pointCloud_curvature;
		vector<LineSeg3D> PointCloudNormals;
		VecArray corresponding_pts;
		//KDTree *m_KDTree;
		//KDTree *m_KDTree_original;
		VecArray m_pts;

		int m_flag;
		float m_plane_d;
		vvr::Canvas2D m_canvas;
		vvr::Colour m_obj_col;
		//std::vector<vec> m_vertices;
		//std::vector<vec> m_vertices2;
		Mesh* m_model = nullptr;

		float total_distance;
	};
}


