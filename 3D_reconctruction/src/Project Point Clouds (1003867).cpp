// Project Point clouds.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "Project Point Clouds (1003867).h"

//////////////////////////////////////////////////////////////////////////
	// Semester Project Poin Clouds by Chatzikalymnios Evangelos
//////////////////////////////////////////////////////////////////////////


// fuction used to extravt a certain part of a vector
template<typename T>
std::vector<T> slice(std::vector<T> const &v, int m, int n)
{
	auto first = v.cbegin() + m;
	auto last = v.cbegin() + n + 1;

	std::vector<T> vec(first, last);
	return vec;
}

// Main function
int main(int argc, char* argv[]) {

	//////////////////////////////////////////////////////////////////////////
	//Load RGB Images in a vector
	//////////////////////////////////////////////////////////////////////////
	int count = 181;
	for (int a = 1; a < count; a++)  // a <=Count would do one too many...
	{
		Mat img = imread(format("../resources/RGB_images/meeting_small_1_%d.png", a), IMREAD_COLOR); // pgm implies grayscale, maybe even: imread(name,0); to return CV_8U
		Mat img2 = imread(format("../resources/RGB_images/meeting_small_1_%d_depth.png", a), IMREAD_ANYDEPTH); // pgm implies grayscale, maybe even: imread(name,0); to return CV_8U
		// *always check* resource-loading.
		if (img.empty())
		{
			string name = (format("../resources/RGB_images/meeting_small_1_%d.png", a));
			cerr << "Image " << name << " can't be loaded!" << endl;
			continue;
		}
		RGB_images.push_back(img);
		depth_images.push_back(img2);
	}

	//Creating a Taskbar to diplay all the RGB images (using read_image function)
	int count2 = 179;
	cv::namedWindow("meeting_small", WINDOW_NORMAL);
	cv::createTrackbar("Frame", "meeting_small", &index, count2, read_image);
	read_image(0, (void*)0);

	// Start PointCloud3DScene Class
	try {
		return vvr::mainLoop(argc, argv, new PointCloud3DScene);
	}
	catch (std::string exc) {
		cerr << exc << endl;
		return 1;
	}
	catch (...) {
		cerr << "Unknown exception" << endl;
		return 1;
	}	
}

PointCloud3DScene::PointCloud3DScene()
{
	//! Load settings.
	vvr::Shape::DEF_LINE_WIDTH = 5;
	vvr::Shape::DEF_POINT_SIZE = 7;
	m_perspective_proj = true;
	m_bg_col = Colour("768E77");
	m_obj_col = Colour("454545");

	reset();
}

void PointCloud3DScene::printKeyboardShortcuts()
{
	std::cout << "Keyboard shortcuts:"
	
		<< std::endl << "'q' => TASK 1: SCENE FRAME INTO POINTCLOUD"
		<< std::endl << "'w' => TASK 2: POINTCLOUDS OF TWO FRAMES"
		<< std::endl << "'e' => TASK 3: SOBEL ICP"
		<< std::endl << "'r' => TASK 4: ICP"
		<< std::endl << "'t' => TASK 5: SCENE SEGMENTATION"<<std::endl 
		<< std::endl << "'b' => FIRST FRAME SELECTION"
		<< std::endl << "'f' => SHOW_FIRST_PC"
		<< std::endl << "'s' => SHOW_SECOND/REMAINING_PCs"
		<< std::endl << "'l' => SHOW_CONNECTING_LINES"
		<< std::endl << "'o' => SHOW_ORIGINAL_PC_POSITION"
		<< std::endl << "'c' => SHOW_COLOURED_PC"<< endl 
		<< std::endl << "'z' =>	SHOW_SCENE_SEGMENTATIOM"
		<< std::endl << "'u' => FLAG_SHOW_TRIANGULATION"
		<< std::endl << "'n' =>	FLAG_TRIANGLE_NORMALS"
		<< std::endl << "'m' =>	FLAG_OUTER_PRODUCT_NORMALS"
		
		<< std::endl << std::endl;
}

void PointCloud3DScene::reset()
{
	Scene::reset();

	// choosing the task
	if (run_first_task)
	{
		Task1();
		run_first_task = false;
		//run_second_task = false;
	}
	else if(run_second_task)
	{
		Task2();
		run_second_task = false;
		run_third_task = false;
	}
	else if (run_third_task)
	{
		Task3();
		run_third_task = false;
		run_second_task = false;
	}
	else if(run_fourth_task) 
	{
		Task4();
		run_third_task = false;
		run_second_task = false;
		run_fourth_task = false;
	}
	else if(run_fifth_task){

		Task5();
		run_fifth_task = false;
		run_third_task = false;
		run_second_task = false;
		run_fourth_task = false;
	}

	//! By Making `first_pass` static and initializing it to true,
	//! we make sure that the if block will be executed only once.
	static bool first_pass = true;

	//! Define what will be vissible by default
	if (first_pass) {
		m_flag = 0;
		m_flag |= FLAG_TASK_1;
		m_flag |= FLAG_SHOW_FIRST_PC;
		first_pass = false;
	}
}

//!//////////////////////////////////////////////////////////////////////////////////
//! Task 1 
//!//////////////////////////////////////////////////////////////////////////////////
void PointCloud3DScene::Task1()
{
	printKeyboardShortcuts();

	// clear vertices vector
	if (m_vertices.size() != 0) m_vertices.clear(); RGB_colour.clear();

	// Enter message
	std::cout << endl << "Entered Task 1" << endl;
	std::cout << endl << "Press 'b' to choose a frame" << endl;
	// by pressing 'b' you choose wich frame to display
	while (true)
	{
		choice = getTrackbarPos("Frame", "meeting_small");	
		if (waitKey(1) == 'b') break;
	}

	std::cout << endl << "Your choice is frame number " << choice << endl;

	// import choosen RB image and the corresponding depth image
	for (int i = choice + 1; i < choice + 2; i++) {
		depth_image = imread(format("../resources/depth_images/meeting_small_1_%d_depth.png", i), IMREAD_ANYDEPTH);
		RGB_image = imread(format("../resources/depth_images/meeting_small_1_%d.png", i), IMREAD_COLOR);

		//Aplly median filter at depth image
		cv::Mat filtered_depth_image(depth_image.rows, depth_image.cols, CV_8U);// chaeck cv_16
		Apply_median_filter(depth_image,filtered_depth_image);
		
		// calculate point cloud from depth image
		CalculatePointCloud(filtered_depth_image, m_vertices);
		//CalculatePointCloud(depth_image, m_vertices);

		// extract real colours from RGB image
		for (int x = 0; x < RGB_image.cols; x++) {
			for (int y = 0; y < RGB_image.rows; y++) {
				RGB_colour.push_back(RGB_image.at<cv::Vec3b>(y, x));
			}
		}
	}

	// Exit message
	std::cout << endl << "Task Task 1 completed" << endl;
}

//!//////////////////////////////////////////////////////////////////////////////////
//! Task 2
//!//////////////////////////////////////////////////////////////////////////////////
void PointCloud3DScene::Task2() {

	printKeyboardShortcuts();

	// clear vertices vector
	if (m_vertices.size() != 0) m_vertices.clear(); RGB_colour.clear();

	std::cout << endl << "Entered TASK 2" << endl;
	std::cout << endl << "Press 'b' to choose the first frame" << endl;

	// by pressing 'b' you choose wich frame to display
	while (true)
	{
		choice = getTrackbarPos("Frame", "meeting_small");
		if (waitKey(1) == 'b') break;
	}

	std::cout << endl << "Your choice is frame number " << choice << endl;

	// import choosen RGB image, the next frame and the corresponding depth images
	for (int i = choice + 1; i < choice + 3; i++) {
		depth_image = imread(format("../resources/depth_images/meeting_small_1_%d_depth.png", i), IMREAD_ANYDEPTH);
		RGB_image = imread(format("../resources/depth_images/meeting_small_1_%d.png", i), IMREAD_COLOR);
		CalculatePointCloud(depth_image, m_vertices);
		for (int x = 0; x < RGB_image.cols; x++) {
			for (int y = 0; y < RGB_image.rows; y++) {
				RGB_colour.push_back(RGB_image.at<cv::Vec3b>(y, x));
			}
		}
	}

	// starting index of the subvector
	int m = 0, n = single_image_size - 1;
	std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
	std::cout << "m_vertices1 size: " << m_vertices1.size() << endl;

	//  ending index of the subvector
	int m2 = single_image_size, n2 = single_image_size * 2 - 1;
	std::vector<vec>  m_vertices_2 = slice(m_vertices, m2, n2);
	std::cout << "m_vertices2 size: " << m_vertices_2.size() << endl;

	// find nearest point of the first pointcloud into the second pointcloud and connect them with lines

	float time = getSeconds();

	Find_Nearest_and_connect_lines(m_vertices1, m_vertices_2);

	time = getSeconds() - time;
	std::cout << endl<< "Nearest matching Time: " << time << endl;
	
	// Exit message
	std::cout << endl << "Task 2 completed" << endl;
}

//!//////////////////////////////////////////////////////////////////////////////////
//! Task 3
//!//////////////////////////////////////////////////////////////////////////////////
void PointCloud3DScene::Task3() {

	printKeyboardShortcuts();

	// clear vertices vector and colours vector
	if (m_vertices.size() != 0) m_vertices.clear(); RGB_colour.clear();
	

	std::cout << endl << "entered task 3!" << endl;
	std::cout << endl << "Press 'b' to choose the first frame" << endl;
	std::cout << endl << "Then press 'n' to choose the second frame" << endl;
	
	bool fist_frame_done = false;
	// by pressing 'b' you choose the initial frame 
	while (true)
	{
		fist_frame_done = true;
		if (waitKey(1) == 'b') {
			choice = getTrackbarPos("Frame", "meeting_small");
			break;
		}
	}
	// by pressing 'n' you choose the last frame 
	if (fist_frame_done) {
		while (true)
		{
			fist_frame_done = true;
			if (waitKey(1) == 'n') {
				choice2 = getTrackbarPos("Frame", "meeting_small");
				break;
			}
		}
	}

	std::cout << endl << "First choice frame number " << choice << endl;
	std::cout << "Second choice frame number " << choice2 << endl;

	// import choosen RGB images and the corresponding depth images
	for (int i = choice + 1; i < choice2 + 2; i++) {
		depth_image = imread(format("../resources/depth_images/meeting_small_1_%d_depth.png", i), IMREAD_ANYDEPTH);
		RGB_image = imread(format("../resources/depth_images/meeting_small_1_%d.png", i), IMREAD_COLOR);
		CalculatePointCloud(depth_image, m_vertices);
		for (int x = 0; x < RGB_image.cols; x++) {
			for (int y = 0; y < RGB_image.rows; y++) {
				RGB_colour.push_back(RGB_image.at<cv::Vec3b>(y, x));
			}
		}
	}

	/////////////////// ICP		///////////////

	float time2 = getSeconds();

	// keep the initial state of pointclouds before ICP
	m_vertices_before = m_vertices;
	RGB_colour_before = RGB_colour;

	// Define two vectors: 
	//1)Rotation matrix vector and 
	//2)vector of transportation vectors
	vector<Matrix3d>vectorR;
	vector<Vector3d>vectorT;

	for (int i = 0; i < choice2 - choice; i++) {

		// Extracting the first subvector
		int m = single_image_size * i, n = single_image_size * (i + 1) - 1;
		std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
	

		// Extracting the second subvector
		int m2 = single_image_size * (i + 1), n2 = single_image_size * (i + 2) - 1;
		std::vector<vec>  m_vertices3 = slice(m_vertices, m2, n2);
	

		// Rotation matrix and transportation vector 
		Matrix3d R_whole(3, 3);
		Vector3d T_whole(0, 0, 0);

		// Apply ICP 
		ICP(m_vertices1, m_vertices3, R_whole, T_whole);

		// keep the Rotation matrix vector and the vector of transportation
		vectorR.push_back(R_whole);
		vectorT.push_back(T_whole);
	}

	///// Move the points ////

	for (int l = 0; l < vectorR.size(); l++) {
		// Initialize the Rotation matrix vector 
		Matrix3d R(3, 3);
		for (int u = 0; u < 3; u++) {
			for (int v = 0; v < 3; v++) {
				R(u, v) = (u == v) ? 1 : 0;
			}
		}
		// Initialize the Transportation vector
		Vector3d T(0, 0, 0);

		// Creating the final Rotation matrix and Trasportation vector for the current pointcloud
		for (int k = l; k > -1; k--) {
			R *= vectorR[k];
			T += vectorT[k];
		}

		// Executing the rigid tranformation (the current tranfsormation include the previous ones)
		for (int j = single_image_size * (l + 1); j < single_image_size * (l + 2) - 1; j++) {
			if (m_vertices[j].x == 0 && m_vertices[j].y == 0 && m_vertices[j].z == 0) continue;
			Vector3d vert;
			vert = R * Vector3d(m_vertices[j].x, m_vertices[j].y, m_vertices[j].z) + T;
			m_vertices[j].x = vert[0];
			m_vertices[j].y = vert[1];
			m_vertices[j].z = vert[2];
		}
	}

	// Remove very close points, considered as dublicates //
	vector<vec> non_dublicate_vertices;
	vector<vec> vertices_with_doubles;
	vector<Vec3b> non_dublicate_colours;

	

	Remove_dublicates(m_vertices, non_dublicate_vertices, non_dublicate_colours);

	vertices_with_doubles = m_vertices;
	m_vertices = non_dublicate_vertices;
	reverse(m_vertices.begin(), m_vertices.end());

	RGB_colour = non_dublicate_colours;
	reverse(RGB_colour.begin(), RGB_colour.end());

	std::cout <<endl<< "vertices size before : " << vertices_with_doubles.size() << endl;
	std::cout << "non_dublicate vertices size: " << non_dublicate_vertices.size() << endl;
	std::cout << "Number of dublicates erased from poitcloud: " << vertices_with_doubles.size() -non_dublicate_vertices.size() << endl;

	time2 = getSeconds() - time2;
	std::cout << "ICP Time: " << time2 << endl;
	// Exit message
	std::cout << endl << "Task Task 3 completed" << endl;
}
//!//////////////////////////////////////////////////////////////////////////////////
//! Task 4 
//!////////////////////////////////////////////////////////////////////////////////// 
void PointCloud3DScene::Task4() {

	printKeyboardShortcuts();

	// clear vertices vector and colours vector
	if (m_vertices.size() != 0) m_vertices.clear(); RGB_colour.clear();

	std::cout << endl << "entered task 4!" << endl;

	bool fist_frame_done = false;
	
	std::cout << endl << "Press 'b' to choose the first frame" << endl;
	std::cout << endl << "Ten press 'n' to choose the second frame" << endl;
	// by pressing 'b' you choose the initial frame 
	while (true)
	{
		fist_frame_done = true;
		if (waitKey(1) == 'b') {
			choice = getTrackbarPos("Frame", "meeting_small");
			break;
		}
	}
	// by pressing 'n' you choose the last frame 
	if (fist_frame_done) {
		while (true)
		{
			fist_frame_done = true;
			if (waitKey(1) == 'n') {
				choice2 = getTrackbarPos("Frame", "meeting_small");
				break;
			}
		}
	}

	std::cout << endl << "First choice frame number " << choice << endl;
	std::cout << "Second choice frame number " << choice2 << endl;

	// import choosen RGB images and the corresponding depth images
	for (int i = choice + 1; i < choice2 + 2; i++) {
		depth_image = imread(format("../resources/depth_images/meeting_small_1_%d_depth.png", i), IMREAD_ANYDEPTH);
		RGB_image = imread(format("../resources/depth_images/meeting_small_1_%d.png", i), IMREAD_COLOR);
		CalculatePointCloud(depth_image, m_vertices);
		for (int x = 0; x < RGB_image.cols; x++) {
			for (int y = 0; y < RGB_image.rows; y++) {
				RGB_colour.push_back(RGB_image.at<cv::Vec3b>(y, x));
			}
		}
	}

	/////////////////SOBEL ICP//////////////////

	float time = getSeconds();

	vector<Mat> Sobel_images;

	// keep the initial state of pointclouds before ICP
	m_vertices_before = m_vertices;
	RGB_colour_before = RGB_colour;

	// Apply sobel filtering to the chosen frames to extract edges
	for (int i = 0; i < choice2 - choice + 1; i++) {
		Mat grayscale_image;
		cvtColor(RGB_images[choice + i], grayscale_image, cv::COLOR_RGB2GRAY);
		Mat Filtered_image(RGB_images[choice + i].rows, RGB_images[choice + i].cols, CV_8UC1);
		Sobel_filter(grayscale_image, Filtered_image);
		Sobel_images.push_back(Filtered_image);
	}

	// Define two vectors: 
	//1)Rotation matrix vector and 
	//2)vector of transportation vectors
	vector<Matrix3d>vectorR;
	vector<Vector3d>vectorT;

	
	for (int i = 0; i < choice2 - choice; i++) {

		// Extracting the first subvector
		int m = single_image_size * i, n = single_image_size * (i + 1) - 1;
		std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
	

		// Extracting the second subvector
		int m2 = single_image_size * (i + 1), n2 = single_image_size * (i + 2) - 1;
		std::vector<vec>  m_vertices3 = slice(m_vertices, m2, n2);
		

		// Rotation matrix and transportation vector 
		Matrix3d R_whole(3, 3);
		Vector3d T_whole(0, 0, 0);

		// Apply sobel ICP 
		Sobel_ICP(i, Sobel_images, m_vertices1, m_vertices3, R_whole, T_whole);

		// keep the Rotation matrix vector and the vector of transportation
		vectorR.push_back(R_whole);
		vectorT.push_back(T_whole);
	}

	///// Move the points ////

	for (int l = 0; l < vectorR.size(); l++) {
		// Initialize the Rotation matrix vector 
		Matrix3d R(3, 3);
		for (int u = 0; u < 3; u++) {
			for (int v = 0; v < 3; v++) {
				R(u, v) = (u == v) ? 1 : 0;
			}
		}
		// Initialize the Transportation vector
		Vector3d T(0, 0, 0);

		// Creating the final Rotation matrix and Trasportation vector for the current pointcloud
		for (int k = l; k > -1; k--) {
			R *= vectorR[k];
			T += vectorT[k];
		}

		// Executing the rigid tranformation (the current transformation include the previous ones)
		for (int j = single_image_size * (l + 1); j < single_image_size * (l + 2) - 1; j++) {
			if (m_vertices[j].x == 0 && m_vertices[j].y == 0 && m_vertices[j].z == 0) continue;
			Vector3d vert;
			vert = R * Vector3d(m_vertices[j].x, m_vertices[j].y, m_vertices[j].z) + T;
			m_vertices[j].x = vert[0];
			m_vertices[j].y = vert[1];
			m_vertices[j].z = vert[2];
		}
	}

	// Remove very close points, considered as dublicates //
	vector<vec> non_dublicate_vertices;
	vector<Vec3b> non_dublicate_colours;
	vector<vec> vertices_with_doubles;

	Remove_dublicates(m_vertices, non_dublicate_vertices, non_dublicate_colours);

	vertices_with_doubles = m_vertices;
	m_vertices = non_dublicate_vertices;
	reverse(m_vertices.begin(), m_vertices.end());

	RGB_colour = non_dublicate_colours;
	reverse(RGB_colour.begin(), RGB_colour.end());

	std::cout << endl << "vertices size before : " << vertices_with_doubles.size() << endl;
	std::cout << "non_dublicate vertices size: " << non_dublicate_vertices.size() << endl;
	std::cout << "Number of dublicates erased from poitcloud: " << vertices_with_doubles.size() - non_dublicate_vertices.size() << endl;

	time = getSeconds() - time;
	std::cout << "Sobel_ICP Time: " << time << endl;
	// Exit message
	std::cout << endl << "Task Task 4 completed" << endl;
}

//!//////////////////////////////////////////////////////////////////////////////////
//! Task 5
//!//////////////////////////////////////////////////////////////////////////////////
void PointCloud3DScene::Task5() {

	printKeyboardShortcuts();

	std::cout << endl << "Entered Task 5" << endl;
	std::cout << endl << "Press 'b' to choose the first frame" << endl;
	// clear vertices vector and colours vector
	if (m_vertices.size() != 0) m_vertices.clear(); RGB_colour.clear();

	// by pressing 'b' you choose wich frame to display
	while (true)
	{
		choice = getTrackbarPos("Frame", "meeting_small");
		choice2 = choice;
		if (waitKey(1) == 'b') break;
	}

	std::cout << endl << "Your choice is frame number " << choice << endl;

	// import choosen RGB image and the corresponding depth image
	for (int i = choice + 1; i < choice2 + 2; i++) {
		depth_image = imread(format("../resources/depth_images/meeting_small_1_%d_depth.png", i), IMREAD_ANYDEPTH);
		RGB_image = imread(format("../resources/depth_images/meeting_small_1_%d.png", i), IMREAD_COLOR);

		//Aplly median filter at depth image
		cv::Mat filtered_depth_image(depth_image.rows, depth_image.cols, CV_8U);// chaeck cv_16
		Apply_median_filter(depth_image, filtered_depth_image);

		CalculatePointCloud(filtered_depth_image, m_vertices);
		//CalculatePointCloud(depth_image, m_vertices);

		// extract real colours from RGB image
		for (int x = 0; x < RGB_image.cols; x++) {
			for (int y = 0; y < RGB_image.rows; y++) {
				RGB_colour.push_back(RGB_image.at<cv::Vec3b>(y, x));
			}
		}
	}

	float time = getSeconds();

	////// Convert vertices into a mesh //////
	TriangulateMesh(m_vertices, m_model);

	time = getSeconds() - time;
	std::cout <<endl<< "Mesh triangulation Time: " << time << endl;

	vector<vvr::Triangle> &triangles = m_model->getTriangles();
	vector<vec> Triangle_normals;

	//* triangles for each vertice
	std::vector<vec> &verts = m_model->getVertices();

	float time4 = getSeconds();
	/////// make a vector<int> pointer to save the tringles indices per mesh vertice ///////
	int size = verts.size();
	vector<int>* tri_indices_per_mesh_vertice = new vector<int>[size];

	int index;
	for (int i = 0; i < triangles.size(); i++) {
		vvr::Triangle &tri_i = m_model->getTriangles()[i];

		index = tri_i.vi1;
		tri_indices_per_mesh_vertice[index].push_back(i);

		index = tri_i.vi2;
		tri_indices_per_mesh_vertice[index].push_back(i);

		index = tri_i.vi3;
		tri_indices_per_mesh_vertice[index].push_back(i);
	}

	/////////////// SEGMENTATION / colour connected vertices with the same colour ///////////////////
	/////// make a vector<int> pointer to save the neighbour vertices indices per mesh vertice ///////

	vector<int>* neighbour_verts_per_mesh_vertice = new vector<int>[size];
	for (int i = 0; i < size; i++) {

		int tri_size = tri_indices_per_mesh_vertice[i].size();
		int tri_index;
		int neighbour_index;
		for (int j = 0; j < tri_size; j++) {

			tri_index = tri_indices_per_mesh_vertice[i].at(j);
			// to simio 1 tou geitonikou
			neighbour_index = triangles[tri_index].vi1;
			bool already_in = false;
			for (int n = 0; n < neighbour_verts_per_mesh_vertice[i].size(); n++) if (neighbour_verts_per_mesh_vertice[i].at(n) == neighbour_index) already_in = true;
			if (neighbour_index != i && (!already_in)) neighbour_verts_per_mesh_vertice[i].push_back(neighbour_index);

			neighbour_index = triangles[tri_index].vi2;
			already_in = false;
			for (int n = 0; n < neighbour_verts_per_mesh_vertice[i].size(); n++) if (neighbour_verts_per_mesh_vertice[i].at(n) == neighbour_index) already_in = true;
			if (neighbour_index != i && (!already_in)) neighbour_verts_per_mesh_vertice[i].push_back(neighbour_index);

			neighbour_index = triangles[tri_index].vi3;
			already_in = false;
			for (int n = 0; n < neighbour_verts_per_mesh_vertice[i].size(); n++) if (neighbour_verts_per_mesh_vertice[i].at(n) == neighbour_index) already_in = true;
			if (neighbour_index != i && (!already_in)) neighbour_verts_per_mesh_vertice[i].push_back(neighbour_index);
		}
	}

	//// make an initial segmentation: color conected vertices with  the same colour ////

	// create a binary matrix to save the vetices that have been 
	// checked for neighbours: 1-> checked, 0-> not yet 
	vector<int> check_table;
	for (int i = 0; i < size; i++) {
		check_table.push_back(0);
	}

	vector<int> neigh_to_check;
	for (int i = 0; i < size; i++) {

		// change colour randomly each time i search a new region
		vvr::Colour col(rand() * 255, rand() * 255, rand() * 255);

		bool first_pass = true;

		if (check_table[i] == 1) {
			continue;
		}
		do {
		
			if (first_pass) {

				check_table[i] = 1;

				first_pass = false;

				int neigh_crowd = neighbour_verts_per_mesh_vertice[i].size();

				for (int j = 0; j < neigh_crowd; j++) {

					int neigh_index = *(&neighbour_verts_per_mesh_vertice[i].at(j));

					vec neigh_vert = verts[neigh_index];

					Point3D new_point(neigh_vert[0], neigh_vert[1], neigh_vert[2], col);

					segemnted_vertices.push_back(new_point);

					neigh_to_check.push_back(neigh_index);
				}
			}
			else {
				do
				{
					// paw sto proto geitona tis listas kathe fora kai psaxno gia geitones tou 
					if (check_table[neigh_to_check[0]] == 1) {
						neigh_to_check.erase(neigh_to_check.begin(), neigh_to_check.begin() + 1);
						continue;
					}

					check_table[neigh_to_check[0]] = 1;

					// tsekaro ton proto geitona gia geitones
					int neigh_crowd = neighbour_verts_per_mesh_vertice[neigh_to_check[0]].size();

					for (int n = 0; n < neigh_crowd; n++)
					{

						int neigh_index = *(&neighbour_verts_per_mesh_vertice[neigh_to_check[0]].at(n));

						// an exo tsekarei to sigekrimeno min ton valeis
						if (check_table[neigh_index] == 1) continue;

						// Insert the neigbour as a Point3D
						vec neigh_vert = verts[neigh_index];
						Point3D new_point(neigh_vert[0], neigh_vert[1], neigh_vert[2], col);

						segemnted_vertices.push_back(new_point);

						// eisago sti lista tvn geitonvn poy prepei na ereyniso to neo shmeio
						neigh_to_check.push_back(neigh_index);
					}
					neigh_to_check.erase(neigh_to_check.begin(), neigh_to_check.begin() + 1);

				} while (neigh_to_check.size() != 0);
			}
		} while (neigh_to_check.size() != 0);
	}

	time4 = getSeconds() - time4;
	std::cout << endl << "Segmentation algorithm Time: " << time4 << endl;

	//////////////END OF SEMENTATION ALGORITHM ///////////////////

	/*
	/////// calculate mesh tringles normals using Covariance method
	float time2 = getSeconds();

	Covariance_Normals(m_vertices, PointCloudNormals);
	time2 = getSeconds() - time2;
	std::cout << endl << "Normal calulation using covairance method Time: " << time2<< endl;
	*/

	/////// calculate mesh tringles normals using outer product
	float time5 = getSeconds();
	
	compute_Normals(m_vertices, PointCloudNormals);
	time5 = getSeconds() - time5;
	std::cout << endl << "Normal calulation using outer product Time: " << time5 << endl;

	/////// calculate mesh tringles normals using the triangle.getNormal() function
	float time3 = getSeconds();

	for (int t = 0; t < triangles.size(); t++) {
		//vec norm = m_model->getTriangles()[t].getNormal();
		//cout << m_model->getTriangles()[t].getNormal() << endl;
		Triangle_normals.push_back(m_model->getTriangles()[t].getNormal());
	}
	time3 = getSeconds() - time3;
	std::cout << endl << "Normal calulation using getTriangles() Time: " << time3 << endl;

	///// Normal Map ///////
	for (int i = 0; i < triangles.size(); i++) {

		vec p = triangles[i].getCenter();
		vec normal = Triangle_normals[i];
	
		////X: -1 to + 1 : Red : 0 to 255
		int R = (normal[0] + 1) * 255 / 2;
		////Y: -1 to + 1 : Green : 0 to 255
		int G = (normal[1] + 1) * 255 / 2;
		//Z : 0 to - 1 : Blue : 128 to 255
		int B = (1 - normal[2]) * 128 - 1;

		//export normal as a line segament
		LineSeg3D Normal = LineSeg3D(p[0], p[1], p[2],
			p[0] + normal[0], p[1] + normal[1],
			p[2] + normal[2], vvr::Colour(R, G, B));

		TriangleNormals.push_back(Normal);
	}
	
	// Exit message
	std::cout << endl << "Task Task 5 completed" << endl;
}

void PointCloud3DScene::mousePressed(int x, int y, int modif)
{
	Ray myRay = this->unproject(x, y);
	float dist; // here the distance to the XY plane will be stored
	myRay.Intersects(Plane(vec(0, 0, 0), vec(0, 0, 1)), &dist);
	vec myPoint = myRay.GetPoint(dist); // Here is the XY point
	std::cout << myPoint << endl;

	// call superclass
	Scene::mousePressed(x, y, modif);
}

void PointCloud3DScene::drawCloud(const vector<vec>& vertices, vector<Vec3b> colour)
{
	//vec cOm;
	//vector<vec> vertices2 = vertices;
	//Compute_center_of_mass(vertices2, cOm);
	for (int i = 0; i < vertices.size(); i+=4) {
		if (vertices[i].x == 0 && vertices[i].y == 0 && vertices[i].z == 0) continue;
		vec vert = vertices[i];	
		Vec3b color = colour[i];
		vvr::Point3D(vert.x , vert.y , vert.z , vvr::Colour(color[2], color[1], color[0])).draw(); //vvr::Colour(color[2], color[1], color[0])      Colour::blue
	}
}

void PointCloud3DScene::drawCloud_Coloured(const vector<vec>& vertices , vvr::Colour Colour)
{
	for (int i = 0; i < vertices.size(); i++) {
		if (vertices[i].x == 0 && vertices[i].y == 0 && vertices[i].z == 0) continue;
		vec vert = vertices[i];
		vvr::Point3D(vert.x, vert.y, vert.z, Colour).draw();
	}
}

void PointCloud3DScene::keyEvent(unsigned char key, bool up, int modif)
{
	Scene::keyEvent(key, up, modif);
	key = tolower(key);
	
	switch (key)
	{
	case 'o': m_flag ^= FLAG_SHOW_ORIGINAL_CLOUDSET; break;
	case 'c': m_flag ^= FLAG_COLOUR_PC; break;
	case 'f': m_flag ^= FLAG_SHOW_FIRST_PC; break;
	case 's': m_flag ^= FLAG_SHOW_SECOND_PC; break;
	case 'l': m_flag ^= FLAG_SHOW_CONNECT_PC; break;
	case 'm': m_flag ^= FLAG_ÏUTER_PRODUCT_NORMALS; break;
	case 'n': m_flag ^= FLAG_TRIANGLE_NORMALS; break;
	case 'z': m_flag ^= FLAG_SEGMENT_PC; break;
	case 'u': m_flag ^= FLAG_SHOW_TRIANGULATION; break;

	case 'q': m_flag ^= FLAG_TASK_1; break;
	case 'w': m_flag ^= FLAG_TASK_2; break;
	case 'e': m_flag ^= FLAG_TASK_3; break;
	case 'r': m_flag ^= FLAG_TASK_4; break;
	case 't': m_flag ^= FLAG_TASK_5; break;

	
	}
}

void PointCloud3DScene::draw()
{
	if (m_flag & FLAG_TASK_1) {

		if (m_flag & FLAG_SHOW_FIRST_PC) {
			int m = 0, n = single_image_size - 1;
			std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);
			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices, Colour::red);
			else drawCloud(m_vertices1, color);
		}
	}
	if (m_flag & FLAG_TASK_2) {

		static bool first_pass2 = true;
		//! Define what will be vissible by default
		if (first_pass2) {
			run_second_task = true;
			run_first_task = false;
			first_pass2 = false;
			reset();
		}

		if (m_flag & FLAG_SHOW_FIRST_PC) {
			int m = 0, n = single_image_size - 1;
			std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);
			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices1, Colour::red);
			else drawCloud(m_vertices1, color);
		}

		if (m_flag & FLAG_SHOW_SECOND_PC) {
			int m = single_image_size, n = m_vertices.size() - 1;
			std::vector<vec>  m_vertices2 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);

			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices2, Colour::blue);
			else drawCloud(m_vertices2, color);
		}
		if (m_flag & FLAG_SHOW_CONNECT_PC) {
			for (int i = 0; i < Connecting_Lines.size(); i++) (Connecting_Lines[i]).draw();
		}
	}
	if (m_flag & FLAG_TASK_3) {

		static bool first_pass3 = true;
		//! Define what will be vissible by default
		if (first_pass3) {
			run_second_task = false;
			run_first_task = false;
			run_third_task = true;
			first_pass3 = false;
			reset();
		}

		if (m_flag & FLAG_SHOW_FIRST_PC) {
			int m = 0, n = single_image_size - 1;
			std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);
			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices1, Colour::red);
			else drawCloud(m_vertices1, color);
		}
		
		if (m_flag & FLAG_SHOW_SECOND_PC) {
			int m = single_image_size, n = m_vertices.size() - 1;
			std::vector<vec>  m_vertices2 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);

			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices2, Colour::blue);
			else drawCloud(m_vertices2, color);
		}
		if (m_flag & FLAG_SHOW_ORIGINAL_CLOUDSET) {
			int m = single_image_size, n = m_vertices_before.size() - 1;
			std::vector<vec>  m_vertices1 = slice(m_vertices_before, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour_before, m, n);
			drawCloud(m_vertices1, color);

		}
		if (m_flag & FLAG_SHOW_CONNECT_PC) {
			for (int i = 0; i < Connecting_Lines.size(); i++) (Connecting_Lines[i]).draw();
		}
	}
	if (m_flag & FLAG_TASK_4) {

		static bool first_pass4 = true;
		//! Define what will be vissible by default
		if (first_pass4) {
			run_second_task = false;
			run_first_task = false;
			run_third_task = false;
			run_fourth_task = true;
			first_pass4 = false;
			reset();
		}

		if (m_flag & FLAG_SHOW_FIRST_PC) {
			int m = 0, n = single_image_size - 1;
			std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);
			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices1, Colour::red);
			else drawCloud(m_vertices1, color);
		}

		if (m_flag & FLAG_SHOW_SECOND_PC) {
			int m = single_image_size, n = m_vertices.size() - 1;
			std::vector<vec>  m_vertices2 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);

			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices2, Colour::blue);
			else drawCloud(m_vertices2, color);
		}
		if (m_flag & FLAG_SHOW_ORIGINAL_CLOUDSET) {
			int m = single_image_size, n = m_vertices_before.size() - 1;
			std::vector<vec>  m_vertices1 = slice(m_vertices_before, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour_before, m, n);
			drawCloud(m_vertices1, color);

		}
		if (m_flag & FLAG_SHOW_CONNECT_PC) {
			for (int i = 0; i < Connecting_Lines.size(); i++) (Connecting_Lines[i]).draw();
		}
	}
	if (m_flag & FLAG_TASK_5) {
		static bool first_pass5 = true;
		//! Define what will be vissible by default
		if (first_pass5) {
			run_fifth_task = false;
			run_second_task = false;
			run_first_task = false;
			run_third_task = false;
			run_fourth_task = false;
			run_fifth_task = true;
			first_pass5 = false;
			reset();
		}
		if (m_flag & FLAG_SHOW_FIRST_PC) {
			int m = 0, n = single_image_size - 1;
			std::vector<vec>  m_vertices1 = slice(m_vertices, m, n);
			std::vector<Vec3b>  color = slice(RGB_colour, m, n);
			if (m_flag & FLAG_COLOUR_PC) drawCloud_Coloured(m_vertices1, Colour::red);
			else drawCloud(m_vertices1, color);
		}
		if (m_flag & FLAG_SHOW_TRIANGULATION) {

			m_model->draw(Colour::black, WIRE);

		}
		if (m_flag & FLAG_ÏUTER_PRODUCT_NORMALS) {
			for (int i = 0; i < PointCloudNormals.size(); i++) {
				PointCloudNormals[i].draw();
			}			
		}
		if (m_flag & FLAG_TRIANGLE_NORMALS) {
			for (int i = 0; i < TriangleNormals.size(); i++) {

				TriangleNormals[i].draw();
			}		
		}
		if (m_flag & FLAG_SEGMENT_PC) {
			for (int i = 0; i < segemnted_vertices.size(); i++) {

				segemnted_vertices[i].draw();
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
						// FUNCTIONS
//////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////
						// ÔASK1
//////////////////////////////////////////////////////////////////////////
void vvr::CalculatePointCloud(const cv::Mat& depth_image, std::vector<vec> &vertices)
{

	vector<int> topleft(1, 1);
	vector<float> center(320,240);

	//Dimensions must be initialized to use 2-D indexing
	int imw = depth_image.cols; 
	int imh = depth_image.rows;

	// constanr defined by the dataset creator
	double constant = 570.3f;
	double  MM_PER_M = 50.0f;

	// convert depth image to 3d point clouds
	Mat xgrid(imh, imw, CV_16S);
	Mat ygrid(imh, imw, CV_16S);;

	for (unsigned short i = 0; i < imh; i++) {
		for (unsigned short j = 0; j < imw; j++) {
			xgrid.at<short>(i, j) = static_cast<short>(j + 1 - center[0]);
			ygrid.at<short>(i, j) = static_cast<short>(i + 1 - center[1]);//- center[1]; //// h float
		}
	}

	for (int i = 0; i < imw; i++) {
		for (int j = 0; j < imh; j++) {

			//if (depth_image.at<unsigned short>(j, i) ==0) continue;
			double x = (xgrid.at<short>(j, i)) * (depth_image.at<unsigned short>(j, i) / constant / MM_PER_M);
			double y = (ygrid.at<short>(j, i)) * (depth_image.at<unsigned short>(j, i) / constant / MM_PER_M);
			double z = (depth_image.at<unsigned short>(j, i) / MM_PER_M);
			//cout << z << endl;

			// y axis rotation
			/*
			Eigen::Matrix3d Ry(3,3);
		
			Ry(0, 0) = -1;
			Ry(0, 1) = 0;
			Ry(0, 2) = 0;
			Ry(1, 0) = 0;
			Ry(1, 1) = 1;
			Ry(1, 2) = 0;
			Ry(2, 0) = 0;
			Ry(2, 1) = 0;
			Ry(2, 2) = -1;
			
			Vector3d output = Vector3d(-x, -y, z);
			Vector3d vertice = Ry*output;
			vec out = vec(vertice[0], vertice[1], vertice[2]);
			*/
			vertices.push_back(vec(-x, -y, z));
		}
	}
}

// Median filter
ushort vvr::Median_filter(const cv::Mat image, int x, int y)
{
	vector<ushort> window_pixels;
	// Gather pixels included in 3x3 wuindow
	for (int i = -1; i < 2; i++) {
		for (int j = -1; j < 2; j++) {
			window_pixels.push_back(image.at<ushort>(y - i, x - j));
		}
	}

	// Sort the values

	std::sort(window_pixels.begin(), window_pixels.end());

	// Return the median value
	return window_pixels[4];
}

void vvr::Apply_median_filter(const cv::Mat& Image, cv::Mat& Filtered_Image)
{
	// Load an image
	if (!Image.data)
	{
		// *always check* resource-loading.
		std::cout << " No Image Loaded! " << endl;
		return;
	}

	Filtered_Image = Image;

	//Apply median filter (3*3 window)

	for (int y = 1; y < Image.rows - 1; y++) {
		for (int x = 1; x < Image.cols - 1; x++) {
			Filtered_Image.at<ushort>(y, x) = Median_filter(Image, x, y);
		}
	}
	// Display original and filtered images
	//namedWindow("Original");
	//cv::imshow("Original", Image);

	//namedWindow("Filtered image");
	//cv::imshow("Filtered image", Filtered_Image);
}
//////////////////////////////////////////////////////////////////////////
						// ÔASK2
//////////////////////////////////////////////////////////////////////////
void vvr::Find_Nearest_and_connect_lines(std::vector<vec> &vertices_1, std::vector<vec> &vertices_2) {

	VecArray m_vertices_unsorted = vertices_1;
	m_KDTree = NULL;
	m_KDTree = new KDTree(vertices_1);

	//    KDtree sort the vertices and changes the colour   // 
	vertices_1 = m_vertices_unsorted;

	vector<Vector3d> corresponding_pts;

	float total_dist = 0, dist;
	int n = 0;

	for (int i = 0; i < vertices_2.size(); i++) {
		// if the point is at zero continue
		if (vertices_2[i].x == 0 && vertices_2[i].y == 0 && vertices_2[i].z == 0) continue;

		const KDNode *nearest = NULL;
		KDTree_Nearest(vertices_2[i], m_KDTree->root(), &nearest, &dist);

		vec nn = nearest->split_point;
		corresponding_pts.push_back(Vector3d(nn.x, nn.y, nn.z));

		LineSeg3D Line(vertices_2[i].x, vertices_2[i].y, vertices_2[i].z,
			nn.x, nn.y, nn.z, Colour::green);
		Connecting_Lines.push_back(Line);

		total_dist += dist;
		n += 1;
	}

	std::cout << endl;
	std::cout << " Final Total distasnce " << total_dist << endl;
	std::cout << " Final Mean distance " << total_dist / n << endl;
}
//////////////////////////////////////////////////////////////////////////
						// ÔASK3
//////////////////////////////////////////////////////////////////////////
void vvr::ICP(std::vector<vec> &m_vertices, std::vector<vec> &m_vertices2, Matrix3d &R_whole, Vector3d &T_whole) {

	//    Form KDTree    // 

	vector<vec> vertices = m_vertices;
	vector<vec> vertices2 = m_vertices2;

	m_KDTree_sobel = NULL;
	//VecArray m_vertices_unsorted = vertices;

	m_KDTree_sobel = new KDTree(vertices);

   ///////////////////////////
   //    KDtree sort the vertices and changes the colour   // 
	//vertices = m_vertices_unsorted;

	vector<Vector3d> corresponding_pts;
	vector<Matrix3d>vectorR;

	T_whole = Vector3d(0, 0, 0);

	// Run ICP 10 times //
	for (int j = 0; j < 10; j++) {

		float total_dist = 0, mean_dist = 0;
		int n = 0;
		float dist;
		vector<Vector3d> p1s;

		for (int i = 0; i < vertices2.size(); i++) {

			if (vertices2[i].x == 0 && vertices2[i].y == 0 && vertices2[i].z == 0) continue;

			// counter used to calculate mean distance;
			n += 1;

			// first vector of points
			p1s.push_back(Vector3d(vertices2[i].x, vertices2[i].y, vertices2[i].z));

			// Find the corresponding point( the closest points for each psi point) using KDTree
			const KDNode *nearest = NULL;

			KDTree_Nearest(vertices2[i], m_KDTree_sobel->root(), &nearest, &dist);

			// push back the corresponding point
			vec nn = nearest->split_point;
			corresponding_pts.push_back(Vector3d(nn.x, nn.y, nn.z));

			if (nn.x == 0 && nn.y == 0 && nn.z == 0) std::cout << "ZERO MATCH";

			LineSeg3D Line(vertices2[i].x, vertices2[i].y, vertices2[i].z,
				nn.x, nn.y, nn.z, Colour::green);

			Connecting_Lines.push_back(Line);

			total_dist += dist;
		}

		mean_dist = total_dist / n;

		//std::cout << "Number of points: " << n << endl;
		//std::cout << "Total distance: " << total_dist << endl;
		//std::cout << "Mean distance: " << mean_dist << endl;
		//std::cout << endl;

		// compute Rigid Transformation 
		TransformType RT;
		RT = computeRigidTransform(p1s, corresponding_pts);

		// update the metrices
		T_whole += RT.second;
		vectorR.push_back(RT.first);

		// move the points before run the next Rigid Transformation calculation
		for (int i = 0; i < vertices2.size(); i++) {
			if (vertices2[i].x == 0 && vertices2[i].y == 0 && vertices2[i].z == 0) continue;
			Vector3d vert;
			vert = RT.first* Vector3d(vertices2[i].x, vertices2[i].y, vertices2[i].z) + RT.second;
			vertices2[i].x = vert[0];
			vertices2[i].y = vert[1];
			vertices2[i].z = vert[2];
		}

		corresponding_pts.erase(corresponding_pts.begin(), corresponding_pts.end());
	}

	// Export the metrices with the total tranformation needed
	for (int u = 0; u < 3; u++) {
		for (int v = 0; v < 3; v++) {
			R_whole(u, v) = (u == v) ? 1 : 0;
		}
	}

	for (int k = vectorR.size() - 1; k > -1; k--) {
		R_whole *= vectorR[k];
	}
	/*
	//    Form KDTree with the entire veruces   //
	m_KDTree = NULL;
	VecArray m_vertices_unsorted = m_vertices;
	m_KDTree = new KDTree(m_vertices); // alli
	m_vertices = m_vertices_unsorted;
	*/

	/*
	//Calculate final error
	int counter = 0;
	float dist, total_dist = 0;
	for (int i = 0; i < m_vertices2.size(); i++) { //- (480 * 30 - 1)
		if (m_vertices2[i].x == 0 && m_vertices2[i].y == 0 && m_vertices2[i].z == 0) continue;
		const KDNode *nearest = NULL;
		KDTree_Nearest(m_vertices2[i], m_KDTree->root(), &nearest, &dist);
		total_dist += dist;
		counter += 1;
	}

	std::cout<<endl<< " Number of points " << counter << endl;
	std::cout << " Final Total distasnce " << total_dist << endl;
	std::cout << " Final Mean distance " << total_dist / counter << endl;
	*/
}

//using Eigen's SVD to fastly compute the rigid transformation between two point clouds.
TransformType computeRigidTransform(const PointsType& src, const PointsType& dst)
{
	assert(src.size() == dst.size());
	int pairSize = src.size();
	Eigen::Vector3d center_src(0, 0, 0), center_dst(0, 0, 0);

	// computes the centroids of the two pointclouds
	for (int i = 0; i < pairSize; ++i)
	{
		center_src += src[i];
		center_dst += dst[i];
	}
	center_src /= (double)pairSize;
	center_dst /= (double)pairSize;

	Eigen::MatrixXd S(pairSize, 3), D(pairSize, 3);

	// computes covariance matrix
	for (int i = 0; i < pairSize; ++i)
	{
		for (int j = 0; j < 3; ++j)

			S(i, j) = src[i][j] - center_src[j];

		for (int j = 0; j < 3; ++j)

			D(i, j) = dst[i][j] - center_dst[j];
	}

	Eigen::MatrixXd Dt = D.transpose();
	Eigen::Matrix3d H = Dt * S;
	Eigen::Matrix3d W, U, V;

	JacobiSVD<Eigen::MatrixXd> svd;
	Eigen::MatrixXd H_(3, 3);

	// Singular Values decomposition  
	for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) H_(i, j) = H(i, j);

	svd.compute(H_, Eigen::ComputeThinU | Eigen::ComputeThinV);

	if (!svd.computeU() || !svd.computeV()) {

		std::cerr << "decomposition error" << endl;
		return std::make_pair(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

	}

	Eigen::Matrix3d Vt = svd.matrixV().transpose();
	Eigen::Matrix3d R = svd.matrixU()*Vt;
	Eigen::Vector3d t = center_dst - R * center_src;

	return std::make_pair(R, t);
}
void vvr::Remove_dublicates(std::vector<vec> &vertices, std::vector<vec> &non_dublicate_vertices, vector<Vec3b> &non_dublicate_colours) {

	float dist, Threshold = pow(10, -4);
	for (int i = choice2 - choice; i > 0; i--) {
		m_KDTree = NULL;

		// Extracting the first subvector
		int m = single_image_size * i, n = single_image_size * (i + 1) - 1;

		std::vector<vec>  vertices1 = slice(m_vertices, m, n);
		std::vector<Vec3b> vertices1_colours = slice(RGB_colour, m, n);
		//std::cout << "m_vertices1 size: " << vertices1.size() << endl;

		// Extracting the KDTree subvector
		int m2 = single_image_size * (i - 1), n2 = single_image_size * i - 1;

		std::vector<vec>  verticesKD = slice(m_vertices, m2, n2);
		std::vector<Vec3b> verticesKD_colours = slice(RGB_colour, m2, n2);

		std::vector<vec> verticesKD_unsorted = verticesKD;
		m_KDTree = new KDTree(verticesKD);
		//    KDtree sort the vertices and changes the colour   // 
		if (i == 1) verticesKD = verticesKD_unsorted;

		for (int j = 0; j < vertices1.size(); j++) {
			//if (vertices1[j].x == 0 && vertices1[j].y == 0 && vertices1[j].z == 0) continue;

			const KDNode *nearest = NULL;
			KDTree_Nearest(vertices1[j], m_KDTree->root(), &nearest, &dist);

			if (dist > Threshold) {
				non_dublicate_vertices.push_back(vertices1[j]);
				non_dublicate_colours.push_back(vertices1_colours[j]);
			}
		}
		if (i == 1) {
			for (int j = 0; j < verticesKD.size(); j++) {
				non_dublicate_vertices.push_back(verticesKD[j]);
				non_dublicate_colours.push_back(verticesKD_colours[j]);
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////////
						// ÔASK4
//////////////////////////////////////////////////////////////////////////
// Computes the x component of the gradient vector
// at a given point in a image
// returns gradient in the x direction
int vvr::xGradient(const cv::Mat image, int x, int y)
{
	return image.at<uchar>(y - 1, x - 1) +
		2 * image.at<uchar>(y, x - 1) +
		image.at<uchar>(y + 1, x - 1) -
		image.at<uchar>(y - 1, x + 1) -
		2 * image.at<uchar>(y, x + 1) -
		image.at<uchar>(y + 1, x + 1);
}

// Computes the y component of the gradient vector
// at a given point in a image
// returns gradient in the y direction
int vvr::yGradient(const cv::Mat image, int x, int y)
{
	return image.at<uchar>(y - 1, x - 1) +
		2 * image.at<uchar>(y - 1, x) +
		image.at<uchar>(y - 1, x + 1) -
		image.at<uchar>(y + 1, x - 1) -
		2 * image.at<uchar>(y + 1, x) -
		image.at<uchar>(y + 1, x + 1);
}

void vvr::Sobel_filter(const cv::Mat& Image, cv::Mat& Filtered_Image)
{
	// Define the x and ycomponent of the gradient vector 
	int gx, gy, sum;

	// Load an image
	if (!Image.data)
	{
		// *always check* resource-loading.
		std::cout <<" No Image Loaded! "<<endl;
		return;
	}

	for (int y = 0; y < Image.rows; y++) {
		for (int x = 0; x < Image.cols; x++) {
			Filtered_Image.at<uchar>(y, x) = 0;		
		}
	}

	//Apply Sobel filter (3*3 window)
	for (int y = 1; y < Image.rows - 1; y++) {
		for (int x = 1; x < Image.cols - 1; x++) {
			gx = xGradient(Image, x, y);
			gy = yGradient(Image, x, y);
			sum = abs(gx) + abs(gy);
			sum = sum > 255 ? 255 : sum;
			sum = sum < 0 ? 0 : sum;
			Filtered_Image.at<uchar>(y, x) = sum;
		}
	}

	// Thresholding
	for (int y = 1; y < Image.rows - 1; y++) {
		for (int x = 1; x < Image.cols - 1; x++) {
			Filtered_Image.at<uchar>(y, x) = Filtered_Image.at<uchar>(y, x) > 180? 255 : 0;
		}
	}

	// Display original and filtered images
	//namedWindow("Original");
	//cv::imshow("Original", Image);

	//namedWindow("Filtered image");
	//cv::imshow("Filtered image", Filtered_Image);
}
void vvr::Sobel_ICP(const int count, vector<Mat> &Sobel_images, std::vector<vec> &vertices_first, std::vector<vec> &vertices_second, Matrix3d &R_whole, Vector3d &T_whole) {

	Mat Filtered_image1 = Sobel_images[count];
	Mat Filtered_image2 = Sobel_images[count + 1];

	////////////// sobel ICP //////////////
	vector<vec> m_vertices_sobel, m_vertice_2_sobel;
	vector<int> vertices2_indices;

	// keep only the the points that define edges 
	// we use only these point to find correspondance between frames
	for (int x = 0; x < Filtered_image1.cols; x++) {
		for (int y = 0; y < Filtered_image1.rows; y++) {
			int Index = x * Filtered_image1.rows + y;
			if (Filtered_image1.at<uchar>(y, x) == 255)	m_vertices_sobel.push_back(vertices_first[Index]);
			if (Filtered_image2.at<uchar>(y, x) == 255) {
				m_vertice_2_sobel.push_back(vertices_second[Index]);
				vertices2_indices.push_back(Index);
			}
		}
	}

	m_KDTree_sobel = NULL;
	//m_KDTree = NULL;
	//VecArray m_vertices_unsorted = vertices_first;
	//m_KDTree = new KDTree(vertices_first);
	m_KDTree_sobel = new KDTree(m_vertices_sobel);
	//    KDtree sort the vertices and changes the colour   // 
	//vertices_first = m_vertices_unsorted;

	vector<Vector3d> corresponding_pts;
	vector<Matrix3d>vectorR;

	T_whole = Vector3d(0, 0, 0);

	// Run ICP 10 times //
	for (int j = 0; j < 10; j++) {

		float total_dist = 0, mean_dist = 0;
		int n = 0;
		float dist;
		vector<Vector3d> p1s;

		for (int i = 0; i < m_vertice_2_sobel.size(); i++) {
			// if the point is at zero continue
			if (m_vertice_2_sobel[i].x == 0 && m_vertice_2_sobel[i].y == 0 && m_vertice_2_sobel[i].z == 0) continue;

			// counter used to calculate mean distance;
			n += 1;

			// vector of point that define edges
			p1s.push_back(Vector3d(m_vertice_2_sobel[i].x, m_vertice_2_sobel[i].y, m_vertice_2_sobel[i].z));

			// Find the corresponding point( the closest points for each psi point) using KDTree
			const KDNode *nearest = NULL;
			///high_resolution_clock::time_point t1 = high_resolution_clock::now();
			KDTree_Nearest(m_vertice_2_sobel[i], m_KDTree_sobel->root(), &nearest, &dist);
			///high_resolution_clock::time_point t2 = high_resolution_clock::now();
			///duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
			///std::cout << "KDTree_Nearest Time: " << time_span.count() << endl;

			// push back the corresponding point
			vec nn = nearest->split_point;
			corresponding_pts.push_back(Vector3d(nn.x, nn.y, nn.z));


			LineSeg3D Line_Sobel(m_vertice_2_sobel[i].x, m_vertice_2_sobel[i].y, m_vertice_2_sobel[i].z,
				nn.x, nn.y, nn.z, Colour::green);

			Connecting_Lines.push_back(Line_Sobel);

			total_dist += dist;
		}
		mean_dist = total_dist / n;

		// compute Rigid Transformation 
		TransformType RT;
		RT = computeRigidTransform(p1s, corresponding_pts);

		// update the metrices
		T_whole += RT.second;
		vectorR.push_back(RT.first);

		// move the points before run the next Rigid Transformation calculation
		for (int j = 0; j < vertices_second.size(); j++) {
			if (vertices_second[j].x == 0 && vertices_second[j].y == 0 && vertices_second[j].z == 0) continue;
			Vector3d vert;
			vert = RT.first* Vector3d(vertices_second[j].x, vertices_second[j].y, vertices_second[j].z) + RT.second;
			vertices_second[j].x = vert[0];
			vertices_second[j].y = vert[1];
			vertices_second[j].z = vert[2];
		}
		// erase the vectors before run the next Rigid Transformation calculation
		m_vertice_2_sobel.erase(m_vertice_2_sobel.begin(), m_vertice_2_sobel.end());
		corresponding_pts.erase(corresponding_pts.begin(), corresponding_pts.end());

		// update m_vertice_2 vertices to use them in the next iritation
		for (int j = 0; j < vertices2_indices.size(); j++) {
			m_vertice_2_sobel.push_back(vertices_second[vertices2_indices[j]]);
		}
	}
	/*
	//Calculate final error
	int counter = 0;
	float dist, total_dist = 0;
	for (int j = 0; j < vertices_second.size(); j++) { //- (480 * 30 - 1)
		if (vertices_second[j].x == 0 && vertices_second[j].y == 0 && vertices_second[j].z == 0) continue;
		const KDNode *nearest = NULL;
		KDTree_Nearest(vertices_second[j], m_KDTree->root(), &nearest, &dist);
		total_dist += dist;
		counter += 1;
	}
	*/
	// Export the metrices with the total tranformation needed
	for (int u = 0; u < 3; u++) {
		for (int v = 0; v < 3; v++) {
			R_whole(u, v) = (u == v) ? 1 : 0;
		}
	}

	for (int k = vectorR.size() - 1; k > -1; k--) {
		R_whole *= vectorR[k];
	}



	/*
	cout << endl;
	cout << " Number of points " << counter << endl;
	cout << " Final Total distasnce " << total_dist << endl;
	cout << " Final Mean distance " << total_dist / counter << endl;
	*/
}
//////////////////////////////////////////////////////////////////////////
						// ÔASK 5
//////////////////////////////////////////////////////////////////////////
void vvr::TriangulateMesh(const std::vector<vec>& vertices, vvr::Mesh*& mesh)
{
	int rows = RGB_image.rows;
	int size = vertices.size();

	Mesh *m_model = new Mesh();
	vector<vvr::Triangle> &tri = m_model->getTriangles();
	tri = {};
	std::vector<vec> &verts = m_model->getVertices();
	verts = vertices;
	vector<float> levels;

	// eksagosi vathoys
	/*
	for (int i = 0; i < size; i++) {
		float z = vertices[i].z;
		bool check = true;

		for (int j = 0; j < levels.size(); j++) {
			if (z == levels[j]) {
				check = false;
				break;
			}
		}
		if (check) levels.push_back(z);
	}
	*/
	vvr::Colour col(rand() * 255, rand() * 255, rand() * 255);

	for (int x = 0; x < RGB_image.cols - 1; x++) {
		for (int y = 0; y < RGB_image.rows - 1; y++) {


			//  A-------C
			//  |\		|
			//  |  \	|	
			//  |    \	|
			//  B-------D

			int index = x * RGB_image.rows + y;
			int next_x = (x + 1) * RGB_image.rows + y;
			int diagonal = (x + 1) * RGB_image.rows + y + 1;

			vec A = verts[index], B = verts[index + 1], C = verts[next_x], D = verts[diagonal];

			// if point is at zero continue
			if (((A.x == 0) && (A.y == 0) && (A.z == 0)) ||
				((B.x == 0) && (B.y == 0) && (B.z == 0)) ||
				((C.x == 0) && (C.y == 0) && (C.z == 0)) ||
				((D.x == 0) && (D.y == 0) && (D.z == 0))) {

				continue;
			}

			// define a theshold to avoide connecting point with big destance
			float thr{ 1 };
			if ((abs(A.z - D.z) < thr) && (abs(A.z - B.z) < thr) && (abs(A.z - C.z) < thr)) {

				tri.push_back(vvr::Triangle(&verts, index, index + 1, diagonal));
				tri.push_back(vvr::Triangle(&verts, index, next_x, diagonal));

			}
		}
	}
	mesh = m_model;
}
//    --------
//   /|      /|
//  p1------p2|
//  | |     | |
//  | |-----|-|
//  |/      |/
//  p3------p4

// calculates normal using outer product of two vectors
vec vvr::calcNormal(const vec &p1, const vec &p2, const vec &p3)
{
	vec V1 = (p2 - p1);
	vec V2 = (p3 - p1);
	vec surfaceNormal;

	surfaceNormal[0] = -((V1[1] * V2[2]) - (V1[2] * V2[1]));
	surfaceNormal[1] = ((V2[2] * V1[0]) - (V2[0] * V1[2]));
	surfaceNormal[2] = -((V1[0] * V2[1]) - (V1[1] * V2[0]));

	//normalize the normals
	double measure = sqrt(pow(abs(surfaceNormal[0]), 2) + pow(abs(surfaceNormal[1]), 2) + pow(abs(surfaceNormal[2]), 2));

	if (measure == 0) surfaceNormal = vec(0, 0, 0);
	else {
		surfaceNormal[0] /= measure;
		surfaceNormal[1] /= measure;
		surfaceNormal[2] /= measure;
	}

	return surfaceNormal;
}

void vvr::compute_Normals(const std::vector<vec>& m_vertices, vector<LineSeg3D>& PointCloudNormals) {

	vector<vec> all_the_normals;
	for (int x = 0; x < RGB_image.cols; x++) { //RGB_image.cols - 1
		for (int y = 0; y < RGB_image.rows; y++) { // RGB_image.rows - 1

			int index1 = x * RGB_image.rows + y;
			vec p1 = m_vertices[index1];

			vec normal;
			if ((x == RGB_image.cols - 1) || (y == RGB_image.rows - 1)) {
				normal = vec(0, 0, 0);
			}
			else if ((x == 0) || (y == 0)) {

				int index2 = (x + 1) *  RGB_image.rows + y;
				int index3 = x * RGB_image.rows + y + 1;
				vec p2 = m_vertices[index2];
				vec p3 = m_vertices[index3];

				normal = calcNormal(p1, p2, p3);
			}
			else {
				int index2 = (x + 1) *  RGB_image.rows + y;
				int index3 = x * RGB_image.rows + y + 1;
				int index4 = (x - 1) *  RGB_image.rows + y;
				int index5 = (x)*  RGB_image.rows + y - 1;

				vec p2 = m_vertices[index2];
				vec p3 = m_vertices[index3];
				vec p4 = m_vertices[index3];
				vec p5 = m_vertices[index3];

				vector<vec> normals;

				normals.push_back(calcNormal(p1, p2, p3));
				normals.push_back(calcNormal(p1, p3, p4));
				normals.push_back(calcNormal(p1, p4, p5));
				normals.push_back(calcNormal(p1, p5, p1));

				float total_x = 0, total_y = 0, total_z = 0;

				for (int j = 0; j < normals.size(); j++) {
					total_x += normals[j].x;
					total_y += normals[j].y;
					total_z += normals[j].z;
				}
				//int n = normals.size();
				normal = vec(total_x, total_y, total_z);

				//normalize the normal
				double measure = sqrt(pow(abs(normal[0]), 2) + pow(abs(normal[1]), 2) + pow(abs(normal[2]), 2));

				if (measure == 0) normal = vec(0, 0, 0);
				else {
					normal[0] /= measure;
					normal[1] /= measure;
					normal[2] /= measure;
				}
			}

			all_the_normals.push_back(normal);

			///// Normal Map ///////

			////X: -1 to + 1 : Red : 0 to 255
			int R = (normal[0] + 1) * 255 / 2;
			////Y: -1 to + 1 : Green : 0 to 255
			int G = (normal[1] + 1) * 255 / 2;
			//Z : 0 to - 1 : Blue : 128 to 255
			int B = (1 - normal[2]) * 128 - 1;

			//export normal as a line segament
			LineSeg3D Normal = LineSeg3D(p1[0], p1[1], p1[2],
				p1[0] + normal[0], p1[1] + normal[1],
				p1[2] + normal[2], vvr::Colour(R, G, B));

			PointCloudNormals.push_back(Normal);
		}
	}
}

void vvr::computeCovarianceMatrix(std::vector<vec> points, Eigen::MatrixXd &covarianceMatrix) {

	double means[3] = { 0, 0, 0 };

	for (int i = 0; i < points.size(); i++)

		means[0] += points[i].x,

		means[1] += points[i].y,

		means[2] += points[i].z;

	means[0] /= points.size(), means[1] /= points.size(), means[2] /= points.size();

	for (int i = 0; i < 3; i++)

		for (int j = 0; j < 3; j++) {

			covarianceMatrix(i, j) = 0.0;

			for (int k = 0; k < points.size(); k++) {
				vec p = points[k];
				covarianceMatrix(i, j) += (means[i] - p[i]) *

					(means[j] - p[j]);
			}
			covarianceMatrix(i, j) /= points.size() - 1;
		}
}

void vvr::Covariance_Normals(std::vector<vec> m_vertices, vector<LineSeg3D>& PointCloudNormals) {

	vector<vec> all_the_normals;

	float time1 = getSeconds();
	//m_KDTree_original = new KDTree(m_vertices);

	VecArray m_vertices_unsorted = m_vertices;
	m_KDTree = NULL;
	m_KDTree = new KDTree(m_vertices);
	m_vertices = m_vertices_unsorted;

	for (int i = 0; i < m_vertices.size(); i++) {
		float best_dist;

		const KDNode **knn = new const KDNode*[8];
		memset(knn, NULL, 8 * sizeof(KDNode*));
		NearestK(8, m_vertices[i], m_KDTree->root(), knn, &best_dist);
		std::vector<vec> points;

		points.push_back(m_vertices[i]);
		for (int i = 0; i < 8; i++) points.push_back(knn[i]->split_point);

		Eigen::MatrixXd  covarianceMatrix(3, 3);
		computeCovarianceMatrix(points, covarianceMatrix);

		EigenSolver<MatrixXd> es(covarianceMatrix);

		VectorXd v = es.eigenvectors().col(0).real();
		//cout << "eigen vector size: " << v.size() << endl;

		all_the_normals.push_back(vec(v[0], v[1], v[2]));
	}
	///// Normal Map ///////
	for (int j = 0; j < m_vertices.size(); j++) {
		vec p1 = m_vertices[j];
		vec normal = all_the_normals[j];
		//X: -1 to + 1 : Red : 0 to 255
		int R = (normal[0] + 1) * 255 / 2;
		////Y: -1 to + 1 : Green : 0 to 255
		int G = (normal[1] + 1) * 255 / 2;
		////Z : 0 to - 1 : Blue : 128 to 255
		int B = (1 - normal[2]) * 128 - 1;

		//export normal as a line segament
		LineSeg3D Normal = LineSeg3D(p1[0], p1[1], p1[2],
			p1[0] + normal[0], p1[1] + normal[1],
			p1[2] + normal[2], vvr::Colour(R, G, B));
		PointCloudNormals.push_back(Normal);
	}

	time1 = getSeconds() - time1;

	std::cout << "normal 2 estimation Time: " << time1 << endl;
	std::cout << "normal size: " << PointCloudNormals.size() << endl;

}
//////////////////////////////////////////////////////////////////////////
						// KDTree
//////////////////////////////////////////////////////////////////////////
void vvr::NearestK(const int k, const vec& test_pt, const KDNode* root, const KDNode **knn, float *best_dist)
{
	if (!root) return;

	const double d = test_pt.DistanceSq(root->split_point);
	const double d_split = root->split_point.ptr()[root->axis] - test_pt.ptr()[root->axis];
	const bool right_of_split = d_split <= 0;

	int index = k - 1;

	for (int j = 0; j < k; j++) {
		if (knn[j] != NULL) {
			if ((knn[j]->split_point.DistanceSq(test_pt)) > d) {
				for (int count = k - 1; count > j; count--) {

					knn[count] = knn[count - 1];

				}
				knn[j] = root;
				break;
			}
		}
		if (knn[j] == NULL) {
			knn[j] = root; break;
		}
	}
	for (int i = 0; i < k; i++) {
		if (knn[i] == NULL) {
			index = i - 1; break;
		}
	}

	*best_dist = test_pt.DistanceSq(knn[index]->split_point);

	NearestK(k, test_pt, right_of_split ? root->child_right : root->child_left, knn, best_dist);

	if (SQUARE(d_split) >= *best_dist) return;

	NearestK(k, test_pt, right_of_split ? root->child_left : root->child_right, knn, best_dist);
}

KDTree::KDTree(VecArray &pts)
	: pts(pts)
{
	const float t = vvr::getSeconds();
	m_root = new KDNode();
	m_depth = makeNode(m_root, pts, 0);
	const float KDTree_construction_time = vvr::getSeconds() - t;
	echo(KDTree_construction_time);
	echo(m_depth);
}

KDTree::~KDTree()
{
	const float t = vvr::getSeconds();
	delete m_root;
	const float KDTree_destruction_time = vvr::getSeconds() - t;
	echo(KDTree_destruction_time);
}

int KDTree::makeNode(KDNode *node, VecArray &pts, const int level)
{
	//! Sort along the appropriate axis, find median point and split.
	const int axis = level % DIMENSIONS;
	std::sort(pts.begin(), pts.end(), VecComparator(axis));
	const int i_median = pts.size() / 2;

	//! Set node members
	node->level = level;
	node->axis = axis;
	node->split_point = pts[i_median];
	node->aabb.SetFrom(&pts[0], pts.size());

	//! Continue recursively or stop.
	if (pts.size() <= 1)
	{
		return level;
	}
	else
	{
		int level_left = 0;
		int level_right = 0;
		VecArray pts_left(pts.begin(), pts.begin() + i_median);
		VecArray pts_right(pts.begin() + i_median + 1, pts.end());

		if (!pts_left.empty())
		{
			node->child_left = new KDNode();
			level_left = makeNode(node->child_left, pts_left, level + 1);

		}
		if (!pts_right.empty())
		{
			node->child_right = new KDNode();
			level_right = makeNode(node->child_right, pts_right, level + 1);
		}

		int max_level = std::max(level_left, level_right);
		return max_level;
	}
}

void KDTree::getNodesOfLevel(KDNode *node, std::vector<KDNode*> &nodes, int level)
{
	if (!level)
	{
		nodes.push_back(node);
	}
	else
	{
		if (node->child_left) getNodesOfLevel(node->child_left, nodes, level - 1);
		if (node->child_right) getNodesOfLevel(node->child_right, nodes, level - 1);
	}
}

std::vector<KDNode*> KDTree::getNodesOfLevel(const int level)
{
	std::vector<KDNode*> nodes;
	if (!m_root) return nodes;
	getNodesOfLevel(m_root, nodes, level);
	return nodes;

}

void vvr::KDTree_Nearest(const vec& test_pt, const KDNode* root, const KDNode **nn, float *best_dist)
{
	if (!root) return; // otan ginei NULL epistrefei

	//Distances!
	const double d = test_pt.DistanceSq(root->split_point);
	const double d_split = root->split_point.ptr()[root->axis] - test_pt.ptr()[root->axis];
	const bool right_of_split = d_split <= 0;

	if (*nn == NULL || d < *best_dist) {
		*best_dist = d;
		*nn = root;
	}
	//Searching
	KDTree_Nearest(test_pt, right_of_split ? root->child_right : root->child_left, nn, best_dist);

	//Pruning
	if (SQUARE(d_split) >= *best_dist) return;
	//else
	KDTree_Nearest(test_pt, right_of_split ? root->child_left : root->child_right, nn, best_dist);
}

//////////////////////////////////////////////////////////////////////////
						// Not Uded Functions
//////////////////////////////////////////////////////////////////////////
void vvr::cuvature_estimation(std::vector<vec> all_the_normals, vector<float>& pointCloud_curvature) {

	vector<vec> m_vertices_original = m_vertices;
	m_KDTree = NULL;
	m_KDTree = new KDTree(m_vertices);
	m_vertices = m_vertices_original;

	for (int index = 0; index < all_the_normals.size(); index++) {

		//cout << index << endl;
		if ((all_the_normals[index].x == NAN) || (all_the_normals[index].y == NAN) || (all_the_normals[index].z == NAN)) continue;

		pointCloud_curvature.push_back(compute_curvature(m_vertices[index], all_the_normals[index]));
	}

	// normalize curvature
	vector<float> unsorted = pointCloud_curvature;
	std::sort(pointCloud_curvature.begin(), pointCloud_curvature.end());

	float min = pointCloud_curvature[0];
	float max = pointCloud_curvature[pointCloud_curvature.size() - 1];
	pointCloud_curvature = unsorted;

	float average = computeMean(pointCloud_curvature);
	float variance = computeVariance(average, pointCloud_curvature);

	std::cout << "average " << average << endl;
	std::cout << "variance " << variance << endl;

	for (int i = 0; i < pointCloud_curvature.size(); i++) {
		pointCloud_curvature[i] = (pointCloud_curvature[i] - min) / (max - min);
		//cout << pointCloud_curvature[i] << endl;
	}
}

float vvr::angle_of_vectors(vec& u, vec& v)
{
	float u_measure = sqrt(pow(u[0], 2) + pow(u[1], 2) + pow(u[2], 2));
	float v_measure = sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));

	float inner_product = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];

	float vec_cos = inner_product / ((u_measure * v_measure)*1.0);

	float angle = acos(vec_cos);

	return angle;
}

float vvr::compute_curvature(const vec &p, vec& normal) {

	float best_dist;

	const KDNode **knn = new const KDNode*[8];
	memset(knn, NULL, 8 * sizeof(KDNode*));

	NearestK(8, p, m_KDTree->root(), knn, &best_dist);

	float curvature = 0;

	for (int i = 0; i < 8; i++) {

		float x_dif = knn[i]->split_point.x - p.x;
		float y_dif = knn[i]->split_point.y - p.y;
		float z_dif = knn[i]->split_point.z - p.z;

		double measure = sqrt(pow(abs(x_dif), 2) + pow(abs(y_dif), 2) + pow(abs(z_dif), 2));

		//cout << measure<< endl;

		if (measure == 0) curvature = 0;
		else {
			x_dif /= measure;
			y_dif /= measure;
			z_dif /= measure;

			//cout << x_dif << endl;
			curvature += abs(x_dif * normal[0] + y_dif * normal[1] + z_dif * normal[2]);
		}
	}
	//cout << curvature << endl;
	return curvature;
}

float vvr::computeMean(std::vector<float> numbers)
{
	if (numbers.empty()) return 0;

	float total = 0;
	for (int number : numbers) {
		total += number;
	}

	return (total / numbers.size());
}

float vvr::computeVariance(float mean, std::vector<float> numbers)
{
	float result = 0;
	for (int number : numbers)
	{
		result += (number - mean)*(number - mean);
	}

	return result / (numbers.size() - 1);
}

void vvr::Colour_connected_mesh_vetices(vvr::Mesh* mesh, vector<Point3D> &segemnted_vertices) {

	vector<vvr::Triangle> &tri = mesh->getTriangles();
	std::vector<vec> &verts = mesh->getVertices();

	while (tri.size() > 0) {
		bool enter = true;
		vvr::Colour col(rand() * 255, rand() * 255, rand() * 255);
		unsigned tri_adj_index1 = 0;
		vector<unsigned> tri_adj_index;

		//vvr::Triangle tri_i(0,0,0);
		//tri_i.get_indec
		vvr::Triangle tri_i = tri[0];
		while (enter) {
			enter = FindAdjacentTriangle(tri_i, tri, tri_adj_index);

			if (enter == false) cout << "false!!" << endl;
			if (tri.size() <= 0) break;

			//cout <<"size "<<tri_adj_index.size() << endl;

			// an vreiw trigono xromatise ta simeia kai svisto
			if (enter) {

				sort(tri_adj_index.begin(), tri_adj_index.end(), std::greater<unsigned>());
				for (int j = 0; j < tri_adj_index.size(); j++) {

					tri_i = tri[tri_adj_index[j]];
					Point3D new_point1(tri_i.v1()[0], tri_i.v1()[1], tri_i.v1()[2], col);
					Point3D new_point2(tri_i.v2()[0], tri_i.v2()[1], tri_i.v2()[2], col);
					Point3D new_point3(tri_i.v3()[0], tri_i.v3()[1], tri_i.v3()[2], col);

					segemnted_vertices.push_back(new_point1);
					segemnted_vertices.push_back(new_point2);
					segemnted_vertices.push_back(new_point3);

					// erase found tringle 
					//tri_i = tri[tri_adj_index[0]];

					tri.erase(tri.begin() + tri_adj_index[j]);
					cout << tri.size() << endl;
				}
				//tri_i = tri[tri_adj_index[0]];
			}
			//else {

				/*
				/////////////////////
				cout << "in else" << endl;
				tri_i = tri[tri_adj_index-30];
				enter = FindAdjacentTriangle(tri_i, tri, tri_adj_index);

				if (enter) {
					Point3D new_point1(tri_i.v1()[0], tri_i.v1()[1], tri_i.v1()[2], col);
					Point3D new_point2(tri_i.v2()[0], tri_i.v2()[1], tri_i.v2()[2], col);
					Point3D new_point3(tri_i.v3()[0], tri_i.v3()[1], tri_i.v3()[2], col);

					segemnted_vertices.push_back(new_point1);
					segemnted_vertices.push_back(new_point2);
					segemnted_vertices.push_back(new_point3);

					tri_i = tri[tri_adj_index];

					// erase found tringle
					tri.erase(tri.begin() + tri_adj_index);
					cout << "jumped "<<tri.size() << endl;
				}
				*/
				//}

		}
	}
}

bool vvr::FindAdjacentTriangle(vvr::Triangle &tri_i, vector<vvr::Triangle> &tris, vector<unsigned>& tri_adj_index)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Vreite ena trigwno pou exei thn pleura p2-p3 || p3-p2 kai apothikeuste to index `i` sto `tri_adj_index
	//!    kai to mh-koino shmeio sto `opp_ver`
	//!
	//!//////////////////////////////////////////////////////////////////////////////////

	tri_adj_index.clear();
	static bool found = false;
	for (int i = 0; i < tris.size(); i++) {

		vec  tri_v1 = tris[i].v1();
		vec  tri_v2 = tris[i].v2();
		vec  tri_v3 = tris[i].v3();


		if (tri_adj_index.size() == 3) break;
		if (((tri_v1[0] == tri_i.v1()[0]) && (tri_v1[1] == tri_i.v1()[1]) && (tri_v1[2] == tri_i.v1()[2])) ||
			((tri_v2[0] == tri_i.v1()[0]) && (tri_v2[1] == tri_i.v1()[1]) && (tri_v2[2] == tri_i.v1()[2])) ||
			((tri_v3[0] == tri_i.v1()[0]) && (tri_v3[1] == tri_i.v1()[1]) && (tri_v3[2] == tri_i.v1()[2]))) {

			tri_adj_index.push_back(i);
			found = true;
			continue;
		}
		if (((tri_v1[0] == tri_i.v2()[0]) && (tri_v1[1] == tri_i.v2()[1]) && (tri_v1[2] == tri_i.v2()[2])) ||
			((tri_v2[0] == tri_i.v2()[0]) && (tri_v2[1] == tri_i.v2()[1]) && (tri_v2[2] == tri_i.v2()[2])) ||
			((tri_v3[0] == tri_i.v2()[0]) && (tri_v3[1] == tri_i.v2()[1]) && (tri_v3[2] == tri_i.v2()[2]))) {

			tri_adj_index.push_back(i);
			found = true;
			continue;
		}
		if (((tri_v1[0] == tri_i.v3()[0]) && (tri_v1[1] == tri_i.v3()[1]) && (tri_v1[2] == tri_i.v3()[2])) ||
			((tri_v2[0] == tri_i.v3()[0]) && (tri_v2[1] == tri_i.v3()[1]) && (tri_v2[2] == tri_i.v3()[2])) ||
			((tri_v3[0] == tri_i.v3()[0]) && (tri_v3[1] == tri_i.v3()[1]) && (tri_v3[2] == tri_i.v3()[2]))) {

			tri_adj_index.push_back(i);
			found = true;
			continue;
		}
	}

	if (found) return true;
	return false;
}

void Task_2_CalculateMatching2(const cv::Mat& leftImage, const cv::Mat& rightImage, std::vector<vec> &vertices, std::vector<vec> &vertices2) {
	int window_size = 15;
	cv::Rect line_roi(80, 30, 480, 420);
	Mat img_2_search_area = rightImage(line_roi);
	cv::Rect img_roi(100, 100, window_size, window_size);
	Mat img_1_block = leftImage(img_roi);

	Mat result;
	matchTemplate(img_2_search_area, img_1_block, result, 5); // to 5 einai methodos // img: 8-bit or 32 bit floating

	double max;
	cv::Point best_match;
	minMaxLoc(result, nullptr, &max, nullptr, &best_match);

	int index = (100 + window_size / 2)*rightImage.rows + (100 + window_size / 2); //to simeio sto verices 1
	int index2 = (best_match.x + window_size / 2)* rightImage.rows + (best_match.y + window_size / 2);
	
	for (int i = 0; i < vertices.size(); i++) {
		vec vert = vertices[i];
		Vec3b color = RGB_colour[i];
		vvr::Point3D(vert.x, vert.y, vert.z, vvr::Colour(color[2], color[1], color[0])).draw();
	};
	for (int i = 0; i < vertices2.size(); i++) {
		vec vert = vertices2[i];
		Vec3b color = RGB_colour2[i];
		vvr::Point3D(vert.x, vert.y, vert.z, vvr::Colour(color[2], color[1], color[0])).draw();
	}
	vvr::Point3D(vertices[index].x, vertices[index].y, vertices[index].z, Colour::red).draw(); //vvr::Colour(color[2], color[1], color[0])      Colour::blue
	vvr::Point3D(vertices2[index2].x, vertices2[index2].y, vertices2[index2].z, Colour::red).draw();

}

void vvr::Task_2_CalculateMatching(const cv::Mat& leftImage ,const cv::Mat& rightImage)
{
	double max;
	int window_size = 15;

	cv::Rect line_roi(80, 30, 480, 420);
	//cv::Rect line_roi(0, 0, rightImage.cols, rightImage.rows);
	Mat img_2_search_area = rightImage(line_roi);
	vector<Vector3d> p1s, corresponding_pts;
	float total_dist = 0, mean_dist = 0;
	int n = 0;
	for (int i = 100; i < rightImage.cols - 100; i+=10) {// rightImage.cols - window_size
		for (int j = 50; j < rightImage.rows -50; j+=10) { // prosoxi sti prospelasi an einai me geometriko sxima (point ) i i,j

			//if (depth_image.at<ushort>(j, i) == 0) continue;
			cv::Rect img_roi(i, j, window_size, window_size);
			Mat img_1_block = leftImage(img_roi);
			//mg_1_block.convertTo(img_1_block, CV_32F);
			/*
			//cv::Point best_match;
			int k = 15;
			
			int row_index;
			//vector<double> Max_upper_j;
			//vector<double> Max_under_j;
			//vector<cv::Point>best_matches_upper_j;
			//vector<cv::Point>best_matches_under_j;
			bool check = false;
			for (int  r = k / 2; r > 0 ; r--) {

				if (j - r < 0) {
					continue;
				}else{
					row_index = r;
					check = true;
					cv::Rect line_roi(0, j-r, rightImage.cols , (3*window_size));
					Mat img_2_search_area = leftImage(line_roi);

					Mat result;
					matchTemplate(img_2_search_area, img_1_block, result, 5);
					cv::Point best_match;

					minMaxLoc(result, nullptr, &max, nullptr, &best_match);
					//Max_upper_j.push_back(max);
					//best_matches_upper_j.push_back(best_match);
				}
			}
			if (!check) {
				
				cv::Rect line_roi(0,0, rightImage.rows, min(3*window_size, rightImage.rows - (j)));
				Mat img_2_search_area = leftImage(line_roi);

				Mat result;
				matchTemplate(img_2_search_area, img_1_block, result, 5); // to 5 einai methodos // img: 8-bit or 32 bit floating

				cv::Point best_match;

				minMaxLoc(result, nullptr, &max, nullptr, &best_match);
				//Max_under_j.push_back(max);
				//best_matches_under_j.push_back(best_match);
				
			}		
			if(check && max> 0.98){
				int index = (i)*rightImage.rows + (j);
				int index2 = (best_match.x )* rightImage.rows +(best_match.y + j - row_index);
				LineSeg3D Line(
					vertices[index].x,
					vertices[index].y,
					vertices[index].z,
					vertices2[index2].x,
					vertices2[index2].y,
					vertices2[index2].z, Colour::green);

				Lines.push_back(Line);
			}
			else if(max > 0.98) {
			
				int index = (i)*rightImage.rows + (j);
				int index2 = (best_match.x )* rightImage.rows + (best_match.y);

				LineSeg3D Line(
					vertices[index].x,
					vertices[index].y,
					vertices[index].z,
					vertices2[index2].x,
					vertices2[index2].y,
					vertices2[index2].z, Colour::green);

				Lines.push_back(Line);
			}
			*/
			
			//cv::Rect line_roi(0, 0, rightImage.cols ,rightImage.rows );
			//Mat img_2_search_area = rightImage(line_roi);

			Mat result;
			matchTemplate(img_2_search_area, img_1_block, result, 5); // to 5 einai methodos // img: 8-bit or 32 bit floating
			
			cv::Point best_match;
			minMaxLoc(result, nullptr, &max, nullptr, &best_match);

			//cout << result.cols << endl
			//cout << "max " << max << endl;
		
			if (max > double(0.95)){
				n += 1;
				int index = (i + window_size/2)*rightImage.rows + (j + window_size / 2);
				int index2 = (best_match.x + window_size / 2)* rightImage.rows +(best_match.y + window_size / 2);
				
				LineSeg3D Line(
					m_vertices[index].x,
					m_vertices[index].y,
					m_vertices[index].z,
					m_vertices2[index2].x,
					m_vertices2[index2].y,
					m_vertices2[index2].z, Colour::green);
			
				Connecting_Lines.push_back(Line);
				p1s.push_back(Vector3d(m_vertices[index].x, m_vertices[index].y ,m_vertices[index].z));
				corresponding_pts.push_back(Vector3d(m_vertices2[index2].x, m_vertices2[index2].y, m_vertices2[index2].z));
				total_dist += Distance(m_vertices[index], m_vertices2[index2]);
			}	
		}
	}

	mean_dist = total_dist / n;
	std::cout << "Number of points: " << n << endl;
	std::cout << "Total distance: " << total_dist << endl;
	std::cout << "Mean distance: " << mean_dist << endl;
	std::cout << "Number of lines: " << Connecting_Lines.size() << endl;
	std::cout << endl;

	TransformType RT;
	RT = computeRigidTransform(corresponding_pts,p1s);
	std::cout << RT.first << endl;
	std::cout << (RT.second)[0] << "  " << (RT.second)[1] << "  " << (RT.second)[2] << endl;

	for (int i = 0; i < m_vertices2.size(); i++) {
		Vector3d vert;
		vert = RT.first* Vector3d(m_vertices2[i].x, m_vertices2[i].y, m_vertices2[i].z) + RT.second;
		m_vertices2[i].x = vert[0];
		m_vertices2[i].y = vert[1];
		m_vertices2[i].z = vert[2];
	}

	corresponding_pts.erase(corresponding_pts.begin(), corresponding_pts.end());

	//Calculate final error
	VecArray m_vertices_unsorted = m_vertices;;
	m_KDTree = new KDTree(m_vertices);
	m_vertices = m_vertices_unsorted;
	int counter = 0;
	float dist2, total_dist2 = 0;
	for (int i = 0; i < m_vertices2.size(); i++) { //- (480 * 30 - 1)
		const KDNode *nearest = NULL;
		KDTree_Nearest(m_vertices2[i], m_KDTree->root(), &nearest, &dist2);
		total_dist2 += dist2;
		counter += 1;
	}

	std::cout << endl;
	std::cout << " Number of points " << counter << endl;
	std::cout << " Final Total distasnce " << total_dist << endl;
	std::cout << " Final Mean distance " << total_dist / counter << endl;
}

void vvr::Compute_center_of_mass(std::vector<vec> &vertices, vec &cOm) {
	float xsum = 0, ysum = 0, zsum = 0, N = 0;
	for (int i = 0; i < vertices.size(); i++) {
		if (vertices[i].x == 0 && vertices[i].y == 0 && vertices[i].z == 0) continue;
		N += 1;
		xsum += vertices[i].x;
		ysum += vertices[i].y;
		zsum += vertices[i].z;
		cOm = vec(double(xsum)/N , double(ysum)/N, double(zsum)/N);
	}
}

