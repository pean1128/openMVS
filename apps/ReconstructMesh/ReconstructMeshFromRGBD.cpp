/*
 * ReconstructMesh.cpp
 *
 * Copyright (c) 2014-2015 SEACAVE
 *
 * Author(s):
 *
 *      cDc <cdc.seacave@gmail.com>
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Additional Terms:
 *
 *      You are required to preserve legal notices and author attributions in
 *      that material or in the Appropriate Legal Notices displayed by works
 *      containing it.
 */

#include "../../libs/MVS/Common.h"
#include "../../libs/MVS/Scene.h"
#include "../../libs/IO/json.hpp"
#include <boost/program_options.hpp>

#define READ_ACMM_DATA 0

using namespace MVS;
using json = nlohmann::json;

// D E F I N E S ///////////////////////////////////////////////////

#define APPNAME _T("ReconstructMesh")

// uncomment to enable multi-threading based on OpenMP
#ifdef _USE_OPENMP
#define RECMESH_USE_OPENMP
#endif


// S T R U C T S ///////////////////////////////////////////////////

namespace {

namespace OPT {
String strInputFileName;
String strOutputFileName;
String strMeshFileName;
bool bMeshExport;
float fDistInsert;
bool bUseConstantWeight;
bool bUseFreeSpaceSupport;
float fThicknessFactor;
float fQualityFactor;
float fDecimateMesh;
unsigned nTargetFaceNum;
float fRemoveSpurious;
bool bRemoveSpikes;
unsigned nCloseHoles;
unsigned nSmoothMesh;
float fSplitMaxArea;
unsigned nArchiveType;
int nProcessPriority;
unsigned nMaxThreads;
String strImagePointsFileName;
String strExportType;
String strConfigFileName;
boost::program_options::variables_map vm;
} // namespace OPT

bool ReadPlyAsPointCloud(
	const std::string& filename, 
	PointCloud& pointcloud)
{
    if (0 != access(filename.c_str(), F_OK)) {
        printf("file %s not exists", filename.c_str());
        return false;
    }

    std::ifstream f(filename);
    std::string line;

    std::getline(f, line);
    if (line != "ply") {
        printf("file format is invalid");
        f.close();
        return false;
    }

    std::getline(f, line);
    if (line != "format ascii 1.0") {
        printf("ply format should be ascii");
        f.close();
        return false;
    }

    std::getline(f, line);
	if (line.rfind("comment VCGLIB generated") == 0) {
    	std::getline(f, line);
	}

    int vertex_count = 0;
    if (line.rfind("element vertex") == 0) {
        vertex_count = std::stoi(line.substr(15));
    }

	/*
		property float x
		property float y
		property float z
		property float nx
		property float ny
		property float nz
		property uchar red
		property uchar green
		property uchar blue
		(property uchar alpha)
	*/
	int idx = 0;
	bool finalize = false;
	for (int i = 0; i < 12; ++i) {
		std::getline(f, line);

		if (line.rfind("element face") == 0) {
			idx = i;
			break;
		} else if (line.rfind("end_header") == 0) {
			idx = i;
			finalize = true;
			break;
		}
	}

	if (!finalize)
	{
		std::getline(f, line);
		std::getline(f, line);

		if (line.rfind("end_header") != 0) {
			printf("[ReadPlyAsPointCloud] read point cloud file fail\n");
			f.close();
			exit(EXIT_FAILURE);
		}
	}
    
    while (std::getline(f, line)) 
	{
        std::stringstream ss(line);

        std::vector<std::string> line_buf;
        std::for_each(
            std::istream_iterator<std::string>(ss), 
            std::istream_iterator<std::string>(), 
            [&](std::string const& word) 
			{ 
				line_buf.emplace_back(word);
			}
        );

		if (line_buf.size() == 10) 
		{
			PointCloud::Point& point = pointcloud.points.AddEmpty();
			point[0] = std::stof(line_buf[0]), 
			point[1] = std::stof(line_buf[1]), 
			point[2] = std::stof(line_buf[2]);

			// PointCloud::ViewArr& point_view = pointcloud.pointViews.AddEmpty();
			// point_view.Resize(1);
			// point_view[0] = view_id;

			// PointCloud::WeightArr& point_weight = pointcloud.pointWeights.AddEmpty();
			// point_weight.Resize(1);
			// point_weight[0] = 1.0;
		} 
		else if (line_buf.size() == 9) 
		{
			PointCloud::Point& point = pointcloud.points.AddEmpty();
			point[0] = std::stof(line_buf[0]), 
			point[1] = std::stof(line_buf[1]), 
			point[2] = std::stof(line_buf[2]);

			// std::cerr << point[0] << " " << point[1] << " " << point[2] << std::endl;
			// exit(-1);
		} 
		else 
		{
			std::cerr << line << std::endl;
			exit(EXIT_FAILURE);
		}
    }
    f.close();

	ASSERT(pointcloud.points.size() == vertex_count);

    printf("vertice size of pointcloud: %lu", pointcloud.points.size());
    return true;
}

bool ReadPointCloudDetails(
	const std::string& filename, 
	Scene& scene)
{
	std::ifstream fin(filename);
    if (!fin.is_open())
    {
        printf("%s can not open", filename.c_str());
        return false;
    }

    json pcd_details = json::parse(fin);
    auto view_amount = pcd_details["depth_view_details"]["amount"].get<int>();
    auto average_depths = pcd_details["depth_view_details"]["average_depth"].get<std::vector<float>>(); 

	std::cerr << view_amount << std::endl;
	std::cerr << average_depths.size() << std::endl;
	ASSERT(view_amount == scene.images.size());

	for (int i = 0; i < view_amount; i++)
	{
		scene.images[i].avgDepth = average_depths[i];
	}
	std::vector<float>().swap(average_depths);

    auto pcd_amount = pcd_details["point_cloud_details"]["amount"].get<int>();
	auto joint_view_ids = pcd_details["point_cloud_details"]["joint_view_ids"].get<std::vector<std::vector<int>>>(); 

	ASSERT(pcd_amount == scene.pointcloud.points.size());

	for (int i = 0; i < pcd_amount; i++)
	{
		PointCloud::ViewArr& point_view = scene.pointcloud.pointViews.AddEmpty();
		point_view.Resize(joint_view_ids[i].size());

		PointCloud::WeightArr& point_weight = scene.pointcloud.pointWeights.AddEmpty();
		point_weight.Resize(joint_view_ids[i].size());

		if (joint_view_ids[i].size() < 1)
		{
			printf("warning: receive empty joint_view_ids\n");
		}

		for (int j = 0; j < joint_view_ids[i].size(); j++)
		{
			point_view[j] = joint_view_ids[i][j];
			point_weight[j] = 1.0;
		}
	}

	std::vector<std::vector<int>>().swap(joint_view_ids);

	fin.close(); 

	return true;
}

void AppendPointCloudFromDepthImage(
	const cv::Mat &depth,
	const Eigen::Matrix3f &intrinsic,
	const Eigen::Matrix3f &rotation,
	const Eigen::Vector3f &center,
	float depth_scale, 
	float min_depth, float max_depth,
	int stride, int view_id, 
	PointCloud& pointcloud) 
{
    if (depth.empty() || depth.type() != CV_16UC1) 
	{
        printf("[AppendPointCloudFromDepthImage] invalid depth data to pointcloud");
        std::abort();
    }

    int depth_width = static_cast<int>(depth.cols);
    int depth_height = static_cast<int>(depth.rows);

    for (int v = 0; v < depth_height; v += stride) 
	{
    	for (int u = 0; u < depth_width; u += stride) 
		{
			float z = 1.0 / depth_scale * static_cast<float>(depth.at<ushort>(v, u));

			if (z > min_depth && z < max_depth) 
			{
				float x = (u - intrinsic(0, 2)) * z / intrinsic(0, 0);
            	float y = (v - intrinsic(1, 2)) * z / intrinsic(1, 1);

            	Eigen::Vector3f pt3d = rotation.inverse() * Eigen::Vector3f(x, y, z) + center; 

				PointCloud::Point& point = pointcloud.points.AddEmpty();
				point[0] = pt3d[0], point[1] = pt3d[1], point[2] = pt3d[2];

				PointCloud::ViewArr& point_view = pointcloud.pointViews.AddEmpty();
				point_view.Resize(1);
				point_view[0] = view_id;

				PointCloud::WeightArr& point_weight = pointcloud.pointWeights.AddEmpty();
				point_weight.Resize(1);
				point_weight[0] = 1.0;
        	}
		}
    }
}

bool GetFilesUsingSuffix(
	const std::string &dir_path, 
	const std::string &suffix,  
	std::vector<std::string>& filenames) 
{
	filenames.clear();
	DIR *dir;
	struct dirent *entry;
	dir = opendir(dir_path.c_str());
	if (dir != NULL) {
		while (entry = readdir(dir)) {
			if (strcmp(entry->d_name, ".") && strcmp(entry->d_name, "..")) {
				if (entry->d_type == DT_DIR)
					continue;
				if (std::string(entry->d_name).find(suffix) == std::string::npos) 
					continue;     
				filenames.emplace_back(dir_path + entry->d_name);
			}
		}
		// Sort
		std::sort(filenames.begin(), filenames.end());
		std::stable_sort(filenames.begin(), filenames.end(), 
			[] (const std::string& a, const std::string& b) 
			{return a.size() < b.size();});
	} else {
		printf("open directory %s failed", dir_path.c_str());
		return false;
	}

	closedir(dir);
	return true;
}

bool ExportSceneFromKeplerData(
	const std::string& root_dir, 
	Scene& scene, bool only_load_camera = false)
{
	std::vector<std::string> rgb_paths;
	std::vector<std::string> depth_paths;
	std::vector<std::string> camera_paths;
	GetFilesUsingSuffix(root_dir, ".jpg", rgb_paths);
	GetFilesUsingSuffix(root_dir, ".png", depth_paths);
	GetFilesUsingSuffix(root_dir, ".txt", camera_paths);

	ASSERT(rgb_paths.size() == depth_paths.size());
	ASSERT(rgb_paths.size() == camera_paths.size());

	int view_num = static_cast<int>(rgb_paths.size());

	scene.platforms.reserve(view_num);
	scene.images.reserve(view_num);

	std::cerr << scene.pointcloud.points.size() << std::endl;
	
	for (int i = 0; i < view_num; i++)
	{
		std::cerr << "[ExportSceneFromKeplerData] process the frame with view id >> " << view_num << std::endl;

		Image& imageData = scene.images.AddEmpty();
		imageData.name = rgb_paths[i];
		imageData.maskName = "./";
		imageData.platformID = 0; // just single platform
		imageData.cameraID = 0; // just single camera
		imageData.poseID = static_cast<uint32_t>(i); 
		imageData.ID = static_cast<uint32_t>(i); 
		imageData.ReloadImage(0, false);
		imageData.scale = 0.f;
		
		cv::Mat depth;
		if (!only_load_camera)
		{
			depth = cv::imread(depth_paths[i], cv::ImreadModes::IMREAD_ANYDEPTH);
			cv::Scalar mean_depth = cv::mean(depth, depth > 0);
			static_cast<float>(mean_depth.val[0]) / 1000.f;
			imageData.avgDepth = static_cast<float>(mean_depth.val[0]) / 1000.f;
		}

		// ViewScore& neighbor = imageData.neighbors.AddEmpty();
		std::cerr << imageData.name << std::endl;
		std::cerr << "image width: " << imageData.width << std::endl;
		std::cerr << "image height: " << imageData.height << std::endl;

		std::ifstream fin(camera_paths[i]);
		if (!fin.is_open())
		{
			printf("can not open file %s!\n", camera_paths[i].c_str());
			std::abort();
		}
		else
		{
			std::string line_in;
			std::getline(fin, line_in);
			if (line_in != "intrinsic") 
			{
				printf("current line should be intrinsic\n");
				std::abort();
			}

			Eigen::Matrix3f K;

			std::getline(fin, line_in);
            sscanf(line_in.c_str(), "%f %f %f", &K(0, 0), &K(0, 1), &K(0, 2));
			
			std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f", &K(1, 0), &K(1, 1), &K(1, 2));
			
            std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f", &K(2, 0), &K(2, 1), &K(2, 2));

			imageData.camera.K = K.cast<double>();
			std::cerr << imageData.camera.K << std::endl;

			Eigen::Matrix3f R;
			Eigen::Vector3f t;

			std::getline(fin, line_in);
			if (line_in != "extrinsic") 
			{
				printf("current line should be extrinsic\n");
				exit(EXIT_FAILURE);
			}

			std::getline(fin, line_in);
            sscanf(line_in.c_str(), "%f %f %f %f", &R(0, 0), &R(0, 1), &R(0, 2), &t(0, 0));
			
			std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f %f", &R(1, 0), &R(1, 1), &R(1, 2), &t(1, 0));
			
            std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f %f", &R(2, 0), &R(2, 1), &R(2, 2), &t(2, 0));

			imageData.camera.R = R.cast<double>();

			Eigen::Vector3f C = -R.inverse() * t;
			imageData.camera.C = C.cast<double>();

			imageData.camera.ComposeP();

			std::cerr << imageData.camera.R << std::endl;
			std::cerr << imageData.camera.C << std::endl;
			std::cerr << imageData.camera.P << std::endl;

			if (!only_load_camera)
			{
				std::cerr << "[ExportSceneFromKeplerData] old point cloud size >> " << scene.pointcloud.points.size() << std::endl;
				AppendPointCloudFromDepthImage(
					depth, 
					K, R, C, 
					1000.f, 0.4f, 3.5f, 2, i, 
					scene.pointcloud); 

				std::cerr << "[ExportSceneFromKeplerData] new point cloud size >> " << scene.pointcloud.points.size() << std::endl;
			}
		}

		fin.close();
		// break;
	}

	const Image& first_image = scene.images.First();
	Platform& platform = scene.platforms.AddEmpty();
	// platform.name;
	
	CameraIntern& camera = platform.cameras.AddEmpty();
	camera.K = Camera::ScaleK(first_image.camera.K, 
		1.0/(double)Camera::GetNormalizationScale(first_image.width, first_image.height));

	camera.R(0, 0) = 1.0, camera.R(0, 1) = 0.0, camera.R(0, 2) = 0.0, 
	camera.R(1, 0) = 0.0, camera.R(1, 1) = 1.0, camera.R(1, 2) = 0.0,
	camera.R(2, 0) = 0.0, camera.R(2, 1) = 0.0, camera.R(2, 2) = 1.0;
	camera.C[0] = 0.0, camera.C[1] = 0.0, camera.C[2] = 0.0;

	std::cerr << camera.R << std::endl;
	std::cerr << camera.C << std::endl;

	platform.poses.reserve(view_num);
	for (int i = 0; i < view_num; i++)
	{
		const Image& imageData = scene.images[i];
		Platform::Pose& poseData = platform.poses.AddEmpty();

		poseData.R = imageData.camera.R;
		poseData.C = imageData.camera.C;

		std::cerr << poseData.R << "\n" << imageData.camera.R << std::endl;
		std::cerr << poseData.C << "\n" << imageData.camera.C << std::endl;
		// break;
	}
	//exit(-1);
}

template <typename T>
void ReadBinaryDepth(
	const std::string& path, 
	size_t& width_,
	size_t& height_,
	size_t& depth_,
	std::vector<char>& data_) 
{
	std::fstream text_file(path, std::ios::in | std::ios::binary);
	//CHECK(text_file.is_open()) << path;

	char unused_char;
	text_file >> width_ >> unused_char >> height_ >> unused_char >> depth_ >>
		unused_char;

	std::streampos pos = text_file.tellg();
	text_file.close();

	ASSERT(width_ > 0);
	ASSERT(height_ > 0);
	ASSERT(depth_ > 0);
	data_.resize(sizeof(T) * width_ * height_ * depth_);

	std::cerr << width_ << " " << unused_char << " " << height_ << " " << unused_char << depth_ << std::endl;

	std::fstream binary_file(path, std::ios::in | std::ios::binary);
	binary_file.seekg(pos);
	binary_file.read(data_.data(), sizeof(T) * width_ * height_ * depth_);
  	binary_file.close();
}


bool ExportSceneFromBytedanceData(
	const std::string& rgb_dir, 
	const std::string& depth_dir, 
	const std::string& camera_dir, 
	Scene& scene)
{
	std::vector<std::string> rgb_paths;
	std::vector<std::string> depth_paths;
	std::vector<std::string> camera_paths;
	GetFilesUsingSuffix(rgb_dir, ".jpg", rgb_paths);
	//GetFilesUsingSuffix(depth_dir, ".bin", depth_paths);
	GetFilesUsingSuffix(camera_dir, ".txt", camera_paths);

	//ASSERT(rgb_paths.size() == depth_paths.size());
	ASSERT(rgb_paths.size() == camera_paths.size());

	int view_num = static_cast<int>(rgb_paths.size());

	scene.platforms.reserve(view_num);
	scene.images.reserve(view_num);

	std::cerr << scene.pointcloud.points.size() << std::endl;
	
	for (int i = 0; i < view_num; i++)
	{
		std::cerr << "[ExportSceneFromBytedanceData] view id >> " << view_num << std::endl;
		std::cerr << ">> " << rgb_paths[i] << std::endl;
		//depth_pathsstd::cerr << ">> " << depth_paths[i] << std::endl;
		std::cerr << ">> " << camera_paths[i] << std::endl;

		Image& imageData = scene.images.AddEmpty();
		imageData.name = rgb_paths[i];
		imageData.maskName = "./";
		imageData.platformID = 0; // just single platform
		imageData.cameraID = 0; // just single camera
		imageData.poseID = static_cast<uint32_t>(i); 
		imageData.ID = static_cast<uint32_t>(i); 
		imageData.ReloadImage(0, false);
		imageData.scale = 0.f;

#if READ_COLMAP_DATA
		std::vector<char> depth_data;
		size_t depth_width, depth_height, depth_depth;
		ReadBinaryDepth<float>(depth_paths[i], depth_width, depth_height, depth_depth, depth_data);

		cv::Mat depth(cv::Size(depth_width, depth_height), CV_32FC1, depth_data.data());
		depth *= 1000.0;
		depth.convertTo(depth, CV_16UC1);
		cv::Scalar mean_depth = cv::mean(depth, depth > 0);
		imageData.avgDepth = static_cast<float>(mean_depth.val[0]) / 1000.f;
		std::cerr << imageData.avgDepth << std::endl;
		//cv::imwrite("test.png", depth_map);
#endif

		// ViewScore& neighbor = imageData.neighbors.AddEmpty();
		std::cerr << imageData.name << std::endl;
		std::cerr << "image width: " << imageData.width << std::endl;
		std::cerr << "image height: " << imageData.height << std::endl;

		std::ifstream fin(camera_paths[i]);
		if (!fin.is_open())
		{
			printf("can not open file %s!\n", camera_paths[i].c_str());
			std::abort();
		}
		else
		{
			std::string line_in;
			std::getline(fin, line_in);
			if (line_in != "extrinsic") 
			{
				printf("current line should be extrinsic\n");
				exit(EXIT_FAILURE);
			}

			Eigen::Matrix3f R;
			Eigen::Vector3f t;

			std::getline(fin, line_in);
            sscanf(line_in.c_str(), "%f %f %f %f", &R(0, 0), &R(0, 1), &R(0, 2), &t(0, 0));
			
			std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f %f", &R(1, 0), &R(1, 1), &R(1, 2), &t(1, 0));
			
            std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f %f", &R(2, 0), &R(2, 1), &R(2, 2), &t(2, 0));

			imageData.camera.R = R.cast<double>();

			Eigen::Vector3f C = -R.inverse() * t;
			imageData.camera.C = C.cast<double>();

			imageData.camera.ComposeP();

			std::cerr << imageData.camera.R << std::endl;
			std::cerr << imageData.camera.C << std::endl;
			std::cerr << imageData.camera.P << std::endl;

			std::getline(fin, line_in);
			std::getline(fin, line_in);
			std::getline(fin, line_in);
			if (line_in != "intrinsic") 
			{
				printf("current line should be intrinsic\n");
				std::abort();
			}

			Eigen::Matrix3f K;

			std::getline(fin, line_in);
            sscanf(line_in.c_str(), "%f %f %f", &K(0, 0), &K(0, 1), &K(0, 2));
			
			std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f", &K(1, 0), &K(1, 1), &K(1, 2));
			
            std::getline(fin, line_in);
			sscanf(line_in.c_str(), "%f %f %f", &K(2, 0), &K(2, 1), &K(2, 2));

			imageData.camera.K = K.cast<double>();
			std::cerr << imageData.camera.K << std::endl;

#if READ_COLMAP_DARA
			std::cerr << "[ExportSceneFromBytedanceData] old point cloud size >> " << scene.pointcloud.points.size() << std::endl;
			AppendPointCloudFromDepthImage(
				depth, 
				K, R, C, 
				1000.f, 0.4f, 3.5f, 2, i, 
				scene.pointcloud); 

			std::cerr << "[ExportSceneFromBytedanceData] new point cloud size >> " << scene.pointcloud.points.size() << std::endl;
#endif
		}

		fin.close();

		// break;
	}

	const Image& first_image = scene.images.First();
	Platform& platform = scene.platforms.AddEmpty();
	// platform.name;
	
	CameraIntern& camera = platform.cameras.AddEmpty();
	camera.K = Camera::ScaleK(first_image.camera.K, 
		1.0/(double)Camera::GetNormalizationScale(first_image.width, first_image.height));

	camera.R(0, 0) = 1.0, camera.R(0, 1) = 0.0, camera.R(0, 2) = 0.0, 
	camera.R(1, 0) = 0.0, camera.R(1, 1) = 1.0, camera.R(1, 2) = 0.0,
	camera.R(2, 0) = 0.0, camera.R(2, 1) = 0.0, camera.R(2, 2) = 1.0;
	camera.C[0] = 0.0, camera.C[1] = 0.0, camera.C[2] = 0.0;

	std::cerr << camera.R << std::endl;
	std::cerr << camera.C << std::endl;

	platform.poses.reserve(view_num);
	for (int i = 0; i < view_num; i++)
	{
		const Image& imageData = scene.images[i];
		Platform::Pose& poseData = platform.poses.AddEmpty();

		poseData.R = imageData.camera.R;
		poseData.C = imageData.camera.C;

		std::cerr << poseData.R << "\n" << imageData.camera.R << std::endl;
		std::cerr << poseData.C << "\n" << imageData.camera.C << std::endl;
		// break;
	}
}


// initialize and parse the command line parameters
bool Initialize(size_t argc, LPCTSTR* argv)
{
	// initialize log and console
	OPEN_LOG();
	OPEN_LOGCONSOLE();

	// group of options allowed only on command line
	boost::program_options::options_description generic("Generic options");
	generic.add_options()
		("help,h", "produce this help message")
		("working-folder,w", boost::program_options::value<std::string>(&WORKING_FOLDER), "working directory (default current directory)")
		("config-file,c", boost::program_options::value<std::string>(&OPT::strConfigFileName)->default_value(APPNAME _T(".cfg")), "file name containing program options")
		("export-type", boost::program_options::value<std::string>(&OPT::strExportType)->default_value(_T("ply")), "file type used to export the 3D scene (ply or obj)")
		("archive-type", boost::program_options::value(&OPT::nArchiveType)->default_value(ARCHIVE_DEFAULT), "project archive type: 0-text, 1-binary, 2-compressed binary")
		("process-priority", boost::program_options::value(&OPT::nProcessPriority)->default_value(-1), "process priority (below normal by default)")
		("max-threads", boost::program_options::value(&OPT::nMaxThreads)->default_value(0), "maximum number of threads (0 for using all available cores)")
		#if TD_VERBOSE != TD_VERBOSE_OFF
		("verbosity,v", boost::program_options::value(&g_nVerbosityLevel)->default_value(
			#if TD_VERBOSE == TD_VERBOSE_DEBUG
			3
			#else
			2
			#endif
			), "verbosity level")
		#endif
		;

	// group of options allowed both on command line and in config file
	boost::program_options::options_description config_main("Reconstruct options");
	config_main.add_options()
		("input-file,i", boost::program_options::value<std::string>(&OPT::strInputFileName), "input filename containing camera poses and image list")
		("output-file,o", boost::program_options::value<std::string>(&OPT::strOutputFileName), "output filename for storing the mesh")
		("min-point-distance,d", boost::program_options::value(&OPT::fDistInsert)->default_value(2.5f), "minimum distance in pixels between the projection of two 3D points to consider them different while triangulating (0 - disabled)")
		("constant-weight", boost::program_options::value(&OPT::bUseConstantWeight)->default_value(true), "considers all view weights 1 instead of the available weight")
		("free-space-support,f", boost::program_options::value(&OPT::bUseFreeSpaceSupport)->default_value(false), "exploits the free-space support in order to reconstruct weakly-represented surfaces")
		("thickness-factor", boost::program_options::value(&OPT::fThicknessFactor)->default_value(1.f), "multiplier adjusting the minimum thickness considered during visibility weighting")
		("quality-factor", boost::program_options::value(&OPT::fQualityFactor)->default_value(1.f), "multiplier adjusting the quality weight considered during graph-cut")
		;
	boost::program_options::options_description config_clean("Clean options");
	config_clean.add_options()
		("decimate", boost::program_options::value(&OPT::fDecimateMesh)->default_value(1.f), "decimation factor in range (0..1] to be applied to the reconstructed surface (1 - disabled)")
		("target-face-num", boost::program_options::value(&OPT::nTargetFaceNum)->default_value(0), "target number of faces to be applied to the reconstructed surface. (0 - disabled)")
		("remove-spurious", boost::program_options::value(&OPT::fRemoveSpurious)->default_value(20.f), "spurious factor for removing faces with too long edges or isolated components (0 - disabled)")
		("remove-spikes", boost::program_options::value(&OPT::bRemoveSpikes)->default_value(true), "flag controlling the removal of spike faces")
		("close-holes", boost::program_options::value(&OPT::nCloseHoles)->default_value(30), "try to close small holes in the reconstructed surface (0 - disabled)")
		("smooth", boost::program_options::value(&OPT::nSmoothMesh)->default_value(2), "number of iterations to smooth the reconstructed surface (0 - disabled)")
		;

	// hidden options, allowed both on command line and
	// in config file, but will not be shown to the user
	boost::program_options::options_description hidden("Hidden options");
	hidden.add_options()
		("mesh-file", boost::program_options::value<std::string>(&OPT::strMeshFileName), "mesh file name to clean (skips the reconstruction step)")
		("mesh-export", boost::program_options::value(&OPT::bMeshExport)->default_value(false), "just export the mesh contained in loaded project")
		("split-max-area", boost::program_options::value(&OPT::fSplitMaxArea)->default_value(0.f), "maximum surface area that a sub-mesh can contain (0 - disabled)")
		("image-points-file", boost::program_options::value<std::string>(&OPT::strImagePointsFileName), "input filename containing the list of points from an image to project on the mesh (optional)")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic).add(config_main).add(config_clean).add(hidden);

	boost::program_options::options_description config_file_options;
	config_file_options.add(config_main).add(config_clean).add(hidden);

	boost::program_options::positional_options_description p;
	p.add("input-file", -1);

	try {
		// parse command line options
		boost::program_options::store(boost::program_options::command_line_parser((int)argc, argv).options(cmdline_options).positional(p).run(), OPT::vm);
		boost::program_options::notify(OPT::vm);
		INIT_WORKING_FOLDER;
		// parse configuration file
		std::ifstream ifs(MAKE_PATH_SAFE(OPT::strConfigFileName));
		if (ifs) {
			boost::program_options::store(parse_config_file(ifs, config_file_options), OPT::vm);
			boost::program_options::notify(OPT::vm);
		}
	}
	catch (const std::exception& e) {
		LOG(e.what());
		return false;
	}

	// initialize the log file
	OPEN_LOGFILE(MAKE_PATH(APPNAME _T("-")+Util::getUniqueName(0)+_T(".log")).c_str());

	// print application details: version and command line
	Util::LogBuild();
	LOG(_T("Command line: ") APPNAME _T("%s"), Util::CommandLineToString(argc, argv).c_str());

	// validate input
	Util::ensureValidPath(OPT::strInputFileName);
	Util::ensureUnifySlash(OPT::strInputFileName);
	if (OPT::vm.count("help") || OPT::strInputFileName.IsEmpty()) {
		boost::program_options::options_description visible("Available options");
		visible.add(generic).add(config_main).add(config_clean);
		GET_LOG() << visible;
	}
	if (OPT::strInputFileName.IsEmpty())
		return false;
	OPT::strExportType = OPT::strExportType.ToLower() == _T("obj") ? _T(".obj") : _T(".ply");

	// initialize optional options
	Util::ensureValidPath(OPT::strOutputFileName);
	Util::ensureUnifySlash(OPT::strOutputFileName);
	Util::ensureValidPath(OPT::strImagePointsFileName);
	if (OPT::strOutputFileName.IsEmpty())
		OPT::strOutputFileName = Util::getFileFullName(OPT::strInputFileName) + _T("_mesh.mvs");

	// initialize global options
	Process::setCurrentProcessPriority((Process::Priority)OPT::nProcessPriority);
	#ifdef _USE_OPENMP
	if (OPT::nMaxThreads != 0)
		omp_set_num_threads(OPT::nMaxThreads);
	#endif

	#ifdef _USE_BREAKPAD
	// start memory dumper
	MiniDumper::Create(APPNAME, WORKING_FOLDER);
	#endif

	Util::Init();
	return true;
}

// finalize application instance
void Finalize()
{
	#if TD_VERBOSE != TD_VERBOSE_OFF
	// print memory statistics
	Util::LogMemoryInfo();
	#endif

	CLOSE_LOGFILE();
	CLOSE_LOGCONSOLE();
	CLOSE_LOG();
}

} // unnamed namespace


int main(int argc, LPCTSTR* argv)
{
	#ifdef _DEBUGINFO
	// set _crtBreakAlloc index to stop in <dbgheap.c> at allocation
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);// | _CRTDBG_CHECK_ALWAYS_DF);
	#endif

	if (!Initialize(argc, argv))
		return EXIT_FAILURE;

	Scene scene(OPT::nMaxThreads);
	
	// // load project
	// if (!scene.Load(MAKE_PATH_SAFE(OPT::strInputFileName), OPT::fSplitMaxArea > 0 || OPT::fDecimateMesh < 1 || OPT::nTargetFaceNum > 0))
	// 	return EXIT_FAILURE;

// 	std::string rgb_dir = "/Users/admin/Documents/Projects/Internal/LargeSceneReconstruction/mvs/python/output_0328/acmm/images/";
// 	std::string depth_dir = "/Users/admin/Downloads/depth_maps/";
// 	std::string camera_dir = "/Users/admin/Documents/Projects/Internal/LargeSceneReconstruction/mvs/python/output_0328/acmm/cams/";
// 	ExportSceneFromBytedanceData(rgb_dir, depth_dir, camera_dir, scene);

// #if READ_ACMM_DATA
// 	ReadPlyAsPointCloud("/Users/admin/Downloads/ACMMP_model.ply", scene.pointcloud);
// 	ReadPointCloudDetails("/Users/admin/Downloads/index.json", scene);
// #endif

	ExportSceneFromKeplerData(
		"/Users/admin/Documents/Projects/Internal/ReconLibrary/build/7081932335616147493_openmvs/", scene, true);

	ReadPlyAsPointCloud("/Users/admin/Documents/Projects/Internal/ReconLibrary/build/7081932335616147493_temp/fusion_api_1.ply", scene.pointcloud);
	ReadPointCloudDetails("/Users/admin/Documents/Projects/Internal/ReconLibrary/build/7081932335616147493_temp/point_view_1.json", scene);
	
	// return EXIT_SUCCESS;

	const String baseFileName(MAKE_PATH_SAFE(Util::getFileFullName(OPT::strOutputFileName)));

	if (OPT::strMeshFileName.IsEmpty() && scene.mesh.IsEmpty()) {
		// reset image resolution to the original size and
		// make sure the image neighbors are initialized before deleting the point-cloud
		#ifdef RECMESH_USE_OPENMP
		bool bAbort(false);
		#pragma omp parallel for
		for (int_t idx=0; idx<(int_t)scene.images.GetSize(); ++idx) {
			#pragma omp flush (bAbort)
			if (bAbort)
				continue;
			const uint32_t idxImage((uint32_t)idx);
		#else
		FOREACH(idxImage, scene.images) {
		#endif
			Image& imageData = scene.images[idxImage];
			if (!imageData.IsValid())
				continue;
			// reset image resolution
			if (!imageData.ReloadImage(0, false)) {
				#ifdef RECMESH_USE_OPENMP
				bAbort = true;
				#pragma omp flush (bAbort)
				continue;
				#else
				return EXIT_FAILURE;
				#endif
			}
			imageData.UpdateCamera(scene.platforms);
			// select neighbor views
			if (imageData.neighbors.IsEmpty()) {
				IndexArr points;
				scene.SelectNeighborViews(idxImage, points, 1, 1);
			}
		}

		#ifdef RECMESH_USE_OPENMP
		if (bAbort)
			return EXIT_FAILURE;
		#endif
		// reconstruct a coarse mesh from the given point-cloud
		TD_TIMER_START();
		if (OPT::bUseConstantWeight)
			scene.pointcloud.pointWeights.Release();
		if (!scene.ReconstructMesh(OPT::fDistInsert, OPT::bUseFreeSpaceSupport, 4, OPT::fThicknessFactor, OPT::fQualityFactor))
			return EXIT_FAILURE;
		VERBOSE("Mesh reconstruction completed: %u vertices, %u faces (%s)", scene.mesh.vertices.GetSize(), scene.mesh.faces.GetSize(), TD_TIMER_GET_FMT().c_str());
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2) {
			// dump raw mesh
			scene.mesh.Save(baseFileName+_T("_raw")+OPT::strExportType);
		}
		#endif
	
		// clean the mesh
		const float fDecimate(OPT::nTargetFaceNum ? static_cast<float>(OPT::nTargetFaceNum) / scene.mesh.faces.size() : OPT::fDecimateMesh);
		scene.mesh.Clean(fDecimate, OPT::fRemoveSpurious, OPT::bRemoveSpikes, OPT::nCloseHoles, OPT::nSmoothMesh, false);
		scene.mesh.Clean(1.f, 0.f, OPT::bRemoveSpikes, OPT::nCloseHoles, 0, false); // extra cleaning trying to close more holes
		scene.mesh.Clean(1.f, 0.f, false, 0, 0, true); // extra cleaning to remove non-manifold problems created by closing holes

		// save the final mesh
		scene.Save(baseFileName+_T(".mvs"), (ARCHIVE_TYPE)OPT::nArchiveType);
		scene.mesh.Save(baseFileName+OPT::strExportType);
		#if TD_VERBOSE != TD_VERBOSE_OFF
		if (VERBOSITY_LEVEL > 2)
			scene.ExportCamerasMLP(baseFileName+_T(".mlp"), baseFileName+OPT::strExportType);
		#endif
	}

	Finalize();
	return EXIT_SUCCESS;
}
/*----------------------------------------------------------------*/
