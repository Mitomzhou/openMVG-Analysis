# openMVG源码解析

## 源码结构
### 转载自 [知乎@迷途小书童](https://zhuanlan.zhihu.com/p/97210820?from_voters_page=true)
#### openMVG尽力提供可读性强的代码，方便二次开发，核心库尽可能精简，全局分成了几个大的模块：
* 核心库：各个功能的核心算法实现
* 样例：教你怎么用核心库实现高级算法
* 工具链：也就是连起来用咯（无序影像特征匹配，完整SfM，影像色彩处理）
#### src部分模块 
  * /src/dependencies 依赖库，包括gLfw, cereal, osi clp；
  * /src/nonFree SIFT特征提取
  * /src/openMVG 是多视图几何（MVG）和SfM的核心库
  * /src/openMVG_Samples 展示如何使用核心库实现高级算法，主要涉及相对定向、特征匹配的鲁棒策略等；
  * /src/software 提供用户使用的软件接口，主要实现了SfM及可视化，视觉里程计（Visual Odometry）,地理坐标系转换，UI便于输入控制点数据；
  * /src/third_party 是底层第三方库，主要有ceres-solver，eigen，fast，stlplus3，jpeg等；
#### /src/openMVG 核心库各个模块名称、功能、包含的文件数如下表所示：
|  模块   | 功能  |
|  ----  | ----  |
| cameras  | 定义针孔相机的几何模型，和多种畸变模型（多项式,Brown,鱼眼） |
| clustering  | kmeans聚类 |
| color_harmonization  | 单元格 |
| exif  | 单元格 |
| features  | 单元格 |
| geodesy  | 单元格 |
| geometry  | 单元格 |
| praph  | 单元格 |
| graphics  | 单元格 |
| image  | image数据存储以及IO，支持ppm,pgm,jpeg,png,tiff |
| linearProgramming  | 单元格 |
| matching  | 单元格 |
| matching_image_collection  | 单元格 |
| multiview  | 单元格 |
| numeric  | 单元格 |
| robust_estimation  | 单元格 |
| sfm | 单元格 |
| spherical | 单元格 |
| stl | 单元格 |
| system | 单元格 |
| tracks | 单元格 |

#### /src/third_party 外部第三方库
|  模块   | 功能  |
|  ----  | ----  |
|  ceres-solver   | 功能  |
|  cmdLine   | 功能  |
|  CppUnitLite   | 功能  |
|  cxsparse   | 功能  |
|  easyexif   | 功能  |
|  eigen   | 功能  |
|  fast   | 功能  |
|  flann   | 功能  |
|  histogram   | 功能  |
|  hnswlib   | 功能  |
|  htmlDoc   | 功能  |
|  jpeg   | 功能  |
|  lemon   | 功能  |
|  png   | 功能  |
|  progress   | 功能  |
|  stlplus3   | 功能  |
|  tiff   | 功能  |
|  vectorGraphics   | 功能  |
|  zlib   | 功能  |

## SfM Pipeline
目前基本的 SfM 策略主要有2种，分别为增量式incremental和全局式global，如下图所示。增量式方法往往选择最优的种子像对建立初始场景（initialization），然后不添加新影像以扩展场景（image registration, triangulation, bundle adjustment），最后对整个场景光束法平差优化一次，调整所有相机的内、外参数和物方点（3D points）。而全局式方法是将输入所有影像，先同时解算相机的全局旋转矩阵（此过程称为rotation averaging），接着是全局平移矩阵（此过程称为translation averaging），再经前方交会（triangulation）初始化整个场景的物方点，最后对整个场景光束法平差，优化所有相机的内、外参数和物方点。一般而言，增量式方法多次光束法平差中会进行粗差探测与剔除（outlier filtering），整体较为鲁棒，缺点是时间效率低下且误差积累会产生场景偏移。而全局式方法整体同时解算，时间效率高，但严重依赖相对定向的质量，鲁棒性较差。
![RUNOOB 图标](https://pic4.zhimg.com/80/v2-fd2dc3c779b6b53eeae9296c0d1e6beb_720w.jpg)
![RUNOOB 图标](https://pic3.zhimg.com/80/v2-e4f2d3fa55f88610dd9dc963cad9ca02_720w.jpg)




## tutorial_demo.py

### step1: main_SfMInit_ImageListing.cpp 
[step1官方文档](https://openmvg.readthedocs.io/en/latest/software/SfM/SfMInit_ImageListing/)

输入图像和相机数据库，输出sfm_data.json包含图像尺寸，焦距（单位像素）
~~~
$ openMVG_main_SfMInit_ImageListing -i ./ImageDataset_SceauxCastle/img -o ./ImageDataset_SceauxCastle/sfm_out -d /home/mitom/3DReconstruction/MVG_MVS/openMVG/src/openMVG/exif/sensor_width_database/sensor_width_camera_database.txt -c 3
~~~
~~~
/home/mitom/3DReconstruction/MVG_MVS/openMVG/src/cmake-build-debug/Linux-x86_64-Debug/openMVG_main_SfMInit_ImageListing -i /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/images -o /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out -d /home/mitom/3DReconstruction/MVG_MVS/openMVG/src/openMVG/exif/sensor_width_database/sensor_width_camera_database.txt -c 3
 You called : 
/home/mitom/3DReconstruction/MVG_MVS/openMVG/src/cmake-build-debug/Linux-x86_64-Debug/openMVG_main_SfMInit_ImageListing
--imageDirectory /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/images
--sensorWidthDatabase /home/mitom/3DReconstruction/MVG_MVS/openMVG/src/openMVG/exif/sensor_width_database/sensor_width_camera_database.txt
--outputDirectory /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out
--focal -1
--intrinsics 
--camera_model 3
--group_camera_model 1

- Image listing -
0%   10   20   30   40   50   60   70   80   90   100%
|----|----|----|----|----|----|----|----|----|----|
***************************************************

SfMInit_ImageListing report:
listed #File(s): 11
usable #File(s) listed in sfm_data: 11
usable #Intrinsic(s) listed in sfm_data: 1
~~~
~~~cpp
...
// The camera model was found in the database so we can compute it's approximated focal length
const double ccdw = datasheet.sensorSize_;
focal = std::max(width, height) * exifReader->getFocal() / ccdw;
...
~~~
#### 相机焦距计算（单位像素）
**focal = max(w, h) * F / S**
 * F：相机焦距长度(focal length，单位毫米mm)
 * w,h: 图片的宽高
 * S: ccd传感器孔径（sensor size，单位mm）
#### 相机模型
~~~cpp
enum EINTRINSIC
{
  PINHOLE_CAMERA_START = 0,
  PINHOLE_CAMERA,         //无畸变
  PINHOLE_CAMERA_RADIAL1, // 径向畸变K1
  PINHOLE_CAMERA_RADIAL3, // 径向畸变K1,K2,K3
  PINHOLE_CAMERA_BROWN, //径向畸变K1，K2，K3，切向畸变T1，T2
  PINHOLE_CAMERA_FISHEYE, //具有4个畸变系数的简单鱼眼畸变模型
  PINHOLE_CAMERA_END,
  CAMERA_SPHERICAL = PINHOLE_CAMERA_END + 1
};
~~~
~~~cpp
// 初始化内参
width = height = ppx = ppy = focal = -1.0;
// ...
// 考虑手动提供焦点
if (sKmatrix.size() > 0) // 已知用户校准K矩阵
{
  if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
    focal = -1.0;
}
else //用户提供的焦距值
  if (focal_pixels != -1 )
    focal = focal_pixels;

// 如果不是手动提供或错误提供
if (focal == -1)
{
  std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
  //打开文件进行检查和分析返回bool
  exifReader->open( sImageFilename );
  //验证文件是否有元数据并且获取相机的型号不为空
  const bool bHaveValidExifMetadata =
    exifReader->doesHaveExifInfo()
    && !exifReader->getModel().empty();
  //错误则报错，对则引用
  if (bHaveValidExifMetadata) // If image contains meta data
  {
    const std::string sCamModel = exifReader->getModel();
    // 处理焦距等于0的情况
    if (exifReader->getFocal() == 0.0f)
    {
      error_report_stream
        << stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
      focal = -1.0;
    }
    else
    // 在列表文件中创建图像条目
    {
      Datasheet datasheet;
      if ( getInfo( sCamModel, vec_database, datasheet ))
      {
        // 在数据库中找到了相机模型，所以我们可以计算出它的近似焦距
        const double ccdw = datasheet.sensorSize_;
        focal = std::max ( width, height ) * exifReader->getFocal() / ccdw;
      }
      else
      {
        error_report_stream
          << stlplus::basename_part(sImageFilename)
          << "\" model \"" << sCamModel << "\" doesn't exist in the dataelse		base" << "\n"
          << "Please consider add your camera model and sensor width in the database." << "\n";
      }
    }
  }
}

//...

// 构建与图像对应的视图，若有gps权重时（就需要定义为优先旋转）
const std::pair<bool, Vec3> gps_info = checkGPS(sImageFilename, i_GPS_XYZ_method);
if (gps_info.first)
{
//Views的子类，可以选择是否优先旋转，或者优先调整位置
  ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

  // 添加与图像相关的内部文件（如果有）
  if (intrinsic == nullptr)
  {
//因为视图具有无效的内部数据
//（使用无效的内在字段值导出视图）
    v.id_intrinsic = UndefinedIndexT;
  }
  else
  {
    // Add the defined intrinsic to the sfm_container
    intrinsics[v.id_intrinsic] = intrinsic;
  }

  v.b_use_pose_center_ = true;
  v.pose_center_ = gps_info.second;

  //先前的权重
  if (prior_w_info.first == true)
  {
    v.center_weight_ = prior_w_info.second;
  }
  //将视图添加到sfm容器
  views[v.id_view] = std::make_shared<ViewPriors>(v);
}
// 没有gps信息时
else		
{
  View v(*iter_image, views.size(), views.size(), views.size(), width, height);

  // Add intrinsic related to the image (if any)
  if (intrinsic == nullptr)
  {
    //Since the view have invalid intrinsic data
    // (export the view, with an invalid intrinsic field value)
    v.id_intrinsic = UndefinedIndexT;
  }
  else
  {
    // Add the defined intrinsic to the sfm_container
    intrinsics[v.id_intrinsic] = intrinsic;
  }

  // Add the view to the sfm_container
  views[v.id_view] = std::make_shared<View>(v);
}

// ...

// 保存则将sfm_data的数据命名并保存到output文件当中
Save(  sfm_data, stlplus::create_filespec( sOutputDir, "sfm_data.json" ).c_str(),  ESfM_Data(VIEWS|INTRINSICS))

/* 11张Demo图片不做gps约束因此不需要增加提前的pose */
~~~



### sfm_data.json
~~~json
{
  ...,
"views":{
        "key": 10, // 视角（图像）编号
        "value": {
            "polymorphic_id": 1073741824,
            "ptr_wrapper": {
                "id": 2147483659,
                "data": {
                    "local_path": "",
                    "filename": "100_7110.JPG",
                    "width": 2832,
                    "height": 2128,
                    "id_view": 10,
                    "id_intrinsic": 0,
                    "id_pose": 10
                }
            }
        }
    },
 "intrinsics": [
        {
            "key": 0,
            "value": {
                "polymorphic_id": 2147483649,
                "polymorphic_name": "pinhole_radial_k3", // 内参k1,k2,k2*/
                "ptr_wrapper": {
                    "id": 2147483660,
                    "data": {
                        "width": 2832,
                        "height": 2128,
                        "focal_length": 2884.2617914205145, // 焦长（单位像素）
                        "principal_point": [
                            1416.0,
                            1064.0
                        ],
                        "disto_k3": [
                            0.0,
                            0.0,
                            0.0
                        ]
                    }
                }
            }
        }
    ], ...
 }
~~~
### step2: main_ComputeFeatures.cpp
[step2官方文档](https://openmvg.readthedocs.io/en/latest/software/SfM/ComputeFeatures/)

特征提取，对给定的图像描述sfm_data.json,对于每个视图，计算特征描述，并存储.feat和.desc文件

~~~
$ openMVG_main_ComputeFeatures -i ./ImageDataset_SceauxCastle/sfm_out/sfm_data.json -o ./ImageDataset_SceauxCastle/sfm_out/matches -m SIFT -f 1 -n 8
~~~
~~~
/home/mitom/3DReconstruction/MVG_MVS/openMVG/src/cmake-build-debug/Linux-x86_64-Debug/openMVG_main_ComputeFeatures -i /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/sfm_data.json -o /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/matches -m SIFT -f 1 -n 10
 You called : 
/home/mitom/3DReconstruction/MVG_MVS/openMVG/src/cmake-build-debug/Linux-x86_64-Debug/openMVG_main_ComputeFeatures
--input_file /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/sfm_data.json
--outdir /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/matches
--describerMethod SIFT
--upright 0
--describerPreset NORMAL
--force 1
--numThreads 10


- EXTRACT FEATURES -
0%   10   20   30   40   50   60   70   80   90   100%
|----|----|----|----|----|----|----|----|----|----|
***************************************************
Task done in (s): 22
~~~
[知乎@灰灰-图像配准](https://zhuanlan.zhihu.com/p/75784915?from=singlemessage)
#### SfM数据结构
~~~cpp
struct SfM_Data
{
Views views;   /// 储存影像物理性质、索引号等基本信息
Poses poses;   /// 储存影像外参数，包括旋转矩阵、平移矩阵（相机姿态）
Intrinsics intrinsics; /// 储存影像内参数，支持多组不同内参数(相机内参)
Landmarks structure;   /// 物方点信息，物方点坐标及其tracks(二维视图特征关联的3D点)
                        /*
                        定义由TrackId索引的地标集合，Landmark包含两个成员，
                        3d点及其所对应于图像上的坐标的hash表，
                        因为一个世界中的坐标可以被多张相机所观测到。
                        Landmarks点位又分为三角测量获得的点（用于BA）和地面控制点（用于GCP）
                        */ 
}
~~~
数据与参数
~~~cpp
std::string sSfM_Data_Filename;	 //main_SfMInit_ImageListing生成的文件的路径
std::string sOutDir = "";		//输出文件夹
bool bUpRight = false;			//AKAZE描述子使用，是否计算方向
std::string sImage_Describer_Method = "SIFT_ANATOMY";		//使用的描述子
bool bForce = false;		//
std::string sFeaturePreset = "";		//描述子质量NORMAL，HIGH，ULTRA
~~~
预设置 
~~~cpp
// 设置好描述子类型与描述子质量后，导出用于动态未来区域计算和/或加载的所用图像描述符和区域类型
std::ofstream stream(sImage_describer.c_str());
if (!stream.is_open())
   return EXIT_FAILURE;

cereal::JSONOutputArchive archive(stream);
archive(cereal::make_nvp("image_describer", image_describer));
auto regionsType = image_describer->Allocate();
archive(cereal::make_nvp("regions_type", regionsType));
~~~
计算特征
~~~cpp
// 初始化每个视角
Views::const_iterator iterViews = sfm_data.views.begin();
std::advance(iterViews, i);
const View * view = iterViews->second.get();

// ...

// 创建两个文件，XXX.feat，XXX.desc
const std::string
        sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
        sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),
        sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

// If features or descriptors file are missing, compute them
if (!preemptive_exit && (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc)))
{
    if (!ReadImage(sView_filename.c_str(), &imageGray))
        continue;
    // 查看是否有遮挡特征mask，见官方文档，解释非常详细
    Image<unsigned char> * mask = nullptr; // The mask is null by default

    const std::string
      mask_filename_local =
        stlplus::create_filespec(sfm_data.s_root_path,
          stlplus::basename_part(sView_filename) + "_mask", "png"),
      mask__filename_global =
        stlplus::create_filespec(sfm_data.s_root_path, "mask", "png");

    Image<unsigned char> imageMask;
    // Try to read the local mask
    if (stlplus::file_exists(mask_filename_local))
    {
      if (!ReadImage(mask_filename_local.c_str(), &imageMask))
      {
        std::cerr << "Invalid mask: " << mask_filename_local << std::endl
                  << "Stopping feature extraction." << std::endl;
        preemptive_exit = true;
        continue;
      }
      // Use the local mask only if it fits the current image size
      // 仅当本地mask符合当前图像大小时使用
      if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
        mask = &imageMask;
    }
    else
    {
        // 全局mask
      // Try to read the global mask
      if (stlplus::file_exists(mask__filename_global))
      {
        if (!ReadImage(mask__filename_global.c_str(), &imageMask))
        {
          std::cerr << "Invalid mask: " << mask__filename_global << std::endl
                    << "Stopping feature extraction." << std::endl;
          preemptive_exit = true;
          continue;
        }
        // Use the global mask only if it fits the current image size
        if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
          mask = &imageMask;
      }
    }
    
    // 计算特征和描述符并将它们导出到文件中（核心）
    // Compute features and descriptors and export them to files
    auto regions = image_describer->Describe(imageGray, mask);
    if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
      std::cerr << "Cannot save regions for images: " << sView_filename << std::endl
                << "Stopping feature extraction." << std::endl;
      preemptive_exit = true;
      continue;
    }
    // ...
}    
  
  // 特征计算可以使用多线程，计算速度会更快，建议使用机器线程1/2，不然内存out

~~~
### step3:main_ComputeMatches.cpp
#### 特征匹配步骤：
* 加载视图图像描述（区域：特征和描述符）
* 计算假定的局部特征匹配（描述符匹配）
* 计算几何相干特征匹配（基于假定匹配的稳健模型估计）
* 导出计算数据

~~~
$ openMVG_main_ComputeMatches  -i ./ImageDataset_SceauxCastle/sfm_out/sfm_data.json -o ./ImageDataset_SceauxCastle/sfm_out/matches -f 1 -n ANNL2
~~~
~~~
/home/mitom/Music/openMVG/src/cmake-build-debug/Linux-x86_64-Debug/openMVG_main_ComputeMatches /home/mitom/3DReconstruction/MVG_MVS/openMVG/src/cmake-build-debug/Linux-x86_64-Debug/openMVG_main_ComputeFeatures -i /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/sfm_data.json -o /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/matches -f 1 -n ANNL2
 You called : 
/home/mitom/Music/openMVG/src/cmake-build-debug/Linux-x86_64-Debug/openMVG_main_ComputeMatches
--input_file /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/sfm_data.json
--out_dir /home/mitom/3DReconstruction/MVG_MVS/data/ImageDataset_SceauxCastle/sfm_out/matches
Optional parameters:
--force 1
--ratio 0.8
--geometric_model f
--video_mode_matching -1
--pair_list 
--nearest_matching_method ANNL2
--guided_matching 0
--cache_size unlimited

- Regions Loading -
0%   10   20   30   40   50   60   70   80   90   100%
|----|----|----|----|----|----|----|----|----|----|
***************************************************

 - PUTATIVE MATCHES - 
Use: exhaustive pairwise matching
Using ANN_L2 matcher
Using the OPENMP thread interface

- Matching -
0%   10   20   30   40   50   60   70   80   90   100%
|----|----|----|----|----|----|----|----|----|----|
***************************************************
Task (Regions Matching) done in (s): 48

- Geometric filtering -
0%   10   20   30   40   50   60   70   80   90   100%
|----|----|----|----|----|----|----|----|----|----|
***************************************************
Task done in (s): 71
Graph statistics:
	#nodes: 11
	#cc: 1
	#singleton: 0
	Node degree statistics:	min: 10, max: 10, mean: 10, median: 10

 Export Adjacency Matrix of the pairwise's geometric matches
~~~
输入与输出说明
~~~cpp
// 文件名
std::string sSfM_Data_Filename;
// 输出文件夹 
std::string sMatchesDirectory = "";
// 几何模型	
std::string sGeometricModel = "f";
//enum EGeometricModel
//{
//    FUNDAMENTAL_MATRIX = 0,基本矩阵
//    ESSENTIAL_MATRIX   = 1,本质矩阵
//    HOMOGRAPHY_MATRIX  = 2,单应矩阵
//    ESSENTIAL_MATRIX_ANGULAR = 3,基本矩阵角
//    ESSENTIAL_MATRIX_ORTHO = 4,基本正交矩阵
//    ESSENTIAL_MATRIX_UPRIGHT = 5基本矩阵直立
//};

// f距离比
float fDistRatio = 0.8f;
// 配对模型
int iMatchingVideoMode = -1;
//enum EPairMode
//{
//    PAIR_EXHAUSTIVE = 0,全部配对
//    PAIR_CONTIGUOUS = 1,相邻配对
//    PAIR_FROM_FILE  = 2从文件选择配对方法
//};

// 从文件的匹配文件的路径
std::string sPredefinedPairList = "";
// 匹配器
std::string sNearestMatchingMethod = "AUTO";
// 是否重新全部配对，不使用之前已经计算出的成果
bool bForce = false;
// 函数Robust_model_estimation中讲解
bool bGuided_matching = false;
// funtor GeometricFilter_FMatrix_AC中讲解
int imax_iteration = 2048;
// 0->默认区域提供程序（在内存中加载和存储所有区域）
// other->缓存区域提供程序（按需加载和存储区域）
unsigned int ui_max_cache_size = 0;
~~~
