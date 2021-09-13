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
| clustering  | 单元格 |
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
输入图像和相机数据库，输出sfm_data.json包含图像尺寸，焦距（单位像素）
~~~
$ openMVG_main_SfMInit_ImageListing -i ./ImageDataset_SceauxCastle/img -o ./ImageDataset_SceauxCastle/sfm_out/matches -d /home/mitom/Music/openMVG/src/openMVG/exif/sensor_width_database/sensor_width_camera_database.txt -c 3
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
Poses poses;   /// 储存影像外参数，包括旋转矩阵、平移矩阵
Intrinsics intrinsics; /// 储存影像内参数，支持多组不同内参数
Landmarks structure;   /// 物方点信息，物方点坐标及其tracks
}
~~~
