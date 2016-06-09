// Kinect2Grabber is pcl::Grabber to retrieve the point cloud data from Kinect v2 using Kinect for Windows SDK 2.x.
// This source code is licensed under the MIT license. Please see the License in License.txt.

#ifndef KINECT2_GRABBER
#define KINECT2_GRABBER

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2\opencv.hpp>


namespace pcl
{
	struct pcl::PointXYZ;
	struct pcl::PointXYZI;
	struct pcl::PointXYZRGB;
	struct pcl::PointXYZRGBA;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease(Interface *& IRelease)
	{
		if (IRelease != NULL) {
			IRelease->Release();
			IRelease = NULL;
		}
	}

	class Kinect2Grabber : public pcl::Grabber
	{
	public:
		Kinect2Grabber();
		virtual ~Kinect2Grabber() throw ();
		virtual void start();
		virtual void stop();
		virtual bool isRunning() const;
		virtual std::string getName() const;
		virtual float getFramesPerSecond() const;
		void toggleRecord();

		typedef void (signal_Kinect2_PointXYZ)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ>>&);
		typedef void (signal_Kinect2_PointXYZI)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZI>>&);
		typedef void (signal_Kinect2_PointXYZRGB)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB>>&);
		typedef void (signal_Kinect2_PointXYZRGBA)(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA>>&);

	protected:
		boost::signals2::signal<signal_Kinect2_PointXYZ>* signal_PointXYZ;
		boost::signals2::signal<signal_Kinect2_PointXYZI>* signal_PointXYZI;
		boost::signals2::signal<signal_Kinect2_PointXYZRGB>* signal_PointXYZRGB;
		boost::signals2::signal<signal_Kinect2_PointXYZRGBA>* signal_PointXYZRGBA;

		pcl::PointCloud<pcl::PointXYZ>::Ptr convertDepthToPointXYZ(UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZI>::Ptr convertInfraredDepthToPointXYZI(UINT16* infraredBuffer, UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB(RGBQUAD* colorBuffer, UINT16* depthBuffer);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr convertRGBADepthToPointXYZRGBA(RGBQUAD* colorBuffer, UINT16* depthBuffer);

		boost::thread thread;
		mutable boost::mutex mutex;

		void threadFunction();

		bool quit;
		bool running;
		bool record;

		HRESULT result;
		IKinectSensor* sensor;
		ICoordinateMapper* mapper;
		IColorFrameSource* colorSource;
		IColorFrameReader* colorReader;
		IDepthFrameSource* depthSource;
		IDepthFrameReader* depthReader;
		IInfraredFrameSource* infraredSource;
		IInfraredFrameReader* infraredReader;

		int colorWidth;
		int colorHeight;
		std::vector<RGBQUAD> colorBuffer;

		int depthWidth;
		int depthHeight;
		std::vector<UINT16> depthBuffer;

		int infraredWidth;
		int infraredHeight;
		std::vector<UINT16> infraredBuffer;
	};

	pcl::Kinect2Grabber::Kinect2Grabber()
		: sensor(nullptr)
		, mapper(nullptr)
		, colorSource(nullptr)
		, colorReader(nullptr)
		, depthSource(nullptr)
		, depthReader(nullptr)
		, infraredSource(nullptr)
		, infraredReader(nullptr)
		, result(S_OK)
		, colorWidth(1920)
		, colorHeight(1080)
		, colorBuffer()
		, depthWidth(512)
		, depthHeight(424)
		, depthBuffer()
		, infraredWidth(512)
		, infraredHeight(424)
		, infraredBuffer()
		, running(false)
		, quit(false)
		, record(false)
		, signal_PointXYZ(nullptr)
		, signal_PointXYZI(nullptr)
		, signal_PointXYZRGB(nullptr)
		, signal_PointXYZRGBA(nullptr)
	{
		// Create Sensor Instance
		result = GetDefaultKinectSensor(&sensor);
		if (FAILED(result)) {
			throw std::exception("Exception : GetDefaultKinectSensor()");
		}

		// Open Sensor
		result = sensor->Open();
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::Open()");
		}

		// Retrieved Coordinate Mapper
		result = sensor->get_CoordinateMapper(&mapper);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_CoordinateMapper()");
		}

		// Retrieved Color Frame Source
		result = sensor->get_ColorFrameSource(&colorSource);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_ColorFrameSource()");
		}

		// Retrieved Depth Frame Source
		result = sensor->get_DepthFrameSource(&depthSource);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_DepthFrameSource()");
		}

		// Retrieved Infrared Frame Source
		result = sensor->get_InfraredFrameSource(&infraredSource);
		if (FAILED(result)) {
			throw std::exception("Exception : IKinectSensor::get_InfraredFrameSource()");
		}

		// Retrieved Color Frame Size
		IFrameDescription* colorDescription;
		result = colorSource->get_FrameDescription(&colorDescription);
		if (FAILED(result)) {
			throw std::exception("Exception : IColorFrameSource::get_FrameDescription()");
		}

		result = colorDescription->get_Width(&colorWidth); // 1920
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Width()");
		}

		result = colorDescription->get_Height(&colorHeight); // 1080
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Height()");
		}

		SafeRelease(colorDescription);

		// To Reserve Color Frame Buffer
		colorBuffer.resize(colorWidth * colorHeight);

		// Retrieved Depth Frame Size
		IFrameDescription* depthDescription;
		result = depthSource->get_FrameDescription(&depthDescription);
		if (FAILED(result)) {
			throw std::exception("Exception : IDepthFrameSource::get_FrameDescription()");
		}

		result = depthDescription->get_Width(&depthWidth); // 512
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Width()");
		}

		result = depthDescription->get_Height(&depthHeight); // 424
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Height()");
		}

		SafeRelease(depthDescription);

		// To Reserve Depth Frame Buffer
		depthBuffer.resize(depthWidth * depthHeight);

		// Retrieved Infrared Frame Size
		IFrameDescription* infraredDescription;
		result = infraredSource->get_FrameDescription(&infraredDescription);
		if (FAILED(result)) {
			throw std::exception("Exception : IInfraredFrameSource::get_FrameDescription()");
		}

		result = infraredDescription->get_Width(&infraredWidth); // 512
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Width()");
		}

		result = infraredDescription->get_Height(&infraredHeight); // 424
		if (FAILED(result)) {
			throw std::exception("Exception : IFrameDescription::get_Height()");
		}

		SafeRelease(infraredDescription);

		// To Reserve Infrared Frame Buffer
		infraredBuffer.resize(infraredWidth * infraredHeight);

		signal_PointXYZ = createSignal<signal_Kinect2_PointXYZ>();
		signal_PointXYZI = createSignal<signal_Kinect2_PointXYZI>();
		signal_PointXYZRGB = createSignal<signal_Kinect2_PointXYZRGB>();
		signal_PointXYZRGBA = createSignal<signal_Kinect2_PointXYZRGBA>();
	}

	pcl::Kinect2Grabber::~Kinect2Grabber() throw()
	{
		stop();

		disconnect_all_slots<signal_Kinect2_PointXYZ>();
		disconnect_all_slots<signal_Kinect2_PointXYZI>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGB>();
		disconnect_all_slots<signal_Kinect2_PointXYZRGBA>();

		thread.join();

		// End Processing
		if (sensor) {
			sensor->Close();
		}
		SafeRelease(sensor);
		SafeRelease(mapper);
		SafeRelease(colorSource);
		SafeRelease(colorReader);
		SafeRelease(depthSource);
		SafeRelease(depthReader);
		SafeRelease(infraredSource);
		SafeRelease(infraredReader);
	}

	void pcl::Kinect2Grabber::start()
	{
		// Open Color Frame Reader
		result = colorSource->OpenReader(&colorReader);
		if (FAILED(result)) {
			throw std::exception("Exception : IColorFrameSource::OpenReader()");
		}

		// Open Depth Frame Reader
		result = depthSource->OpenReader(&depthReader);
		if (FAILED(result)) {
			throw std::exception("Exception : IDepthFrameSource::OpenReader()");
		}

		// Open Infrared Frame Reader
		result = infraredSource->OpenReader(&infraredReader);
		if (FAILED(result)) {
			throw std::exception("Exception : IInfraredFrameSource::OpenReader()");
		}

		running = true;

		thread = boost::thread(&Kinect2Grabber::threadFunction, this);
	}

	void pcl::Kinect2Grabber::stop()
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		quit = true;
		running = false;

		lock.unlock();
	}

	bool pcl::Kinect2Grabber::isRunning() const
	{
		boost::unique_lock<boost::mutex> lock(mutex);

		return running;

		lock.unlock();
	}

	std::string pcl::Kinect2Grabber::getName() const
	{
		return std::string("Kinect2Grabber");
	}

	float pcl::Kinect2Grabber::getFramesPerSecond() const
	{
		return 30.0f;
	}

	inline void Kinect2Grabber::toggleRecord()
	{
		boost::unique_lock<boost::mutex> lock(mutex);
		record = !record;
		lock.unlock();
	}

	void pcl::Kinect2Grabber::threadFunction()
	{
		//OpenCV vars
		int width = 512;
		int height = 424;
		std::ostringstream os;
		size_t imageCount_ = 0;
		ColorSpacePoint* depth2rgb = new ColorSpacePoint[512 * 424];     // Maps depth pixels to rgb pixels
		DepthSpacePoint* color2depth = new DepthSpacePoint[1920 * 1080];
		CameraSpacePoint* camerapoints = new CameraSpacePoint[1920 * 1080];

		while (!quit) {
			boost::unique_lock<boost::mutex> lock(mutex);

			// Acquire Latest Depth Frame first so mapp can be created.
			IDepthFrame* depthFrame = nullptr;
			result = depthReader->AcquireLatestFrame(&depthFrame);
			if (SUCCEEDED(result)) {
				// Retrieved Depth Data
				result = depthFrame->CopyFrameDataToArray(depthBuffer.size(), &depthBuffer[0]);
				if (FAILED(result)) {
					throw std::exception("Exception : IDepthFrame::CopyFrameDataToArray()");
				}
				else if (record)	//Save images to disk with opencv
				{
					//create rectification map for color
					//mapper->MapDepthFrameToColorSpace(512 * 424, &depthBuffer[0], 512 * 424, depth2rgb);
					//mapper->MapColorFrameToCameraSpace(512 * 424, &depthBuffer[0], 1920 * 1080, camerapoints);
					mapper->MapColorFrameToDepthSpace(512 * 424, &depthBuffer[0], 1920 * 1080, color2depth);
					//CameraIntrinsics	test;
					//mapper->GetDepthCameraIntrinsics(&test);

					os.str("");
					os.clear();
					os << "out//depth//img_" << std::setw(4) << std::setfill('0') << imageCount_ << ".png";
					cv::Mat bufferMat(height, width, CV_16UC1);
					bufferMat = cv::Mat(height, width, CV_16UC1, &depthBuffer[0]);	//copy data into buffer Mat
					cv::imwrite(os.str(), bufferMat);	//This saves the 16 bits per channel grey scale image
				}
			}
			SafeRelease(depthFrame);

			// Acquire Latest Color Frame
			IColorFrame* colorFrame = nullptr;
			result = colorReader->AcquireLatestFrame(&colorFrame);
			if (SUCCEEDED(result)) {
				// Retrieved Color Data
				colorBuffer.clear();
				colorBuffer.resize(1920 * 1080);
				result = colorFrame->CopyConvertedFrameDataToArray(colorBuffer.size() * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(&colorBuffer[0]), ColorImageFormat::ColorImageFormat_Bgra);
				if (FAILED(result)) {
					throw std::exception("Exception : IColorFrame::CopyConvertedFrameDataToArray()");
				}
				else if (record)	//Save images to disk with opencv
				{
					//Remap full res color to depth matched color
					cv::Mat rect_color = cv::Mat(424, 512, CV_8UC3);
					cv::MatIterator_<cv::Vec3b> it, end;
					//size_t	counter = 0;
					for (size_t i = 0; i < colorBuffer.size(); i++)
					{
						DepthSpacePoint p = color2depth[i];
						if (!(p.X < 0 || p.Y < 0 || p.X > 512 || p.Y > 424)) {	//check it is within depth bounds
							rect_color.at<cv::Vec3b>(p.Y, p.X)[0] = colorBuffer[i].rgbBlue;
							rect_color.at<cv::Vec3b>(p.Y, p.X)[1] = colorBuffer[i].rgbGreen;
							rect_color.at<cv::Vec3b>(p.Y, p.X)[2] = colorBuffer[i].rgbRed;
						}
					}
					//for (it = rect_color.begin<cv::Vec3b>(), end = rect_color.end<cv::Vec3b>(); it != end; ++it)
					//{
					//	DepthSpacePoint p = color2depth[counter];
					//	// Check if color pixel coordinates are in bounds
					//	if (p.X < 0 || p.Y < 0 || p.X > 1920 || p.Y > 1080) {
					//		(*it)[0] = 0;
					//		(*it)[1] = 0;
					//		(*it)[2] = 0;
					//	}
					//	else {
					//		int idx = (int)p.X + 1920*(int)p.Y;	//Get corresponding color pixel
					//		(*it)[0] = colorBuffer[idx].rgbBlue;	//copy RGB channels to MAT.
					//		(*it)[1] = colorBuffer[idx].rgbGreen;
					//		(*it)[2] = colorBuffer[idx].rgbRed;
					//	}
					//	counter++;
					//}
					os.str("");
					os.clear();
					os << "out//rect_color//img_" << std::setw(4) << std::setfill('0') << imageCount_ << ".png";
					//cv::Mat bufferMat(cheight, cwidth, CV_8UC4);
					//bufferMat = cv::Mat(cheight, cwidth, CV_8UC4, reinterpret_cast<void*>(&colorBuffer[0]));	//copy data into buffer Mat
					cv::imwrite(os.str(), rect_color);	//Save RGB Color 8bbp
				}
			}
			SafeRelease(colorFrame);


			// Acquire Latest Infrared Frame
			IInfraredFrame* infraredFrame = nullptr;
			result = infraredReader->AcquireLatestFrame(&infraredFrame);
			if (SUCCEEDED(result)) {
				// Retrieved Infrared Data
				result = infraredFrame->CopyFrameDataToArray(infraredBuffer.size(), &infraredBuffer[0]);
				if (FAILED(result)) {
					throw std::exception("Exception : IInfraredFrame::CopyFrameDataToArray()");
				}
			}
			SafeRelease(infraredFrame);


			lock.unlock();

			if (signal_PointXYZ->num_slots() > 0) {
				signal_PointXYZ->operator()(convertDepthToPointXYZ(&depthBuffer[0]));
			}

			if (signal_PointXYZI->num_slots() > 0) {
				signal_PointXYZI->operator()(convertInfraredDepthToPointXYZI(&infraredBuffer[0], &depthBuffer[0]));
			}

			if (signal_PointXYZRGB->num_slots() > 0) {
				signal_PointXYZRGB->operator()(convertRGBDepthToPointXYZRGB(&colorBuffer[0], &depthBuffer[0]));
			}

			if (signal_PointXYZRGBA->num_slots() > 0) {
				pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp = convertRGBADepthToPointXYZRGBA(&colorBuffer[0], &depthBuffer[0]);
				signal_PointXYZRGBA->operator()(temp);
				if (record)
				{
					//Save point cloud
					os.str("");
					os.clear();
					os << "out//clouds//cloud_" << std::setw(4) << std::setfill('0') << imageCount_ << ".pcd";
					pcl::io::savePCDFile(os.str(), *temp.get(), true);
					imageCount_++;
				}
			}
		}
		delete	depth2rgb;
		delete color2depth;
		delete camerapoints;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl::Kinect2Grabber::convertDepthToPointXYZ(UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZ* pt = &cloud->points[0];
		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++, pt++) {
				pcl::PointXYZ point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				point.x = cameraSpacePoint.X;
				point.y = cameraSpacePoint.Y;
				point.z = cameraSpacePoint.Z;

				*pt = point;
			}
		}

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZI>::Ptr pcl::Kinect2Grabber::convertInfraredDepthToPointXYZI(UINT16* infraredBuffer, UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZI* pt = &cloud->points[0];
		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++, pt++) {
				pcl::PointXYZI point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Setting PointCloud Intensity
				point.intensity = static_cast<float>(infraredBuffer[y * depthWidth + x]);

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				point.x = cameraSpacePoint.X;
				point.y = cameraSpacePoint.Y;
				point.z = cameraSpacePoint.Z;

				*pt = point;
			}
		}

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl::Kinect2Grabber::convertRGBDepthToPointXYZRGB(RGBQUAD* colorBuffer, UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZRGB* pt = &cloud->points[0];
		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++, pt++) {
				pcl::PointXYZRGB point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGB
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				mapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
					point.b = color.rgbBlue;
					point.g = color.rgbGreen;
					point.r = color.rgbRed;
				}

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				}

				*pt = point;
			}
		}

		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcl::Kinect2Grabber::convertRGBADepthToPointXYZRGBA(RGBQUAD* colorBuffer, UINT16* depthBuffer)
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());

		cloud->width = static_cast<uint32_t>(depthWidth);
		cloud->height = static_cast<uint32_t>(depthHeight);
		cloud->is_dense = false;

		cloud->points.resize(cloud->height * cloud->width);

		pcl::PointXYZRGBA* pt = &cloud->points[0];
		for (int y = 0; y < depthHeight; y++) {
			for (int x = 0; x < depthWidth; x++, pt++) {
				pcl::PointXYZRGBA point;

				DepthSpacePoint depthSpacePoint = { static_cast<float>(x), static_cast<float>(y) };
				UINT16 depth = depthBuffer[y * depthWidth + x];

				// Coordinate Mapping Depth to Color Space, and Setting PointCloud RGBA
				ColorSpacePoint colorSpacePoint = { 0.0f, 0.0f };
				mapper->MapDepthPointToColorSpace(depthSpacePoint, depth, &colorSpacePoint);
				int colorX = static_cast<int>(std::floor(colorSpacePoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorSpacePoint.Y + 0.5f));
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					RGBQUAD color = colorBuffer[colorY * colorWidth + colorX];
					point.b = color.rgbBlue;
					point.g = color.rgbGreen;
					point.r = color.rgbRed;
					point.a = color.rgbReserved;
				}

				// Coordinate Mapping Depth to Camera Space, and Setting PointCloud XYZ
				CameraSpacePoint cameraSpacePoint = { 0.0f, 0.0f, 0.0f };
				mapper->MapDepthPointToCameraSpace(depthSpacePoint, depth, &cameraSpacePoint);
				if ((0 <= colorX) && (colorX < colorWidth) && (0 <= colorY) && (colorY < colorHeight)) {
					point.x = cameraSpacePoint.X;
					point.y = cameraSpacePoint.Y;
					point.z = cameraSpacePoint.Z;
				}

				*pt = point;
			}
		}

		return cloud;
	}
}

#endif KINECT2_GRABBER

