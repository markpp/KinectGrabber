// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>

typedef pcl::PointXYZRGBA PointType;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);

//Global point for keyboard
boost::shared_ptr<pcl::Grabber> grabber;

int main( int argc, char* argv[] )
{
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    // Retrieved Point Cloud Callback Function
    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );
            cloud = ptr;		
        };

    // Kinect2Grabber
    grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

    // Start Grabber
    grabber->start();

	viewer->registerKeyboardCallback(keyboardEventOccurred);

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( cloud && lock.owns_lock() ){
            if( cloud->size() != 0 ){
                /* Processing to Point Cloud */

                // Update Point Cloud
                if( !viewer->updatePointCloud( cloud, "cloud" ) ){
                    viewer->addPointCloud( cloud, "cloud" );
                }
            }
        }
    }

    // Stop Grabber
    grabber->stop();
    
    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event)
{
	if (event.getKeySym() == "c" && event.keyDown())
	{
		std::cout << "c was pressed => recording frames" << std::endl;
		boost::shared_ptr<pcl::Kinect2Grabber> p = boost::dynamic_pointer_cast<pcl::Kinect2Grabber, pcl::Grabber>(grabber);
		if (p != nullptr)
		{
			// It is safe to dereference p
			p->toggleRecord();
		}
	}
}