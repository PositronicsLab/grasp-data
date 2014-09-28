#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

#include <sstream>
#include "log.h"

namespace gazebo
{
  class SystemGUI : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~SystemGUI()
    {
      this->connections.clear();
      if (this->userCam)
        this->userCam->EnableSaveFrame(false);
      this->userCam.reset();

      if( time_log )
        time_log->close(); 
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      this->connections.push_back(
          event::Events::ConnectPreRender(
            boost::bind(&SystemGUI::Update, this)));

      std::cout << "render_plugin loaded\n";
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
      // Get scene pointer
      rendering::ScenePtr scene = rendering::get_scene();

      // Wait until the scene is initialized.
      if( ( !scene || !scene->GetInitialized() ) ) {
        return;
      }

      static int frame = 1;
      static gazebo::common::Time last_sim_time = scene->GetSimTime();
      gazebo::common::Time sim_time = scene->GetSimTime();

      if( !this->userCam ) {
        // Get a pointer to the active user camera
        this->userCam = gui::get_active_camera();
        // Specify the path to save frames into
        std::stringstream ss_path;
        static std::string scene_name = scene->GetName();

        ss_path << "/tmp/gazebo_frames/" << scene_name; 
        if( !time_log ) {
          std::stringstream ss_log;
          ss_log << ss_path.str() << "/time.log";
          time_log = boost::shared_ptr<log_c>( new log_c( ss_log.str() ) );
          time_log->open();

          std::string log_hdr = "frame,sim_time\n";
          time_log->write( log_hdr );
        }

        this->userCam->SetSaveFramePathname( ss_path.str() );
        // Enable saving frames for initial frame
        this->userCam->EnableSaveFrame(true);
      } else {
        if( sim_time == last_sim_time ) {
          // Disable saving frames
          this->userCam->EnableSaveFrame(false);

          return;
        }
      }
      // Enable saving frames
      this->userCam->EnableSaveFrame(true);

/*
      // Look for a specific visual by name.
      if (scene->GetVisual("ground_plane"))
        std::cout << "Has ground plane visual\n";
*/
      std::stringstream ss_log_data;
      ss_log_data << frame << "," << sim_time.Double() << std::endl;
      time_log->write( ss_log_data.str() );
 
      last_sim_time = sim_time;
      frame++;
    }

    /// Pointer the user camera.
    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;

    private: boost::shared_ptr<log_c> time_log;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(SystemGUI)
}
