

#ifndef __visp_tracker__MODELBASEDSETTINGSEDGECONFIG_H__
#define __visp_tracker__MODELBASEDSETTINGSEDGECONFIG_H__

namespace visp_tracker
{
    class Tracker;
    
    class ModelBasedSettingsEdgeConfig
    {
       public:
        ModelBasedSettingsEdgeConfig() {}
        ModelBasedSettingsEdgeConfig( Tracker * theTracker );
        
        ~ModelBasedSettingsEdgeConfig() { };
        
        double angle_appear_;
        double angle_disappear_;
        int mask_size_;
        int range_;
        double threshold_;
        double mu1_;
        double mu2_;
        double sample_step_;
        int strip_;
        double first_threshold_;
    
        bool state_;
        std::string name_;

    };

} // namespace

#endif
