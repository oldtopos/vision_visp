
#ifndef __visp_tracker__MODELBASEDSETTINGSCONFIG_H__
#define __visp_tracker__MODELBASEDSETTINGSCONFIG_H__

namespace visp_tracker
{
    class Tracker;
    
    class ModelBasedSettingsConfig
    {
       public:
        ModelBasedSettingsConfig() {}
        ModelBasedSettingsConfig( Tracker * theTracker );
        ~ModelBasedSettingsConfig() { };
        
        
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
        int mask_border_;
        int max_features_;
        int window_size_;
        double quality_;
        double min_distance_;
        double harris_;
        int size_block_;
        int pyramid_lvl_;
        
        bool state;
        std::string name;

    };

} // namespace

#endif
