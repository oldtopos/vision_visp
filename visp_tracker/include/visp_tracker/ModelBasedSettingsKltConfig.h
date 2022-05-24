

#ifndef __visp_tracker__MODELBASEDSETTINGSKLTCONFIG_H__
#define __visp_tracker__MODELBASEDSETTINGSKLTCONFIG_H__

namespace visp_tracker
{
    class Tracker;
    
    class ModelBasedSettingsKltConfig
    {
       public:
        ModelBasedSettingsKltConfig() {}
        ModelBasedSettingsKltConfig( Tracker * theTracker );
        
        ~ModelBasedSettingsKltConfig() { };
        
        double angle_appear_;
        double angle_disappear_;
        int mask_border_;
        int max_features_;
        int window_size_;
        double quality_;
        double min_distance_;
        double harris_;
        int size_block_;
        int pyramid_lvl_;

        bool state_;
        std::string name_;

    };

} // namespace

#endif
