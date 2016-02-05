#ifndef PARAMETERS_H
#define PARAMETERS_H


class SegmentationParameters {

public:

    SegmentationParameters(): m_threshold(0.06){
        scales = 4;
        starting_scale = 2;
        scale_for_propagation = 0;
        video_segmentation = false;

    }

    SegmentationParameters(double threshold): m_threshold(threshold){
        scales = 4;
        starting_scale = 2;
        scale_for_propagation = 0;
        video_segmentation = false;

    }

    double getThreshold(){
        return m_threshold;
    }

    void setThreshold(double threshold){
        m_threshold = threshold;
    }

    int getScales(){
        return scales;
    }

    void setScales(int s){
        scales = s;
    }

    int getStartingScale(){
        return starting_scale;
    }

    void setStartingScale(int start_scale){
        starting_scale = start_scale;
    }

    int getScaleForPropagation(){
        return scale_for_propagation;
    }

    void setScaleForPropagation(int scale){
        scale_for_propagation = scale;
    }

private:
    double m_threshold;
    int scales;
    int starting_scale;
    int scale_for_propagation;
    bool video_segmentation;

};

#endif // PARAMETERS_H
