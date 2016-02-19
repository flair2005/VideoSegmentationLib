#ifndef PARAMETERS_H
#define PARAMETERS_H


class SegmentationParameters {

public:

    SegmentationParameters(): m_threshold(0.06){
        scales = 4;
        starting_scale = 2;
        scale_for_propagation = 0;
        video_segmentation = false;
        sp = 15;
        sr = 15;
        min_size = 30;

    }

    SegmentationParameters(double threshold): m_threshold(threshold){
        scales = 4;
        starting_scale = 2;
        scale_for_propagation = 0;
        video_segmentation = false;
        sp = 15;
        sr = 15;
        min_size = 15;

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

    void setSP(int sp){
        this->sp = sp;
    }

    void setSR(int sr){
        this->sr = sr;
    }

    void setMinSize(int min){
        this->min_size = min;
    }

    int getSP(){
        return sp;
    }

    int getSR(){
        return sr;
    }

    int getMinSize(){
        return min_size;
    }

private:
    double m_threshold;
    int scales;
    int starting_scale;
    int scale_for_propagation;
    bool video_segmentation;

    //mean shift parameters
    int sp, sr, min_size;

};

#endif // PARAMETERS_H
