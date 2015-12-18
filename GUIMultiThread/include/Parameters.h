#ifndef PARAMETERS_H
#define PARAMETERS_H


class SegmentationParameters {

public:

    SegmentationParameters(): m_threshold(0.06){
    }

    SegmentationParameters(double threshold): m_threshold(threshold){
    }

    double getThreshold(){
        return m_threshold;
    }

    void setThreshold(double threshold){
        m_threshold = threshold;
    }

private:
    double m_threshold;
};

#endif // PARAMETERS_H
