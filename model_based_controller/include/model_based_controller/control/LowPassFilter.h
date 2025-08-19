//
// Created by ray on 25-8-19.
//


#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H


class LowPassFilter {
public:
    LowPassFilter(double samplePeriod, double cutFrequency);

    ~LowPassFilter() = default;

    void addValue(double newValue);

    [[nodiscard]] double getValue() const;

    void clear();

private:
    double weight_;
    double pass_value_{};
    bool start_;
};


#endif //LOWPASSFILTER_H
