//
// Created by ray on 2025-07-25.
//


#ifndef QMLOWPASSFILTER_H
#define QMLOWPASSFILTER_H


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


#endif //QMLOWPASSFILTER_H
