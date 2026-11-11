//
// Created by biao on 24-9-18.
//


#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H
#include <chrono>
#include <controller_common/common/enumClass.h>
#include <unitree_guide_controller/common/mathTypes.h>

inline long long getSystemTime() {
    const auto now = std::chrono::system_clock::now();
    const auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

class WaveGenerator {
public:
    WaveGenerator(double period, double st_ratio, const Vec4 &bias);

    ~WaveGenerator() = default;

    /**
     * Feed the current simulation time (seconds) before update(). The gait
     * phase must advance with simulated time: driving it from the wall clock
     * makes the phase jump whenever the simulator stalls (map publishing, a
     * slow frame), which shows up as a sudden foot-placement jump and a fall.
     */
    void setTime(double now_seconds) { now_ = now_seconds; }

    void update();

    // Start a newly entered gait from a four-foot support phase.
    void resetToStance();

    /**
     * Re-configure the gait clock. Only takes effect on the next gait entry,
     * because the phase is derived from the elapsed time since start_t_.
     * Values outside [0.3, 1.5] s and (0.4, 0.9) are clamped.
     */
    void setGaitParams(double period, double st_ratio);

    [[nodiscard]] double get_t_stance() const { return period_ * st_ratio_; }
    [[nodiscard]] double get_t_swing() const { return period_ * (1 - st_ratio_); }
    [[nodiscard]] double get_t() const { return period_; }

    Vec4 phase_;
    VecInt4 contact_;
    WaveStatus status_{};

private:
    /**
     * Update phase, contact and status based on current time.
     * @param phase foot phase
     * @param contact foot contact
     * @param status Wave Status
     */
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

    double period_{};
    double st_ratio_{}; // stance phase ratio
    Vec4 bias_;

    Vec4 normal_t_; // normalize time [0,1)
    Vec4 phase_past_; // foot phase
    VecInt4 contact_past_; // foot contact
    VecInt4 switch_status_;
    WaveStatus status_past_;

    long start_t_{};
    double now_{0.0};
};


#endif //WAVEGENERATOR_H
