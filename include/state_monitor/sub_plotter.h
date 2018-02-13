#ifndef SUB_PLOTTER_STATE_MONITOR_H
#define SUB_PLOTTER_STATE_MONITOR_H

#include <mgl2/mgl.h>

template <size_t data_dim_>
class PlotterData {
 public:
  PlotterData(const double keep_data_for_secs)
      : data_ptr_offset_(0),
        keep_data_for_secs_(keep_data_for_secs),
        max_data_value_(std::make_pair(-1, std::numeric_limits<mreal>::min())),
        min_data_value_(std::make_pair(-1, std::numeric_limits<mreal>::max())) {
  }

  template <typename... Args>
  void addDataPoint(const double time_secs, Args... args) {
    static_assert(sizeof...(Args) == data_dim_,
                  "addDataPoint() needs data_dim_ + 1 arguments (the point to "
                  "insert + when it happened)");
    time_data_.push_back(time_secs);

    addDataPointRecursive(args...);

    removeOldData();
    findMinAndMax();
  }

  void getMGLData(mglData *mgl_time_data,
                  std::array<mglData, data_dim_> *mgl_variable_data) {
    mgl_time_data->Link(&(time_data_[data_ptr_offset_]),
                        time_data_.size() - data_ptr_offset_);

    for (size_t i = 0; i < data_dim_; ++i) {
      mgl_variable_data->at(i)
          .Link(&(variable_data_[i][data_ptr_offset_]),
                variable_data_[i].size() - data_ptr_offset_);
    }
  }

  size_t getNumDataPoints() const { return time_data_.size(); }

  double getMaxDataValue() const { return max_data_value_.second; }

  double getMinDataValue() const { return min_data_value_.second; }

  double getMaxTimeValue() const {
    if (time_data_.size() == 0) {
      return 0;
    }
    return time_data_.back();
  }

  double getMinTimeValue() const {
    if (time_data_.size() == 0) {
      return 0;
    }
    return time_data_[data_ptr_offset_];
  }

 private:
  template <typename... Args>
  void addDataPointRecursive(const double data_point, Args... args) {
    variable_data_[data_dim_ - (sizeof...(Args) + 1)].push_back(data_point);
    addDataPointRecursive(args...);
  }
  void addDataPointRecursive(const double highest_dim_data_point) {
    variable_data_.back().push_back(highest_dim_data_point);
  }

  void removeOldData() {
    // check if a bag has looped (new reading is from a long way in the past)
    if ((time_data_.size() > 1) &&
        ((time_data_.back() + keep_data_for_secs_) < time_data_.rbegin()[1])) {
      mreal temp = time_data_.back();
      time_data_.clear();
      time_data_.push_back(temp);

      for (std::vector<mreal> &element_data : variable_data_) {
        temp = element_data.back();
        element_data.clear();
        element_data.push_back(temp);
      }

      data_ptr_offset_ = 0;
      max_data_value_ = std::make_pair(-1, std::numeric_limits<mreal>::min());
      min_data_value_ = std::make_pair(-1, std::numeric_limits<mreal>::max());

      findMinAndMax();
    }

    // move data ptr to keep only keep_data_for_secs data in range
    while ((time_data_[data_ptr_offset_] + keep_data_for_secs_) <
           time_data_.back()) {
      ++data_ptr_offset_;
    }

    // periodically remove old data (wasteful on ram, but we usually only have a
    // few thousand points)
    if ((time_data_.back() - time_data_.front()) >
        kTimePaddingFactor * keep_data_for_secs_) {
      time_data_.erase(time_data_.begin(),
                       time_data_.begin() + data_ptr_offset_);
      for (std::vector<mreal> &element_data : variable_data_) {
        element_data.erase(element_data.begin(),
                           element_data.begin() + data_ptr_offset_);
      }

      max_data_value_.first -= data_ptr_offset_;
      min_data_value_.first -= data_ptr_offset_;
      data_ptr_offset_ = 0;
    }
  }

  void findMinAndMax() {
    if (variable_data_.front().empty()) {
      return;
    }

    // find new min and max values
    for (const std::vector<mreal> &element_data : variable_data_) {
      if (max_data_value_.second <= element_data.back()) {
        max_data_value_ =
            std::make_pair(element_data.size() - 1, element_data.back());
      }
      if (min_data_value_.second >= element_data.back()) {
        min_data_value_ =
            std::make_pair(element_data.size() - 1, element_data.back());
      }
    }

    // if our old point is off the axes we need to rescan all points (expensive,
    // but probably cheaper than maintaining a sorted list of points)
    if (max_data_value_.first < data_ptr_offset_) {
      max_data_value_ = std::make_pair(0, std::numeric_limits<mreal>::min());
      for (const std::vector<mreal> &element_data : variable_data_) {
        for (size_t i = data_ptr_offset_; i < element_data.size(); ++i) {
          if (max_data_value_.second < element_data.back()) {
            max_data_value_ = std::make_pair(i, element_data[i]);
          }
        }
      }
    }
    if (min_data_value_.first < data_ptr_offset_) {
      min_data_value_ = std::make_pair(0, std::numeric_limits<mreal>::max());
      for (const std::vector<mreal> &element_data : variable_data_) {
        for (size_t i = data_ptr_offset_; i < element_data.size(); ++i) {
          if (max_data_value_.second > element_data.back()) {
            max_data_value_ = std::make_pair(i, element_data[i]);
          }
        }
      }
    }
  }

  // we keep the data for kTimePaddingFactor times much longer than specified to
  // help minimize the number of times we need to erase from the front of a
  // vector
  static constexpr double kTimePaddingFactor = 10.0;

  std::vector<mreal> time_data_;
  size_t data_ptr_offset_;
  const mreal keep_data_for_secs_;
  std::array<std::vector<mreal>, data_dim_> variable_data_;
  std::pair<int, mreal> max_data_value_, min_data_value_;
};

template <size_t data_dim_>
class SubPlotter {
 public:
  SubPlotter(const std::shared_ptr<mglGraph> &gr,
             const double keep_data_for_secs, const std::string &title,
             const size_t num_subplots_wide, const size_t num_subplots_high,
             const size_t subplot_idx)
      : gr_(gr),
        plotter_data_(keep_data_for_secs),
        title_(title),
        num_subplots_wide_(num_subplots_wide),
        num_subplots_high_(num_subplots_high),
        subplot_idx_(subplot_idx){};

  template <typename... Args>
  void addDataPoint(const double time_secs, Args... args) {
    plotter_data_.addDataPoint(time_secs, args...);
  }

  void plot() const {
    mglData mgl_time_data;
    std::array<mglData, data_dim_> mgl_variable_data;
    plotter_data_.getMGLData(&mgl_time_data, &mgl_variable_data);

    gr_->SubPlot(num_subplots_wide_, num_subplots_high_, subplot_idx_, "");

    if (plotter_data_.getNumDataPoints() > 0) {
      constexpr mreal kRangePad = 1e-4;
      constexpr mreal kPadAxes = 1.05;
      mreal min_data_value = plotter_data_.getMinDataValue();
      mreal max_data_value = plotter_data_.getMaxDataValue();
      const mreal range = max_data_value - min_data_value + kRangePad;
      min_data_value = min_data_value - kPadAxes*range;
      max_data_value = max_data_value + kPadAxes*range;

      gr_->SetRanges(
          plotter_data_.getMinTimeValue(), plotter_data_.getMaxTimeValue(),
          min_data_value, max_data_value);

      for (mglData &mgl_element_data : mgl_variable_data) {
        gr_->Plot(mgl_time_data, mgl_element_data, "-");
      }
    }

    gr_->SetFontSizePT(5);
    gr_->Title(title_.c_str(), "w");
    gr_->SetFontSizePT(10);
    gr_->SetOrigin(NAN, NAN);
    gr_->Axis("y", "w");
  }

 private:
  std::shared_ptr<mglGraph> gr_;
  // mathgl does not beleive in const methods, hence the mutable
  mutable PlotterData<data_dim_> plotter_data_;

  const std::string title_;
  const size_t num_subplots_wide_;
  const size_t num_subplots_high_;
  const size_t subplot_idx_;
};

#endif  // SUB_PLOTTER_STATE_MONITOR_H