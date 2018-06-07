#pragma once

#include <global.h>
#include <feature_detector.h>
#include <matcher.h>
#include <queue>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <utility.h>
#include <performance_monitor.h>

namespace sdvio
{

class Frame;
class Feature;
class Point;

/// A seed is a probabilistic depth estimate for a single pixel.
struct Seed
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static int batch_counter;
  static int seed_counter;
  int batch_id;                //!< Batch id is the id of the keyframe for which the seed was created.
  int id;                      //!< Seed ID, only used for visualization.
  Feature* ftr;                //!< Feature in the keyframe for which the depth should be computed.
  float a;                     //!< a of Beta distribution: When high, probability of inlier is large.
  float b;                     //!< b of Beta distribution: When high, probability of outlier is large.
  float mu;                    //!< Mean of normal distribution.
  float z_range;               //!< Max range of the possible depth.
  float sigma2;                //!< Variance of normal distribution.
  Matrix2d patch_cov;          //!< Patch covariance in reference image.
  Seed(Feature* ftr, float depth_mean, float depth_min);
};

class DepthFilter
{
public:
    
    typedef boost::unique_lock<boost::mutex> lock_t;
    typedef boost::function<void ( Point*, double )> callback_t;
  
    /// Depth-filter config parameters
    struct Options
    {
	bool check_ftr_angle;                       //!< gradient features are only updated if the epipolar line is orthogonal to the gradient.
	bool epi_search_1d;                         //!< restrict Gauss Newton in the epipolar search to the epipolar line.
	bool verbose;                               //!< display output.
	bool use_photometric_disparity_error;       //!< use photometric disparity error instead of 1px error in tau computation.
	int max_n_kfs;                              //!< maximum number of keyframes for which we maintain seeds.
	double sigma_i_sq;                          //!< image noise.
	double seed_convergence_sigma2_thresh;      //!< threshold on depth uncertainty for convergence.
	Options(): 
	    check_ftr_angle(false),
	    epi_search_1d(false),
	    verbose(false),
	    use_photometric_disparity_error(false),
	    max_n_kfs(3),
	    sigma_i_sq(5e-4),
	    seed_convergence_sigma2_thresh(200.0)
	{}
    } options_;
    
    DepthFilter(
	feature_detection::DetectorPtr feature_detector,
	callback_t seed_converged_cb);

    virtual ~DepthFilter();

    /// Start this thread when seed updating should be in a parallel thread.
    void startThread();

    /// Stop the parallel thread that is running.
    void stopThread();

    /// Add frame to the queue to be processed.
    void addFrame(FramePtr frame);

    /// Add new keyframe to the queue
    void addKeyframe(FramePtr frame, double depth_mean, double depth_min);

    /// Remove all seeds which are initialized from the specified keyframe. This
    /// function is used to make sure that no seeds points to a non-existent frame
    /// when a frame is removed from the map.
    void removeKeyframe(FramePtr frame);

    /// If the map is reset, call this function such that we don't have pointers
    /// to old frames.
    void reset();

    /// Returns a copy of the seeds belonging to frame. Thread-safe.
    /// Can be used to compute the Next-Best-View in parallel.
    /// IMPORTANT! Make sure you hold a valid reference counting pointer to frame
    /// so it is not being deleted while you use it.
    void getSeedsCopy(const FramePtr& frame, std::list<Seed>& seeds);

    /// Return a reference to the seeds. This is NOT THREAD SAFE!
    std::list<Seed>& getSeeds() { return seeds_; }

    /// Bayes update of the seed, x is the measurement, tau2 the measurement uncertainty
    static void updateSeed(
	const float x,
	const float tau2,
	Seed* seed);

    /// Compute the uncertainty of the measurement.
    static double computeTau(
	const SE3& T_ref_cur,
	const Vector3d& f,
	const double z,
	const double px_error_angle);

protected:
    feature_detection::DetectorPtr feature_detector_;
    callback_t seed_converged_cb_;
    std::list<Seed> seeds_;
    boost::mutex seeds_mut_;
    bool seeds_updating_halt_;            //!< Set this value to true when seeds updating should be interrupted.
    boost::thread* thread_;
    std::queue<FramePtr> frame_queue_;
    boost::mutex frame_queue_mut_;
    boost::condition_variable frame_queue_cond_;
    FramePtr new_keyframe_;               //!< Next keyframe to extract new seeds.
    bool new_keyframe_set_;               //!< Do we have a new keyframe to process?.
    double new_keyframe_min_depth_;       //!< Minimum depth in the new keyframe. Used for range in new seeds.
    double new_keyframe_mean_depth_;      //!< Maximum depth in the new keyframe. Used for range in new seeds.
//     utility::PerformanceMonitor permon_;       //!< Separate performance monitor since the DepthFilter runs in a parallel thread.
    Matcher matcher_;

    /// Initialize new seeds from a frame.
    void initializeSeeds(FramePtr frame);

    /// Update all seeds with a new measurement frame.
    virtual void updateSeeds(FramePtr frame);

    /// When a new keyframe arrives, the frame queue should be cleared.
    void clearFrameQueue();

    /// A thread that is continuously updating the seeds.
    void updateSeedsLoop();
};
    

}