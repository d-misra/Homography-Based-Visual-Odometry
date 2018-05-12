
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <iostream>
#include <chrono>

class VideoLoop
{
public:
    VideoLoop(std::string _filename)
    {
        // open this video
        cap_.open(_filename);
        // get number of frames
        frame_end_ = cap_.get(cv::CAP_PROP_FRAME_COUNT);
        // begin with frame 0
        frame_idx_ = 0;
        // begin with positive direction
        frame_dir_ = 1;

    }

    void read(cv::Mat &_im)
    {
        // read this frame
        cap_ >> _im;

        // update frame index
        frame_idx_ += frame_dir_;
        // go back if last frame
        if(frame_idx_ >= frame_end_)
        {
            frame_dir_ = -1;
            frame_idx_ = frame_end_-1;
        }
        else if(frame_idx_ <= 0)
        {
            frame_dir_ = 1;
            frame_idx_ = 0;
        }
        // go to next frame to be read
        cap_.set(cv::CAP_PROP_POS_FRAMES, frame_idx_);

    }

protected:
    cv::VideoCapture cap_;
    int frame_end_, frame_idx_, frame_dir_;
};
