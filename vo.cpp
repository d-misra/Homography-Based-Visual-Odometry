#include <ecn_visualodom/video_loop.h>
#include <ecn_visualodom/visual_odom.h>
#include <ecn_visualodom/ar_display.h>

using namespace std;


int main(int argc, char ** argv)
{

    // video source and image
    VideoLoop cap("../armen.mp4");
    cv::Mat im;
    cap.read(im);

    // camera parameters
    vpCameraParameters cam(684.169232, 680.105767, 365.163397, 292.358876);

    // visual odometer
    bool relative_to_initial = false;
    VisualOdom vo(cam, relative_to_initial);

    // we look for a more or less horizontal plane
    // vo.setNormalGuess(TO DO);

    vo.setNormalGuess(0,-1,0);

    // AR
    ModelDisplay ar(cam, im, true);

    // initial pose: translation known, rotation to be estimated later
    vpHomogeneousMatrix cMo0(-0.1,-0.1,0.7,0,0,0);

    // cMo = current pose, M = relative pose between two images (from previous to next)
    vpHomogeneousMatrix cMo, M;

    // loop variables
    vpRotationMatrix R;
    vpColVector nor;
    vpTranslationVector t;
    double d;
    bool compute_initial_pose = true;
    int skip = 1;
    int skipped = 0;

    while(ar.continueRendering())
    {
        // get next image
        cap.read(im);

        // process and compute 2M1
        // returns True if succeded
        if(vo.process(im,M) && skipped == 0)
        {
            if(compute_initial_pose)
            {
                // get normal to plane
                vo.getNormal(nor);
                // initialize absolute pose from normal
                vo.getRotationFromNormal(R);
                // update initial pose
                cMo0.insert(R);

                // write current estimation
                cMo = cMo0;

                // compute and update distance estimate from normal
                // d = TO DO;
                t=cMo0.getTranslationVector();
                d=nor[0]*t[0]+nor[1]*t[1]+nor[2]*t[2];
                vo.setDistance(d);


                // rescale this translation from d
                // TO DO
                M.insert(M.getTranslationVector()*d);

                compute_initial_pose = false;
            }


            // update cMo
            // TO DO
            if(relative_to_initial)
                cMo=M*cMo0;
            else
                cMo=M*cMo;

        }
        skipped = (skipped + 1) % skip;

        // AR or frame display
        ar.display(im, cMo);

        // refresh display
        cv::waitKey(100);

    }



}
