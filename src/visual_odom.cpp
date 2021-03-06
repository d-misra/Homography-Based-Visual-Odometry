#include <ecn_visualodom/visual_odom.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using std::vector;
using std::cout;
using std::endl;


VisualOdom::VisualOdom(const vpCameraParameters cam, bool _relative_to_initial) :matcher(cv::NORM_HAMMING)
{
    // init calibration matrices from camera parameters
    Kcv = (cv::Mat1d(3, 3) <<
           cam.get_px(), 0, cam.get_u0(),
           0, cam.get_py(), cam.get_v0(),
           0, 0, 1);
    K = cam.get_K();
    Ki = cam.get_K_inverse();

    // no image yet
    first_time = true;
    relative_to_initial = _relative_to_initial;

    // matching
    akaze = cv::AKAZE::create();

    // default guesses
    n_guess.resize(3);
    n_guess[2] = 1;
    d_guess = 1;
}



// process a new image and extracts relative transform
bool VisualOdom::process(cv::Mat &im2, vpHomogeneousMatrix &_M)
{
    // to gray level
    // TO DO
    cvtColor( im2, img, CV_RGB2GRAY );

    if(first_time)
    {
        // just detect and store point features  into kp1, des1 with akaze
        // TO DO
        akaze->detectAndCompute(img, cv::noArray(),kp1, des1);

        first_time = false;

        // copy this image
        im2.copyTo(im1);
        return false;
    }
    else
    {
        // detect point features into kp2, des2
        // TO DO
        akaze->detectAndCompute(img, cv::noArray(), kp2, des2);

        // match with stored features
        // TO DO
        //matcher=BFMatcher(NORM_HAMMING, crossCheck=True);
        matcher.match(des1,des2,matches);

        // build vectors of matched points
        std::vector<cv::Point2f> matched1, matched2;
        for(auto &m: matches)
        {
            matched1.push_back(kp1[m.queryIdx].pt);
            matched2.push_back(kp2[m.trainIdx].pt);
        }

        // use RANSAC to compute homography and store it in Hp
        // Hp = TO DO

        Hp = cv::findHomography(matched1, matched2, CV_RANSAC, 4, mask);

        // show correct matches
        cv::drawMatches(im1, kp1, im2, kp2, matches, imatches, cv::Scalar::all(-1), cv::Scalar::all(-1), mask);
        cv::imshow("Matches", imatches);

        // keep only inliers
        int mask_count = 0;
        for(unsigned i = 0; i < matches.size(); i++)
        {
            if(mask.at<int>(i))
            {
                // write this index in the next valid element
                std::iter_swap(matched1.begin()+i, matched1.begin()+mask_count);
                std::iter_swap(matched2.begin()+i, matched2.begin()+mask_count);
                mask_count++;
            }
        }
        // resize vectors to mask size
        matched1.resize(mask_count);
        matched2.resize(mask_count);

        // decompose homography -> n solutions in (R,t,nor)
        int n = 0;
        // TO DO
        n= cv::decomposeHomographyMat(Hp, Kcv, R, t, nor);


        cout << " Found " << n << " solutions" << endl;

        // build corresponding homography candidates
        H.resize(n);
        for(unsigned int i=0;i<n;++i)
            H[i].buildFrom(R[i], t[i], nor[i]);

        // prune some homography candidates based on point being at negative Z

        // build normalized coordinates of points in camera 1
        vpMatrix X1 = cvPointToNormalized(matched1); // dim(X1) = (3, nb_pts)

        for(unsigned int j=0;j<matched1.size();++j) //on points
        {
            for(unsigned int i=0;i<H.size();++i) //on solutions , checks if the jth column of X1 is in front of cqmerq
            {
                // compute Z from X.nx + Y.ny + Z.nz = d
                // equivalent to x.nx + y.ny + nz = d/Z
                // hence sign(Z) = x.nx + y.ny + nz (positive d)
                // if Z is negative then this transform is not possible
                //Z=X1[0][j]*nor[i][0]+X1[1][j]*nor[i][1]+nor[i][2];
                //if( TO DO )
                //if((X1[0][j]*nor[i].at<double>(0,0)+X1[1][j]*nor[i].at<double>(1,0)+nor[i].at<double>(3,0))<0)

                if(X1[0][j]*H[i].n[0]+X1[1][j]*H[i].n[1]+H[i].n[2]<0)
                {
                    cout << "discarded solution " << i <<", negative depth" << endl;
                    H.erase(H.begin()+i);
                    break;
                }
            }
        }

        // if all pruned, wrong behavior, exit
        if(H.size() == 0)
            return false;

        // assume best solution is H[0]
        int idx = 0;

        // if 2 solutions left, check against normal guess        
        if(H.size() == 2)
        {
            // compare normals H[0].n and H[1].n to current normal estimation n_guess
            // change idx to 1 if needed
            double dp0=n_guess[0]*H[0].n[0]+n_guess[1]*H[0].n[1]+n_guess[2]*H[0].n[2]; //the dot product betzeen normal of the 0th solution and initial nguess
            double dp1=n_guess[0]*H[1].n[0]+n_guess[1]*H[1].n[1]+n_guess[2]*H[1].n[2]; // same for the 1st solution

            if(fabs(fabs(dp0)-1)>fabs(fabs(dp1)-1)) //check which normal is closer and discard the other solution
            {
                idx=1;
            }

        }
        cout << "Kept solution " << idx << endl;

        // rescale translation from scale guess
        // TO DO
        //d_guess=n_guess[0]*H[idx].t[0]+n_guess[1]*H[idx].t[1]+n_guess[2]*H[idx].t[2];
        H[idx].t=H[idx].t*d_guess;

        // build corresponding relative transformation        
        _M.buildFrom(H[idx].t, H[idx].R);


        if(relative_to_initial)
        {
            // do not update descriptors, just refine normal
            // actual normal
            n_guess = H[idx].n;
        }
        else
        {
            // update reference image, keypoints and descriptors
            im2.copyTo(im1);
            kp1 = kp2;
            des1 = des2;

            // update estimation of normal in new frame
            n_guess =  H[idx].R*H[idx].n;

            // update plane distance in new frame
            // compute this value for all points and take the mean value
            vpColVector X2;
            vpRowVector d(X1.getCols());
            double Z1;
            double denom;
            for(unsigned int i=0;i<X1.getCols();++i)
            {
                // Z1 from current distance guess
                // TO DO
                denom=X1.getCol(i).t()*n_guess;
                Z1=d_guess/(denom);

                // Coordinates of X in frame of camera 2
                // TO DO
                X2=X1.getCol(i)*Z1;

                // corresponding distance d in camera 2
                // TO DO
                d[i]=X2.t()*n_guess;
            }
            // take the mean
            d_guess = vpRowVector::mean(d);
        }
    }
    return true;

}
