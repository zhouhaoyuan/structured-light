#ifndef YUAN_PATTERN_H
#define YUAN_PATTERN_H

#include <common/returncode.hpp>
#include <common/debug.hpp>
#include <common/parameters.hpp>
#include <common/capture/capture.hpp>
#include <common/pattern/pattern.hpp>
#include <common/disparity_map.hpp>

#include <structured_light/structured_light.hpp>
#include <structured_light/gray_code/gray_code.hpp>
#include <structured_light/three_phase/three_phase.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#define YUAN_PATTERN_FREQUENCY_MISSING                   "YUAN_PATTERN_FREQUENCY_MISSING"
#define YUAN_PATTERN_SEQUENCE_COUNT_MISSING                   "YUAN_PATTERN_SEQUENCE_COUNT_MISSING"
#define YUAN_PATTERN_PIXELS_PER_PERIOD_MISSING           "YUAN_PATTERN_PIXELS_PER_PERIOD_MISSING"
#define YUAN_PATTERN_PIXELS_PER_PERIOD_NOT_DIVISIBLE_BY_EIGHT           "YUAN_PATTERN_PIXELS_PER_PERIOD_NOT_DIVISIBLE_BY_EIGHT"
#define YUAN_PATTERN_BITDEPTH_MISSING                    "YUAN_PATTERN_BITDEPTH_MISSING"
#define YUAN_PATTERN_BITDEPTH_TOO_SMALL                  "YUAN_PATTERN_BITDEPTH_TOO_SMALL"

namespace dlp{
/**
     * @brief The yuan_pattern class used to generate and decode patterns
     */
class yuan_pattern : public dlp::StructuredLight
{
public:
    yuan_pattern();
    ~yuan_pattern();

    class Parameters{
    public:
        //自定义变量
        DLP_NEW_PARAMETERS_ENTRY(Frequency,         "YUAN_PATTERN_PARAMETERS_FREQUENCY", double, 2.0);
        DLP_NEW_PARAMETERS_ENTRY(SequenceCount,     "YUAN_PATTERN_PARAMETERS_SEQUENCE_COUNT", unsigned int,    0);
        DLP_NEW_PARAMETERS_ENTRY(PixelsPerPeriod,   "YUAN_PATTERN_PARAMETERS_PIXELS_PER_PERIOD", unsigned int, 8);
        DLP_NEW_PARAMETERS_ENTRY(Bitdepth,          "YUAN_PATTERN_PARAMETERS_BITDEPTH",  dlp::Pattern::Bitdepth, dlp::Pattern::Bitdepth::MONO_8BPP);
        DLP_NEW_PARAMETERS_ENTRY(RepeatPhases,      "YUAN_PATTERN_PARAMETERS_REPEAT_PHASES", unsigned int, 1);

    };

    ReturnCode Setup(const dlp::Parameters &settings);
    ReturnCode GetSetup( dlp::Parameters *settings) const;
    ReturnCode GeneratePatternSequence(dlp::Pattern::Sequence *pattern_sequence);
    ReturnCode DecodeCaptureSequence(Capture::Sequence *capture_sequence,dlp::DisparityMap *disparity_map);

private:
    void Patterns_TransformToDlp(std::vector<cv::Mat> &matlist, dlp::Pattern::Sequence *pattern_sequence);

    bool reconstruction_parameters_set();

public:
    //coding
    void generate_sinusoidal_fringe(std::vector<cv::Mat> &image,dlp::Pattern::Orientation orientation,
                                    Parameters::PixelsPerPeriod _PixelsPerPeriod,const int stepNum,
                                    double phaseshift = 0);

    void generate_threeFrequencyHeterodyning(std::vector<cv::Mat> &image,const int stepNum,
                                             const int P1 = 40,const int P2 = 46, const int P3 = 350 );

    //彩色双频双三步
    void generate_color_dual_three_step_sin_pattern(std::vector<cv::Mat> &colorPattern,
                                                    int pixelperperiod_1,int pixelperperiod_2,
                                                    int stepNum, double phaseShift,
                                                    dlp::Pattern::Orientation orientation);

    //decoding
    void decode_phaseShifting_LookUpTable(cv::Mat &mask, std::vector<cv::Mat> &srcImageSequence, cv::Mat &dstImage,const int stepNum);
    void decode_multiStep_phaseShifting(cv::Mat &mask, std::vector<cv::Mat> &srcImageSequence, cv::Mat &dstImage,const int stepNum);
    void decode_dualStepPhaseShift(cv::Mat &mask, std::vector<cv::Mat> &srcImageSequence, cv::Mat &dstImage,const int stepNum);

    void decode_multiFrequencyHeterodyning(std::vector<cv::Mat> &image, const cv::Mat &maskImage,cv::Mat &unwrap);
    void decode_multiWavelength(std::vector<cv::Mat> &image, const cv::Mat &maskImage,cv::Mat &unwrap,dlp::Pattern::Orientation orientation);
    void decode_dualFrequencyHeterodyning(std::vector<cv::Mat> &image,const int T1,const int T2, const cv::Mat &maskImage,cv::Mat &unwrap);

    //reconstruction
     void reconstruct3D_by_singleCameraSystem(cv::Mat &unwrap, std::vector<cv::Point3f> &pCloud,cv::Mat &mask);
     void reconstruct3D_by_DlpCameraSystem(std::vector<cv::Point2f> leftPoints, std::vector<cv::Point2f> rightPoints, std::vector<cv::Point3f> &pCloud);
    void createCorrectPoints(const cv::Mat &unwrapX,const cv::Mat &unwrapY,int pixels,std::vector<cv::Point2f> &l_points, std::vector<cv::Point2f> &r_points,const cv::Mat &mask);
    bool AXbSolve(cv::Mat &A, cv::Mat &b, double* x, int n);


     void savePointCloud(std::vector<cv::Point3f> pCloud,QString path);
    //设置
    void set_multiFrequency(const int p1, const int p2, const int p3);
    void set_ImageSize(const int imageHeight,const int imageWidth);

private:
    Parameters::Frequency        frequency_;
    Parameters::SequenceCount    sequence_count_;
    Parameters::PixelsPerPeriod  pixels_per_period_;
    Parameters::Bitdepth         bitdepth_;
    Parameters::RepeatPhases     repeat_phases_;

    float phase_counts_;
    float maximum_value_;
    unsigned int resolution_;

    //Look-up-table
    double** atan2Table_threeStep;
    double** atan2Table_fourStep;

    int frequency_P1;
    int frequency_P2;
    int frequency_P3;

    int ImageHeight = 800;
    int ImageWidth = 1280;

    //CalibrateResult
    cv::Mat aMat;
    cv::Mat camera_intrinsic;
    cv::Mat camera_distCoeffs;

    cv::Mat E_1,E_2;
    cv::Mat M_1,M_2;

    cv::Mat _R,_T;
    cv::Mat left_intrinsic,right_intrinsic;
    cv::Mat left_distCoeffs,right_distCoeffs;

    bool singleReconstructionFlag;
};

inline bool yuan_pattern::reconstruction_parameters_set()
{
//    cv::FileStorage fs;
//    fs.open("../ScanData/SingleCalibResult.yml",cv::FileStorage::READ);
//    if(!fs.isOpened())
//    {
//        std::cout<<"Error: the SingleCalibResult yml file can not be opened!......"<<std::endl;
//    }else{
//        std::cout<<"The SingleCalibResult yml file opened successfully!......"<<std::endl;
//        fs["cameraIntrinsic"]>>camera_intrinsic;
//        fs["cameraDistCoeffs"]>>camera_distCoeffs;
//        fs["aMatrix"]>>aMat;
//        fs.release();
//        singleReconstructionFlag = true;
//    }

    cv::FileStorage fs1;
    fs1.open("../ScanData/DoubleCalibResult.yml",cv::FileStorage::READ);
    if(!fs1.isOpened())
    {
        std::cout<<"Error: the DoubleCalibResult yml file can not be opened!......"<<std::endl;
        return false;
    }else{
        std::cout<<"The DoubleCalibResult yml file opened successfully!......"<<std::endl;
        fs1["R"]>>_R;
        fs1["T"]>>_T;
        fs1["cameraM"]>>left_intrinsic;
        fs1["proM"]>>right_intrinsic;
        fs1["cameraKc"]>>left_distCoeffs;
        fs1["proKc"]>>right_distCoeffs;
        fs1.release();
        singleReconstructionFlag = false;
        return true;
    }

//    if( !fs1.isOpened()){

//        return false;
//    }else{
//        return true;
//    }

}

}

#endif // YUAN_PATTERN_H
