#include "yuan_pattern.h"

#include <QTime>
#include <QFile>
#include <QDebug>
#include <QTextStream>


#include <limits>
#include <math.h>

dlp::yuan_pattern::yuan_pattern()
{
    //读参
    reconstruction_parameters_set();

    if(!singleReconstructionFlag){

        E_1= cv::Mat::eye(3,4,CV_64FC1);
        E_2= cv::Mat::zeros(3,4,CV_64FC1);

        M_1= cv::Mat::zeros(3,4,CV_64FC1);
        M_2= cv::Mat::zeros(3,4,CV_64FC1);

//        _R.copyTo(E_2.colRange(0,3));
        for(int i=0;i< _R.rows;i++)
        {
            for(int j= 0;j<_R.cols;j++)
            {
                E_2.at<double>(i,j)= _R.at<double>(i,j);
            }
        }

        for(int i= 0;i<3;i++)
        {
            E_2.at<double>(i,3)= _T.at<double>(i);
        }

        M_1= left_intrinsic *E_1;
        M_2= right_intrinsic *E_2;
    }

    frequency_P1 = 40;
    frequency_P2 = 46;
    frequency_P3 = 350;

    //look-up-table
    //_threeStep
    atan2Table_threeStep = new double*[511];
    for(unsigned int i = 0; i < 511; ++i){

        atan2Table_threeStep[i] = new double[1021];
    }
    for(int i = 0; i < 511; ++i){
        for(int j = 0; j < 1021; ++j){

            atan2Table_threeStep[i][j] = atan2((double)(std::sqrt(3)*( i - 255 )),(double)( j - 510 ))+CV_PI;
        }
    }
    std::cout<<"atan2Table_threeStep initialized successfully ... \n";

    //_fourStep
    atan2Table_fourStep = new double*[511];
    for(unsigned int i = 0; i < 511; ++i){

        atan2Table_fourStep[i] = new double[511];
    }
    for(int i = 0; i < 511; ++i){
        for(int j = 0; j < 511; ++j){

           atan2Table_fourStep[i][j] = atan2( (double)i-255, (double)j-255)+CV_PI;
        }
    }
    std::cout<<"atan2Table_fourStep initialized successfully ... \n";
}

dlp::yuan_pattern::~yuan_pattern()
{
    for(unsigned int i = 0; i < 511; ++i){
        delete [] atan2Table_threeStep[i];
    }
    delete [] atan2Table_threeStep;
    atan2Table_threeStep = NULL;

    for(unsigned int i = 0; i < 511; ++i){
        delete [] atan2Table_fourStep[i];
    }
    delete [] atan2Table_fourStep;
    atan2Table_fourStep = NULL;
}

dlp::ReturnCode dlp::yuan_pattern::Setup(const dlp::Parameters &settings)
{
    ReturnCode ret;

    if(!this->projector_set_){
        if(settings.Get(&this->pattern_rows_).hasErrors())
            return ret.AddError(STRUCTURED_LIGHT_SETTINGS_PATTERN_ROWS_MISSING);

       if((settings.Get(&this->pattern_columns_).hasErrors()))
            return ret.AddError(STRUCTURED_LIGHT_SETTINGS_PATTERN_COLUMNS_MISSING);
    }

    ret = settings.Get(&this->pattern_orientation_);
    if(ret.hasErrors())
        return ret.AddError(STRUCTURED_LIGHT_SETTINGS_PATTERN_ORIENTATION_MISSING);

    ret = settings.Get(&this->pattern_color_);
    if(ret.hasErrors())
        return ret.AddError(STRUCTURED_LIGHT_SETTINGS_PATTERN_COLOR_MISSING);

    ret = settings.Get(&this->bitdepth_);
    if(ret.hasErrors())
        return ret.AddError(YUAN_PATTERN_BITDEPTH_MISSING);

    switch(this->bitdepth_.Get()){
        case dlp::Pattern::Bitdepth::MONO_8BPP:
            this->maximum_value_ = 255;
            break;
        case dlp::Pattern::Bitdepth::MONO_7BPP:
            this->maximum_value_ = 127;
            break;
        case dlp::Pattern::Bitdepth::MONO_6BPP:
            this->maximum_value_ = 63;
            break;
        case dlp::Pattern::Bitdepth::MONO_5BPP:
            this->maximum_value_ = 31;
            break;
        case dlp::Pattern::Bitdepth::MONO_4BPP:
            this->maximum_value_ = 31;
            break;
        case dlp::Pattern::Bitdepth::MONO_3BPP:
            this->maximum_value_ = 31;
            break;
        case dlp::Pattern::Bitdepth::MONO_2BPP:
            this->maximum_value_ = 31;
            break;
        case dlp::Pattern::Bitdepth::MONO_1BPP:
            this->maximum_value_ = 31;
            break;
        default:
            return ret.AddError(YUAN_PATTERN_BITDEPTH_TOO_SMALL);
    }

    switch(this->pattern_orientation_.Get()){
    case dlp::Pattern::Orientation::VERTICAL:
        this->resolution_ = this->pattern_columns_.Get();
        break;
    case dlp::Pattern::Orientation::HORIZONTAL:
        this->resolution_ = this->pattern_rows_.Get();
        break;
    case dlp::Pattern::Orientation::INVALID:
    default:
        return ret.AddError(STRUCTURED_LIGHT_NOT_SETUP);
        break;
    }

    ret = settings.Get(&this->pixels_per_period_);
    if(ret.hasErrors())
        return ret.AddError(YUAN_PATTERN_PIXELS_PER_PERIOD_MISSING);

    this->frequency_.Set( (double) this->resolution_ / this->pixels_per_period_.Get());
    this->phase_counts_ = this->frequency_.Get() * 2;

    // Store the number of four phase patterns ????????
    this->sequence_count_total_ = 4;

    ret = settings.Get(&this->sequence_count_);
    if(ret.hasErrors())
        return ret.AddError(YUAN_PATTERN_SEQUENCE_COUNT_MISSING);

    if(settings.Contains(this->repeat_phases_))
        settings.Get(&this->repeat_phases_);




    // Setup has been completed
    this->is_setup_ = true;

    return ret;
}

dlp::ReturnCode dlp::yuan_pattern::GetSetup(dlp::Parameters *settings) const
{
    ReturnCode ret;

    if(!settings)
        return ret.AddError(STRUCTURED_LIGHT_NULL_POINTER_ARGUMENT);

    settings->Set(this->pattern_rows_);
    settings->Set(this->pattern_columns_);
    settings->Set(this->pattern_color_);
    settings->Set(this->pattern_orientation_);
    settings->Set(this->sequence_count_);
    settings->Set(this->bitdepth_);

    return ret;
}

dlp::ReturnCode dlp::yuan_pattern::GeneratePatternSequence(dlp::Pattern::Sequence *pattern_sequence)
{
    ReturnCode ret;

    // Check that Yuan_pattern object is setup
    if((!this->isSetup()))
        return ret.AddError(STRUCTURED_LIGHT_NOT_SETUP);

    // Check that argument is not null
    if(!pattern_sequence)
        return ret.AddError(STRUCTURED_LIGHT_NULL_POINTER_ARGUMENT);

    // Clear the pattern sequence
    pattern_sequence->Clear();

    //调用自己的条纹生成函数,输出为vector<Mat>
    std::vector<cv::Mat> matlist;
    generate_threeFrequencyHeterodyning(matlist,4);

    //转换为dlp的图案
    Patterns_TransformToDlp(matlist,pattern_sequence);


    return ret;
}

dlp::ReturnCode dlp::yuan_pattern::DecodeCaptureSequence(dlp::Capture::Sequence *capture_sequence, dlp::DisparityMap *disparity_map)
{
    ReturnCode ret;
    return ret;
}

void dlp::yuan_pattern::Patterns_TransformToDlp(std::vector<cv::Mat> &matlist, dlp::Pattern::Sequence *pattern_sequence)
{
    QTime t1;
    t1.start();

    unsigned int rows     = this->pattern_rows_.Get();
    unsigned int columns  = this->pattern_columns_.Get();
//    unsigned int rows     = 912;
//    unsigned int columns  = 1140;

    unsigned int num = matlist.size();
    for(unsigned int i= 0;i < num ;++i)//遍历转换
    {
        cv::Mat matData;
        matData = matlist[i];

        dlp::Image sine_phase_image;
        sine_phase_image.Create(columns, rows, dlp::Image::Format::MONO_UCHAR );

        for(     unsigned int yRow = 0; yRow < rows;    ++yRow){
            for( unsigned int xCol = 0; xCol < columns; ++xCol){
                sine_phase_image.Unsafe_SetPixel(    xCol, yRow, unsigned char( matData.at<uchar>(yRow,xCol)) );
            }
        }

        dlp::Pattern sine_phase_pattern;

        sine_phase_pattern.bitdepth  = this->bitdepth_.Get();
        sine_phase_pattern.color     = this->pattern_color_.Get();
        sine_phase_pattern.data_type = dlp::Pattern::DataType::IMAGE_DATA;
        sine_phase_pattern.image_data.Create(sine_phase_image);

        // Add the patterns to the return sequence
        for(unsigned int iCount = 0; iCount < this->repeat_phases_.Get();iCount++){
            pattern_sequence->Add(sine_phase_pattern);
        }
        sine_phase_image.Clear();
    }

    std::cout<<"Patterns_TransformToDlp consumed : "<<t1.elapsed()<<" ms\n";
}

void dlp::yuan_pattern::generate_sinusoidal_fringe(std::vector<cv::Mat> &image, dlp::Pattern::Orientation orientation, Parameters::PixelsPerPeriod _PixelsPerPeriod,const int stepNum,double phaseshift)
{
    QTime t1;
    t1.start();

    unsigned int rows = this->ImageHeight;
    unsigned int columns = this->ImageWidth;

    double cv2PI = 2*CV_PI;
    double freq = (double)1/_PixelsPerPeriod.Get();
    double perStepNum = (double)1/stepNum;

    switch(orientation)
    {
    case dlp::Pattern::Orientation::VERTICAL:
    {
        for(unsigned int n = 0; n < stepNum; ++n){

            double m = 0;
            switch(stepNum)
            {
            case 3:
                m = n+0.6;
                break;
            case 4:
                m = n-2.0;
                break;
            default:
                m = n-2.5;
                break;
            }

            cv::Mat sinImage(rows,columns,CV_8UC1);
            uchar* Ptr = sinImage.ptr<uchar>(0);
            for(unsigned int j = 0; j < columns; ++j){

                Ptr[j] = 128 + 127*cos( cv2PI*j*freq + (m)*cv2PI*perStepNum + phaseshift);
            }
            for(unsigned int i = 1; i < rows; ++i){

                sinImage.row(0).copyTo(sinImage.row(i));
            }

            image.push_back(sinImage);
        }

    }
        break;
    case dlp::Pattern::Orientation::HORIZONTAL:
    {
        for(unsigned int n = 0; n < stepNum; ++n){

            double m = 0;
            switch(stepNum)
            {
            case 3:
                m = n+0.6;
                break;
            case 4:
                m = n-2.0;
                break;
            default:
                m = n-2.5;
                break;
            }

            cv::Mat sinImage(rows,columns,CV_8UC1);

            for(unsigned int i = 0; i < rows; ++i){

                uchar* Ptr = sinImage.ptr<uchar>(i);
                Ptr[0] = 128 + 127*cos( cv2PI*i*freq + (m)*cv2PI*perStepNum + phaseshift);
            }
            for(unsigned int j = 1; j < columns; ++j){

                sinImage.col(0).copyTo( sinImage.col(j) );
            }
            image.push_back(sinImage);
        }

    }
        break;

    }

    std::cout<<"generate_sinusoidal_fringe consumed: " << t1.elapsed() << " ms"<<std::endl;
}

void dlp::yuan_pattern::generate_threeFrequencyHeterodyning(std::vector<cv::Mat> &image,const int stepNum,const int P1,const int P2, const int P3 )
{
    generate_sinusoidal_fringe(image,dlp::Pattern::Orientation::VERTICAL,P1,stepNum);
    generate_sinusoidal_fringe(image,dlp::Pattern::Orientation::VERTICAL,P2,stepNum);
    generate_sinusoidal_fringe(image,dlp::Pattern::Orientation::VERTICAL,P3,stepNum);

//    generate_sinusoidal_fringe(image,dlp::Pattern::Orientation::HORIZONTAL,P1,stepNum);
//    generate_sinusoidal_fringe(image,dlp::Pattern::Orientation::HORIZONTAL,P2,stepNum);
//    generate_sinusoidal_fringe(image,dlp::Pattern::Orientation::HORIZONTAL,P3,stepNum);

}

void dlp::yuan_pattern::generate_color_dual_three_step_sin_pattern(std::vector<cv::Mat> &colorPattern, int pixelperperiod_1, int pixelperperiod_2, int stepNum, double phaseShift,dlp::Pattern::Orientation orientation)
{
    QTime t1;
    t1.start();

    std::vector<cv::Mat> f1_channel,f2_channel;//先蓝后红

    generate_sinusoidal_fringe(f1_channel, orientation, pixelperperiod_1, stepNum, 0);
    generate_sinusoidal_fringe(f1_channel, orientation, pixelperperiod_1, stepNum, phaseShift);
    generate_sinusoidal_fringe(f2_channel, orientation, pixelperperiod_2, stepNum, 0);
    generate_sinusoidal_fringe(f2_channel, orientation, pixelperperiod_2, stepNum, phaseShift);

    cv::Mat zeroMat = cv::Mat(this->ImageHeight,this->ImageWidth,CV_8UC1, cv::Scalar(0));

    std::vector<cv::Mat> channels;
    for(size_t i = 0; i < stepNum ; ++i)
    {
        channels.clear();
        channels.push_back(f1_channel[i]);
        channels.push_back(zeroMat);
        channels.push_back(f1_channel[i+stepNum]);

        cv::Mat colorMat;
        cv::merge(channels, colorMat);
        colorPattern.push_back(colorMat);
    }

    for(size_t i = 0; i < stepNum ; ++i)
    {
        channels.clear();
        channels.push_back(f2_channel[i]);
        channels.push_back(zeroMat);
        channels.push_back(f2_channel[i+stepNum]);

        cv::Mat colorMat;
        cv::merge(channels, colorMat);
        colorPattern.push_back(colorMat);
    }
    std::cout<<"generate_color_dual_three_step_sin_pattern consumed : "<<t1.elapsed()<<" ms\n";
}

void dlp::yuan_pattern::decode_phaseShifting_LookUpTable(cv::Mat &mask, std::vector<cv::Mat> &srcImageSequence, cv::Mat &dstImage,const int stepNum)
{
    unsigned int ImageHeight =srcImageSequence[0].rows;
    unsigned int ImageWidth  =srcImageSequence[0].cols;

    QTime t1;
    t1.start();

    switch (stepNum) {
    case 3:
    {
        if(srcImageSequence.size()!= 3)
        {
            std::cout<<"Error: the number of sinusoidal pattern is incorrect ... "<<std::endl;
            return;
        }

        cv::Mat img1 = srcImageSequence[0];
        cv::Mat img2 = srcImageSequence[1];
        cv::Mat img3 = srcImageSequence[2];

        if(img1.isContinuous() && img2.isContinuous() && img3.isContinuous()){
            ImageWidth = ImageHeight*ImageWidth;
            ImageHeight = 1;
        }

        for(unsigned int i = 0; i < ImageHeight; ++i)
        {
            uchar* ptr1= img1.ptr<uchar>(i);
            uchar* ptr2= img2.ptr<uchar>(i);
            uchar* ptr3= img3.ptr<uchar>(i);
            uchar* mask_ptr= mask.ptr<uchar>(i);
            double* optr= dstImage.ptr<double>(i);

            for(unsigned int j=0; j < ImageWidth; ++j)
            {
                if(mask_ptr[j]!=0)
                {
                    optr[j] = (double)atan2Table_threeStep[ ptr1[j] - ptr3[j] +255 ][ 2*ptr2[j] - ptr1[j] - ptr3[j] +510 ];
                    //optr[j] = atan2((double)(std::sqrt(3)*( ptr1[j] - ptr3[j] )),(double)( 2*ptr2[j] - ptr1[j] - ptr3[j] ))+CV_PI;

//                    double phase = atan2((double)(std::sqrt(3)*( ptr1[j] - ptr3[j] )),(double)( 2*ptr2[j] - ptr1[j] - ptr3[j] ))+CV_PI;
//                    if(i==0&&j<20)std::cout<<optr[j]<<" , "<<phase<<std::endl;
                }
            }
        }
        //cv::normalize(sinusoidal_phase,sinusoidal_phase,1.0,0.0,cv::NORM_MINMAX);
    }
        break;
    case 4:
    {
        if(srcImageSequence.size() != 4)
        {
            std::cout<<"the number of sinusoidal pattern is incorrect ... "<<std::endl;
            return;
        }

        cv::Mat img1 = srcImageSequence[0];
        cv::Mat img2 = srcImageSequence[1];
        cv::Mat img3 = srcImageSequence[2];
        cv::Mat img4 = srcImageSequence[3];

        if(img1.isContinuous() && img2.isContinuous() && img3.isContinuous() && img4.isContinuous()){
            ImageWidth = ImageHeight*ImageWidth;
            ImageHeight = 1;
        }

        for(unsigned int i=0;i<ImageHeight;++i)
        {

            uchar* ptr1= img1.ptr<uchar>(i);
            uchar* ptr2= img2.ptr<uchar>(i);
            uchar* ptr3= img3.ptr<uchar>(i);
            uchar* ptr4= img4.ptr<uchar>(i);
            uchar* mask_ptr= mask.ptr<uchar>(i);
            double* optr= dstImage.ptr<double>(i);

            for(unsigned int j=0;j<ImageWidth;++j)
            {
                if(mask_ptr[j]!=0)
                {
                    optr[j] = (double)atan2Table_fourStep[ ptr4[j]-ptr2[j]+255 ][ ptr1[j]-ptr3[j]+255 ];

//                    double phase = (double)atan2( ptr4[j]-ptr2[j],ptr1[j]-ptr3[j])+CV_PI;
//                    if(i==0&&j<20)std::cout<<optr[j]<<" , "<<phase<<std::endl;
                }
            }
        }
        //cv::normalize(sinusoidal_phase,sinusoidal_phase,1.0,0.0,cv::NORM_MINMAX);

    }
        break;
    default:
        break;
    }
    std::cout<<"decode_phaseShifting_LookUpTable consumed: "<<t1.elapsed()<<" ms"<<std::endl;
}

void dlp::yuan_pattern::decode_multiStep_phaseShifting(cv::Mat &mask, std::vector<cv::Mat> &srcImageSequence, cv::Mat &dstImage, const int stepNum)
{
    if(srcImageSequence.size()!=stepNum)
    {
        std::cout<<"Error: the size of srcImageSequence is not equal to stepNum ... \n";
        return;
    }
    QTime t1;
    t1.start();

    int ImageHeight = srcImageSequence[0].rows;
    int ImageWidth = srcImageSequence[0].cols;

    double numerator=0;
    double denominator=0;
    double coefficient = 2*CV_PI/stepNum;

    for(int i=0;i<ImageHeight;++i)
    {
        double* result_ptr = dstImage.ptr<double>(i);
        uchar* mask_ptr = mask.ptr<uchar>(i);

        for(int j=0;j<ImageWidth;++j)
        {
            if(mask_ptr[j]!=0)
            {
                numerator=0;
                denominator=0;
                for(unsigned int n=0;n<stepNum;++n)
                {
                    uchar *ptr=srcImageSequence[n].ptr<uchar>(i);
                    numerator += ptr[j]*sin(n*coefficient);
                    denominator += ptr[j]*cos(n*coefficient);
                }

                result_ptr[j] = -atan2(numerator,denominator)+CV_PI;
            }

        }

    }
    std::cout<<"the time of multi_step_phaseShift is "<<t1.elapsed()<<" ms\n";
}

void dlp::yuan_pattern::decode_dualStepPhaseShift(cv::Mat &mask, std::vector<cv::Mat> &srcImageSequence, cv::Mat &dstImage, const int stepNum)
{
    unsigned int ImageHeight = srcImageSequence[0].rows;
    unsigned int ImageWidth  = srcImageSequence[0].cols;
    double CV_PIperStep = CV_PI/stepNum;
    double CV2PI = CV_PI*2;

    QTime t1;
    t1.start();

    switch (stepNum) {
    case 3:
    {
        if(srcImageSequence.size()!= 6)
        {
            std::cout<<"Error: the number of sinusoidal pattern is incorrect ... "<<std::endl;
            return;
        }

        cv::Mat img1 = srcImageSequence[0];
        cv::Mat img2 = srcImageSequence[1];
        cv::Mat img3 = srcImageSequence[2];
        cv::Mat img4 = srcImageSequence[3];
        cv::Mat img5 = srcImageSequence[4];
        cv::Mat img6 = srcImageSequence[5];

        if(img1.isContinuous() && img2.isContinuous() && img3.isContinuous()){
            ImageWidth = ImageHeight*ImageWidth;
            ImageHeight = 1;
        }

        for(unsigned int i = 0; i < ImageHeight; ++i)
        {
            uchar* ptr1= img1.ptr<uchar>(i);
            uchar* ptr2= img2.ptr<uchar>(i);
            uchar* ptr3= img3.ptr<uchar>(i);
            uchar* ptr4= img4.ptr<uchar>(i);
            uchar* ptr5= img5.ptr<uchar>(i);
            uchar* ptr6= img6.ptr<uchar>(i);
            uchar* mask_ptr= mask.ptr<uchar>(i);
            double* optr= dstImage.ptr<double>(i);

            for(unsigned int j=0; j < ImageWidth; ++j)
            {
                if(mask_ptr[j]!=0)
                {
                    double phase1 = (double)atan2Table_threeStep[ ptr1[j] - ptr3[j] +255 ][ 2*ptr2[j] - ptr1[j] - ptr3[j] +510 ];
                    double phase2 = (double)atan2Table_threeStep[ ptr4[j] - ptr6[j] +255 ][ 2*ptr5[j] - ptr4[j] - ptr6[j] +510 ];

                    double phaseError = 0.0;
                    if(phase1 < phase2)
                    {
                        phaseError = phase1 - phase2 + CV_PIperStep;
                    }
                    else{

                        phaseError = phase1 - phase2 + CV_PIperStep - 2*CV_PI;
                    }
                    if(phaseError < 0.5)
                    {
                        mask_ptr[j] = 0;
                        continue;
                    }

                    optr[j] =  (phase1 <= phase2) ? (phase1 + phase2 - CV_PIperStep)*0.5 : (phase1 + phase2 - CV_PIperStep + CV2PI)*0.5 ;
                }
            }
        }
        //cv::normalize(sinusoidal_phase,sinusoidal_phase,1.0,0.0,cv::NORM_MINMAX);
    }
        break;
    case 4:
    {
        if(srcImageSequence.size()!= 8)
        {
            std::cout<<"Error: the number of sinusoidal pattern is incorrect ... "<<std::endl;
            return;
        }

        std::vector<cv::Mat> list1,list2;
        list1.insert(list1.begin(),srcImageSequence.begin(),srcImageSequence.begin()+4);
        list2.insert(list2.begin(),srcImageSequence.begin()+4,srcImageSequence.end());

        cv::Mat phase1 = cv::Mat::zeros(ImageHeight,ImageWidth,CV_64FC1);
        cv::Mat phase2 = cv::Mat::zeros(ImageHeight,ImageWidth,CV_64FC1);
        decode_phaseShifting_LookUpTable(mask,list1,phase1,4);
        decode_phaseShifting_LookUpTable(mask,list2,phase2,4);

        if(phase1.isContinuous() && phase2.isContinuous()){
            ImageWidth = ImageHeight*ImageWidth;
            ImageHeight = 1;
        }

        for(unsigned int i = 0; i < ImageHeight; ++i)
        {

            double* ptr1= phase1.ptr<double>(i);
            double* ptr2= phase2.ptr<double>(i);

            uchar* mask_ptr= mask.ptr<uchar>(i);
            double* optr= dstImage.ptr<double>(i);

            for(unsigned int j=0; j < ImageWidth; ++j)
            {
                if(mask_ptr[j]!=0)
                {

                    double phaseError = 0.0;
                    if(ptr1[j] < ptr2[j])
                    {
                        phaseError = ptr1[j] - ptr2[j] + CV_PIperStep;
                    }
                    else{

                        phaseError = ptr1[j] - ptr2[j] + CV_PIperStep - 2*CV_PI;
                    }
                    if(phaseError < 0.5)
                    {
                        mask_ptr[j] = 0;
                        continue;
                    }

                    optr[j] =  (ptr1[j] <= ptr2[j]) ? (ptr1[j] + ptr2[j] - CV_PIperStep)*0.5 : (ptr1[j] + ptr2[j] - CV_PIperStep + CV2PI)*0.5 ;
                }
            }
        }
        //cv::normalize(sinusoidal_phase,sinusoidal_phase,1.0,0.0,cv::NORM_MINMAX);

    }
        break;
    default:
        break;
    }
    std::cout<<"decode_dualStepPhaseShift consumed: "<<t1.elapsed()<<" ms"<<std::endl;
}

void dlp::yuan_pattern::decode_multiFrequencyHeterodyning(std::vector<cv::Mat> &image, const cv::Mat &maskImage,cv::Mat &unwrap)
{
    QTime t1;
    t1.start();

    if(image.empty()){
        std::cout<<"Decode Error: the image is empty ... \n";
        return ;
    }

    unsigned int rows = image[0].rows;
    unsigned int columns = image[0].cols;

    unwrap = cv::Mat::zeros(rows,columns,CV_64FC1);

    cv::Mat image_p1= image[0];
    cv::Mat image_p2= image[1];
    cv::Mat image_p3= image[2];

    double P1 = frequency_P1;
    double P2 = frequency_P2;
    double P3 = frequency_P3;

    double p12 = (double)P2*P1/(P2- P1);
    double c3 = p12/(P3-p12);
    double cv2PI = CV_PI*2;

    for(unsigned int i= 0;i< rows; ++i)
    {
        double* ptr_w1 = image_p1.ptr<double>(i);
        double* ptr_w2 = image_p2.ptr<double>(i);
        double* ptr_w3 = image_p3.ptr<double>(i);
        const uchar *ptr_mask = maskImage.ptr<uchar>(i);
        double* unwrapPtr = unwrap.ptr<double>(i);

        for(unsigned int j= 0;j< columns; ++j)
        {
           if(ptr_mask[j]!=0 )
           {
               //包裹相位1、2叠加
               //解包裹相位wrap_3，1、直接按比例换算123-3，2、计算出k值再换算，3、两者综合作误差补偿
               double delta_w1 = ptr_w1[j]/cv2PI;
               double delta_w2 = ptr_w2[j]/cv2PI;
               double delta_w3 = ptr_w3[j]/cv2PI;
               double delta_w12 = ( delta_w2 <= delta_w1 )? delta_w1 - delta_w2 : delta_w1 - delta_w2 +1 ;
               double delta_w123 = ( delta_w3 <= delta_w12 )? delta_w12 - delta_w3 : delta_w12 - delta_w3 +1 ;
               /***************************************************************************************/

               //比例展开wrap3，小跃变
               double value_unw3 = cv2PI * c3 * delta_w123;

               //取整展开wrap3,大跃变
               double value_un_k3= cv2PI * floor(c3*delta_w123)+ ptr_w3[j];

               //结合比例展开、k值展开部分进行相位补偿comp_unwrap_3
               double value_un_err3= value_un_k3 - value_unw3;

               if( CV_PI < value_un_err3)
               {
                    value_un_k3 -= cv2PI;
               }

               if( -CV_PI > value_un_err3)
               {
                    value_un_k3 += cv2PI;
               }

               double correct_unwrap3 = value_un_k3;

               //理论上的unwrap1
               double value_unw1= correct_unwrap3*P3/P1;
               //校正
               double k1 = round((value_unw1 - ptr_w1[j])/cv2PI);

               unwrapPtr[j] = cv2PI * k1 + ptr_w1[j];
              // unwrapPtr[j] = value_unw3;
          }

        }
    }

    std::cout<<"decode_threefrequency consumed "<<t1.elapsed()<<" ms\n";

}

void dlp::yuan_pattern::decode_multiWavelength(std::vector<cv::Mat> &image, const cv::Mat &maskImage, cv::Mat &unwrap,dlp::Pattern::Orientation orientation)
{
    QTime t1;
    t1.start();

    if(image.empty()){
        std::cout<<"Decode Error: the image is empty ... \n";
        return ;
    }else if(image.size()!=5){
        std::cout<<"Decode Error: the image size is not correct ... \n";
        return ;
    }

    unsigned int rows = image[0].rows;
    unsigned int columns = image[0].cols;

    double coefficient12 = 0;
    double coefficient23 = 0;
    double coefficient34 = 0;
    double coefficient45 = 0;

    coefficient12 = (double)2000/960;
    coefficient23 = (double)960/320;
    coefficient34 = (double)320/80;
    coefficient45 = (double)80/20;

    double perCV2PI = 1/(2*CV_PI);
    double CV2PI = 2*CV_PI;

    for(unsigned int i = 0; i < rows; ++i){

        double* ptr1 = image[0].ptr<double>(i);
        double* ptr2 = image[1].ptr<double>(i);
        double* ptr3 = image[2].ptr<double>(i);
        double* ptr4 = image[3].ptr<double>(i);
        double* ptr5 = image[4].ptr<double>(i);
        double* ptr = unwrap.ptr<double>(i);
        const uchar* maskptr = maskImage.ptr<uchar>(i);

        for(unsigned int j = 0; j < columns; ++j){

               if(maskptr[j]!=0){

                   double k2 = round( (coefficient12 * ptr1[j] -  ptr2[j])*perCV2PI );
                   double unwrap2 = k2*CV2PI + ptr2[j];

                   double k3 = round( (coefficient23 * unwrap2 -  ptr3[j])*perCV2PI );
                   double unwrap3 = k3*CV2PI + ptr3[j];

                   double k4 = round( (coefficient34 * unwrap3 -  ptr4[j])*perCV2PI );
                   double unwrap4 = k4*CV2PI + ptr4[j];

                   double k5 = round( (coefficient45 * unwrap4 -  ptr5[j])*perCV2PI );
                   double unwrap5 = k5*CV2PI + ptr5[j];

                   ptr[j] = unwrap5;
               }
        }

    }
    std::cout<<"decode_multiWavelength consumed "<<t1.elapsed()<<" ms\n";
}

void dlp::yuan_pattern::decode_dualFrequencyHeterodyning(std::vector<cv::Mat> &image,const int T1,const int T2, const cv::Mat &maskImage, cv::Mat &unwrap)
{
    QTime t1;
    t1.start();

    if(image.empty())
    {
        std::cout<<"Error: decode_dualFrequencyHeterodyning, the image is empty\n";
        return;
    }
    int imageHeight = image[0].rows;
    int imageWidth  = image[0].cols;

    //T1是高频单周期像素数，T2是低频
    double compositeT = (double)T2/(T2 - T1);
    double compositePhase = 0.0;
    double CV2PI = CV_PI * 2;

    unsigned int i,j;
    for( i = 0; i < imageHeight; ++i)
    {
       const uchar* maskPtr = maskImage.ptr<uchar>(i);
       double* phase1Ptr = image[0].ptr<double>(i);
       double* phase2Ptr = image[1].ptr<double>(i);
       double* unwrapPtr = unwrap.ptr<double>(i);

       for( j = 0; j < imageWidth; ++j)
       {
           if(maskPtr[j]!=0)
           {
               if(phase1Ptr[j] < phase2Ptr[j])
               {
                   compositePhase = phase1Ptr[j] - phase2Ptr[j] + CV2PI;
               }
               else
               {
                   compositePhase = phase1Ptr[j] - phase2Ptr[j];
               }

               unwrapPtr[j] = CV2PI * round((compositeT * compositePhase - phase1Ptr[j])/CV2PI)+phase1Ptr[j];
           }
        }
     }
     std::cout<<"decode_doublefrequencyHeterodyne consumed "<<t1.elapsed()<<" ms\n";
}

void dlp::yuan_pattern::reconstruct3D_by_singleCameraSystem(cv::Mat &unwrap, std::vector<cv::Point3f> &pCloud, cv::Mat &mask)
{
    QTime t1;
    t1.start();

    unsigned int rows = unwrap.rows;
    unsigned int cols = unwrap.cols;

    cv::Mat coefficient = cv::Mat::zeros(3,3,CV_64FC1);
    coefficient.at<double>(1,0) = camera_intrinsic.at<double>(0,0);
    coefficient.at<double>(1,1) = camera_intrinsic.at<double>(0,1);
    coefficient.at<double>(2,0) = camera_intrinsic.at<double>(1,0);
    coefficient.at<double>(2,1) = camera_intrinsic.at<double>(1,1);

    cv::Mat bMat = cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat result = cv::Mat::zeros(3,1,CV_64FC1);

    for(unsigned int i = 0; i < rows; ++i){

        double* phasePtr = unwrap.ptr<double>(i);
        uchar* maskPtr = mask.ptr<uchar>(i);

        for(unsigned int j = 0; j < cols; ++j){

            if(maskPtr[j]!=0){

                coefficient.at<double>(0,0) = phasePtr[j] * aMat.at<double>(4,0) - aMat.at<double>(0,0);//Xita*a5-a1
                coefficient.at<double>(0,1) = phasePtr[j] * aMat.at<double>(5,0) - aMat.at<double>(1,0);//Xita*a6-a2
                coefficient.at<double>(0,2) = phasePtr[j] * aMat.at<double>(6,0) - aMat.at<double>(2,0);//Xita*a7-a3

                coefficient.at<double>(1,2) = camera_intrinsic.at<double>(0,2) - j;//a7-a3
                coefficient.at<double>(2,2) = camera_intrinsic.at<double>(1,2) - i;//a7-a3

                bMat.at<double>(0,0) = aMat.at<double>(3,0) - phasePtr[j] * aMat.at<double>(7,0);

                result = coefficient.inv()*bMat;

                cv::Point3f temp;
                temp.x = result.at<double>(0,0);
                temp.y = result.at<double>(1,0);
                temp.z = result.at<double>(2,0);

                pCloud.push_back(temp);
            }

        }
    }
    std::cout<<"reconstruct3D_by_singleCameraSystem consumed : "<<t1.elapsed()<<" ms ...\n";
}

void dlp::yuan_pattern::reconstruct3D_by_DlpCameraSystem(std::vector<cv::Point2f> leftPoints, std::vector<cv::Point2f> rightPoints, std::vector<cv::Point3f> &pCloud)
{
    QTime t1;
    t1.start();

    int pointsNum= leftPoints.size();
    pCloud.clear();

    cv::Mat leftM= cv::Mat::zeros(3,3,CV_64FC1);
    cv::Mat rightM= cv::Mat::zeros(3,1,CV_64FC1);
    cv::Mat point= cv::Mat::zeros(3,1,CV_64FC1);

    cv::Point3f resultP;
    cv::Point2f cameraP;
    cv::Point2f dlpP;
   // #pragma omp parallel for
    for(int i= 0;i< pointsNum;++i)
    {
        cameraP = leftPoints[i];
        dlpP = rightPoints[i];

//        if( dlpP.x > 1920 || dlpP.x < 0 || dlpP.y > 1080 || dlpP.y < 0)
//        {
//            continue;
//        }
        leftM.at<double>(0,0)= cameraP.x*M_1.at<double>(2,0)- M_1.at<double>(0,0);
        leftM.at<double>(0,1)= cameraP.x*M_1.at<double>(2,1)- M_1.at<double>(0,1);
        leftM.at<double>(0,2)= cameraP.x*M_1.at<double>(2,2)- M_1.at<double>(0,2);
        leftM.at<double>(1,0)= cameraP.y*M_1.at<double>(2,0)- M_1.at<double>(1,0);
        leftM.at<double>(1,1)= cameraP.y*M_1.at<double>(2,1)- M_1.at<double>(1,1);
        leftM.at<double>(1,2)= cameraP.y*M_1.at<double>(2,2)- M_1.at<double>(1,2);
        leftM.at<double>(2,0)= dlpP.x*M_2.at<double>(2,0)- M_2.at<double>(0,0);
        leftM.at<double>(2,1)= dlpP.x*M_2.at<double>(2,1)- M_2.at<double>(0,1);
        leftM.at<double>(2,2)= dlpP.x*M_2.at<double>(2,2)- M_2.at<double>(0,2);
//        leftM.at<double>(3,0)= dlpP.y*M_2.at<double>(2,0)- M_2.at<double>(1,0);
//        leftM.at<double>(3,1)= dlpP.y*M_2.at<double>(2,1)- M_2.at<double>(1,1);
//        leftM.at<double>(3,2)= dlpP.y*M_2.at<double>(2,2)- M_2.at<double>(1,2);

        rightM.at<double>(0,0)= M_1.at<double>(0,3)- cameraP.x*M_1.at<double>(2,3);
        rightM.at<double>(1,0)= M_1.at<double>(1,3)- cameraP.y*M_1.at<double>(2,3);
        rightM.at<double>(2,0)= M_2.at<double>(0,3)- dlpP.x*M_2.at<double>(2,3);
//        rightM.at<double>(3,0)= M_2.at<double>(1,3)- dlpP.y*M_2.at<double>(2,3);

       // cv::solve(leftM,rightM,point,CV_SVD);

        cv::solve(leftM,rightM,point);
        resultP.x= point.at<double>(0,0);
        resultP.y= point.at<double>(1,0);
        resultP.z= point.at<double>(2,0);

//        double result[3];
//        AXbSolve(leftM,rightM,result,3);
//        resultP.x= result[0];
//        resultP.y= result[1];
//        resultP.z= result[2];

        pCloud.push_back(resultP);
   }
        std::cout<<"reconstruct3D consumed : "<<t1.elapsed()<<" ms\n";

}

void dlp::yuan_pattern::createCorrectPoints(const cv::Mat &unwrapX, const cv::Mat &unwrapY, int pixels, std::vector<cv::Point2f> &l_points, std::vector<cv::Point2f> &r_points, const cv::Mat &mask)
{
    QTime t1;
    t1.start();

    l_points.clear();
    r_points.clear();

    int nr = unwrapX.rows;
    int nc = unwrapX.cols;

    double pixelsPerTwoPi =  pixels/(2*CV_PI);

    double dlp_x_value = 0, dlp_y_value = 0;

    for(int r= 0; r < nr; ++r)
    {

        const double* x_ptr = unwrapX.ptr<double>(r);
        const double* y_ptr = unwrapY.ptr<double>(r);
        const uchar*  m_ptr = mask.ptr<uchar>(r);

        for(int c= 0;c< nc; ++c)
        {

            if(m_ptr[c] != 0)
            {

                    dlp_x_value= x_ptr[c]*pixelsPerTwoPi;
                    dlp_y_value= y_ptr[c]*pixelsPerTwoPi;

                    l_points.push_back(cv::Point2f(c,r));
                    r_points.push_back(cv::Point2f(dlp_x_value,dlp_y_value));

                   //cv::undistortPoints(l_points, l_points, left_intrinsic, left_distCoeffs, cv::noArray(), cv::noArray());
                   //cv::undistortPoints(r_points, r_points, right_intrinsic, right_distCoeffs, cv::noArray(), cv::noArray());
             }

         }

    }
    std::cout<<"createCorrectPoints consumed : "<<t1.elapsed()<<" ms\n";
}

bool dlp::yuan_pattern::AXbSolve(cv::Mat &A, cv::Mat &b, double *x, int n)
{
    double** M = new double*[n];
    const double EPSILONG = pow(10,-6);
        for (int i=0; i<n; i++)
        {
            M[i] = new double[n+1];
            for (int j=0; j<n; j++)
            {
                M[i][j] = A.at<double>(i,j);
            }
            M[i][n] = b.at<double>(i,0);
        }
        for (int k=0; k<n; k++)
        {//n个主元
            double colMax = fabs(M[k][k]);
            int maxLineIndex = k;
            for(int i=k+1; i<n; i++)
            {//寻找第k列的最大元素
                if(fabs(M[i][k]) > colMax)
                {
                    colMax = fabs(M[i][k]);
                    maxLineIndex = i;
                }
            }
            if(colMax < EPSILONG)
            {//奇异矩阵
                for (int i=0; i<n; i++)
                {
                    delete M[i];
                }
                delete M;
                return false;
            }
            double temp;
            //交换k行和maxLineIndex行
            for (int m=0; m<n+1; m++)
            {
                temp = M[k][m];
                M[k][m] = M[maxLineIndex][m];
                M[maxLineIndex][m] = temp;
            }
            //消去
            for(int i=k+1; i<n; i++)
            {
                for (int j=k+1; j<n+1; j++)
                {
                    M[i][j] = M[k][k]*M[i][j]/M[i][k] - M[k][j];
                }
            }
        }
        //回归求解
        for (int i=n-1; i>=0; i--)
        {
            x[i] = M[i][n];
            for (int j=i+1; j<n; j++)
            {
                x[i] -= M[i][j]*x[j];
            }
            x[i] /= M[i][i];
        }
        for (int i=0; i<n; i++)
        {
            delete M[i];
        }
        delete M;
        return true;

}

void dlp::yuan_pattern::savePointCloud(std::vector<cv::Point3f> pCloud, QString path)
{
    QTime t1;
    t1.start();

    QString savePoints= path;

    if(savePoints.isEmpty())
    {
        std::cout<<"Error: the pCloud saving path is empty ... \n";
        return ;
    }

    QFile p_file(savePoints);
    if(!p_file.open(QIODevice::WriteOnly)){
        qDebug()<<"Error : the saving file cannot open ... ";
        return ;
    }

    QTextStream p_out(&p_file);

    //.ply头
    p_out<<"ply";
    p_out<<"\r\n";
    p_out<<"format ascii 1.0";
    p_out<<"\r\n";
    p_out<<"comment made by scan";
    p_out<<"\r\n";

    p_out<<"element vertex "<<pCloud.size();
    p_out<<"\r\n";

    p_out<<"property float x ";
    p_out<<"\r\n";

    p_out<<"property float y ";
    p_out<<"\r\n";

    p_out<<"property float z ";
    p_out<<"\r\n";

    p_out<<"end_header";
    p_out<<"\r\n";

    for( std::vector<cv::Point3f>::const_iterator it_pcloud = pCloud.begin(); it_pcloud != pCloud.end(); ++it_pcloud)
    {
        cv::Point3f resultP= *it_pcloud;

        p_out<<resultP.x<<" "<<resultP.y<<" "<<resultP.z;
        p_out<<"\r\n";
    }

    p_file.close();
    std::cout<<"savePointCloud consumed : "<<t1.elapsed()<<" ms\n";
}

void dlp::yuan_pattern::set_multiFrequency(const int p1, const int p2, const int p3)
{
    this->frequency_P1 = p1;
    this->frequency_P2 = p2;
    this->frequency_P3 = p3;
}

void dlp::yuan_pattern::set_ImageSize(const int imageHeight, const int imageWidth)
{
    this->ImageHeight = imageHeight;
    this->ImageWidth = imageWidth;
}
