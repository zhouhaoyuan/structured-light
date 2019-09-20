#ifndef BINARYCODE_H
#define BINARYCODE_H
/** @file   Binary_code.hpp
 *  @brief  Contains definitions for the DLP SDK Binary code classes
 *  @copyright  2016 Texas Instruments Incorporated - http://www.ti.com/ ALL RIGHTS RESERVED
 */

#ifndef DLP_SDK_Binary_CODE_HPP
#define DLP_SDK_Binary_CODE_HPP

#include <common/returncode.hpp>
#include <common/debug.hpp>
#include <common/parameters.hpp>
#include <common/capture/capture.hpp>
#include <common/pattern/pattern.hpp>

#include <structured_light/structured_light.hpp>

#define Binary_CODE_PIXEL_THRESHOLD_MISSING "Binary_CODE_PIXEL_THRESHOLD_MISSING"
#define Binary_CODE_REGIONS_REQUIRE_SUB_PIXELS    "Binary_CODE_REGIONS_REQUIRE_SUB_PIXELS"

/** @brief  Contains all DLP SDK classes, functions, etc. */
namespace dlp{

/** @class      BinaryCode
 *  @ingroup    StructuredLight
 *  @brief      Structured Light subclass used to generate and decode Binary coded
 *              binary patterns
 */
class BinaryCode: public dlp::StructuredLight{
public:

    class Parameters{
    public:
        DLP_NEW_PARAMETERS_ENTRY(   SequenceCount,           "Binary_CODE_PARAMETERS_SEQUENCE_COUNT", unsigned int,    0);
        DLP_NEW_PARAMETERS_ENTRY( IncludeInverted,    "Binary_CODE_PARAMETERS_PATTERN_INCLUDE_INVERTED",         bool, false);
        DLP_NEW_PARAMETERS_ENTRY(  PixelThreshold,          "Binary_CODE_PARAMETERS_PIXEL_THRESHOLD", unsigned int,    5);
        DLP_NEW_PARAMETERS_ENTRY( MeasureRegions,  "Binary_CODE_PARAMETERS_MEASURE_REGIONS", float, 0.0);
        DLP_NEW_PARAMETERS_ENTRY(   CodeType,           "Binary_CODE_PARAMETERS_CODE_TYPE", unsigned int,    0);
    };

    BinaryCode();
    ~BinaryCode();

    ReturnCode Setup(const dlp::Parameters &settings);
    ReturnCode GetSetup( dlp::Parameters *settings) const;

    ReturnCode GeneratePatternSequence(Pattern::Sequence *pattern_sequence);
    ReturnCode DecodeCaptureSequence(Capture::Sequence *capture_sequence,dlp::DisparityMap *disparity_map);





private:
    Parameters::SequenceCount   sequence_count_;
    Parameters::IncludeInverted include_inverted_;
    Parameters::PixelThreshold  pixel_threshold_;
    Parameters::MeasureRegions  measure_regions_;
    Parameters::CodeType  code_type_;

    unsigned int region_size_;
    unsigned int maximum_patterns_;
    unsigned int maximum_disparity_;
    unsigned int msb_pattern_value_;
    unsigned int resolution_;
    unsigned int offset_;



    void createBinaryImage(Pattern::Orientation orientation, int i_sequence, cv::Mat &data);

    void setBinary(cv::Mat img,cv::Point p,int size);

};
}

#endif // DLP_SDK_Binary_CODE_HPP

#endif // BINARYCODE_H
