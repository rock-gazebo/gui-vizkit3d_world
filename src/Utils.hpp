#ifndef GUI_VIZKIT3D_WORLD_SRC_UTILS_HPP_
#define GUI_VIZKIT3D_WORLD_SRC_UTILS_HPP_

#include <stdio.h>
#include <stdlib.h>
#include <QtGui/QImage>
#include <base/samples/Frame.hpp>
#include <boost/thread/thread.hpp>


/**
 * Return a std::string with environment variable value
 *
 * @param varname: the name of environment variable
 * @return std::string with environmentn variable value
 */
inline std::string getEnv(std::string varname)
{
    char *value = getenv(varname.c_str());
    return (value == NULL) ? "" : std::string(value);
}

/**
 * Convert QImage format to frame_mode_t
 *
 * @param format: the QImage format
 * @return base::samples::frame::frame_mode_t
 */
inline base::samples::frame::frame_mode_t toFrameMode(QImage::Format format) {

    /**
     * the QImage formats supported in this function
     * thes formats have your correspondent frame_mode_t format
     */
    static const QImage::Format supported_formats[] = {
            QImage::Format_Indexed8,
            QImage::Format_RGB32,
            QImage::Format_ARGB32,
            QImage::Format_RGB888,
    };

    /**
     * the frame_mode_t supported in this function
     */
    static const base::samples::frame::frame_mode_t supported_modes[] = {
            base::samples::frame::MODE_GRAYSCALE,
            base::samples::frame::MODE_RGB32,
            base::samples::frame::MODE_RGB32,
            base::samples::frame::MODE_BGR,
            base::samples::frame::MODE_UNDEFINED
    };

    int totalSupportedFormats = sizeof(supported_modes) / sizeof(base::samples::frame::frame_mode_t);

    int cnt = -1;
    while ((supported_formats[++cnt] != format) && (cnt < (totalSupportedFormats-1)));

    /**
     * If the format is not supported, then return value is -1
     */
    return supported_modes[cnt];
}

/**
 * Copy image pixels from QImage to base::samples::frame::Frame
 *
 * @param src: the source image
 * @param dst: the destionation image
 * @param flipImage: if is true, flip image in vertical direction
 */
inline void cpyQImageToFrame(const QImage& src, base::samples::frame::Frame& dst, bool flipImage = false)
{
    /**
     * The number of bytes of QImage rows must be multiple of 4.
     * QImage width * number of channels is always multiple of 4.
     */
    int bytesPerPixel = src.bitPlaneCount() >> 3;
    int rowCount = src.width() * bytesPerPixel;

    /**
     * if rowCount is equal to src.bytesPerLine and flipImage is false, just copy the src.bits to dst frame
     */
    if (rowCount == src.bytesPerLine() && !flipImage) {
        dst.setImage(src.bits(), src.byteCount());
    }
    else {
        //resize image buffer
        dst.image.resize(src.height() * rowCount);
        //get pointer to Frame image buffer
        uint8_t *dstbits = static_cast<uchar *>(&dst.image[0]);
        //get pointer to QImage image buffer
        const uint8_t *srcbits = reinterpret_cast<const uint8_t*>(src.bits());

        //copy each line from source image to frame image
        for (int y = 0; y < src.height(); y++){
            int offset = (flipImage) ? ((src.height() - y - 1) * rowCount) : y * rowCount;
            memcpy(dstbits + offset, srcbits + y * src.bytesPerLine(), rowCount);
        }
    }

}

/**
 * Convert QImage to base::samples::frame::Frame
 *
 * @param src: QImage with source pixels
 * @param dst: base::samples::frame::Frame which receives pixels
 * @param flipImage: if is true, then flip image in vertical direction
 */
inline void cvtQImageToFrame(const QImage& src, base::samples::frame::Frame& dst, bool flipImage = false) {

    base::samples::frame::frame_mode_t mode = toFrameMode(src.format());

    if (mode == base::samples::frame::MODE_UNDEFINED){
        throw std::runtime_error("Unable convert QImage to base::samples::frame::Frame (unsupported image mode).");
    }

    switch (mode)
    {
        case base::samples::frame::MODE_RGB32:
        {
            /**
             * In image processing it is not necessary the alpha channel
             * If image has 32 bits per pixels, then its is converted to 24 bits, excluding the alpha channel or extra information
             */
            QImage rgb24 = src.convertToFormat(QImage::Format_RGB888);
            dst.init(src.width(), src.height(), 8, base::samples::frame::MODE_BGR, -1);
            rgb24 = rgb24.rgbSwapped();
            cpyQImageToFrame(rgb24, dst, flipImage);
        }
        break;
        default:
        {
            dst.init(src.width(), src.height(), 8, mode, -1);
            cpyQImageToFrame(src, dst, flipImage);
        }
        break;
    }
}

/**
 * Change the thread priority
 * This functions is used to change the thread priority to max
 * Is used in this program to set Qt event loop thread to max priority
 */
inline void setthread_priority_max(boost::thread& thread){
    sched_param param;
    int policy = SCHED_BATCH;

    param.sched_priority = sched_get_priority_max(policy);

    if (pthread_setschedparam(thread.native_handle(), policy, &param) != 0)
    {
       throw std::runtime_error("Unable to change thread priority.");
    }
}


#endif /* GUI_VIZKIT3D_WORLD_SRC_UTILS_HPP_ */
