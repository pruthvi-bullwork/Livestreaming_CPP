// Updated src/server.cpp with optimized settings

#include <gst/gst.h>

// Other includes...

int main(int argc, char *argv[]) {
    // Initialization code...

    // x264 settings
    x264_param_t param;
    x264_param_default(&param);
    param.i_keyint_max = 48; // Set keyframe interval
    param.b_bframe = 0; // No B-frames
    param.b_intra_refresh = 1; // Intra-refresh enabled
    
    // ... Additional initialization

    // Direct BGR to GstBuffer mapping (previously used std::vector memcpy)
    GstBuffer *buffer;
    // Assuming buffer is already created
    // ... Direct mapping code here

    // Overlay drawing optimization
    // Reduced alpha blending for overlays
    // ... Overlay drawing code here

    // Lower bitrate
    int bitrate = 1000; // kbps
    // Apply bitrate settings...

    // Main loop...

    return 0;
}