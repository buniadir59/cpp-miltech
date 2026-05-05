#include "../include/telemetry.hpp" //FIX#1: incorrect path

#include <cstdlib>
#include <cerrno>  // errno, ERANGE
#include <cmath>   // std::isfinite
#include <limits>
#include <fstream>
#include <iostream>
#include <string>


// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

int split_line(char line[], char* fields[], int max_fields) {
    int count = 0;
    char* cursor = line;

    while (*cursor != '\0' && count < max_fields) {
        while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
            *cursor = '\0';
            ++cursor;
        }

        if (*cursor == '\0') {
            break;
        }

        fields[count] = cursor;
        ++count;

        while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' &&
               *cursor != '\r') {
            ++cursor;
        }
    }

    return count;
}

bool parse_long(const char* text, int line_count, long& res_long) {
    char* end = nullptr;
    errno = 0;
    res_long = std::strtol(text, &end, 10);

    if (errno == ERANGE || end == text  || *end != '\0') { 
        std::cerr   << "ERR: invalid integer value at line " << line_count << ".\n";
        return false;
    }

    return true;
}

bool parse_int(const char* text, int line_count, int& res_int) {
    long res_long;
    if (!parse_long(text, line_count, res_long)) {
        return false;
    } 

    if (res_long < std::numeric_limits<int>::min()
             || res_long > std::numeric_limits<int>::max()) {
        std::cerr << "ERR: integer value out of range at line " << line_count << "\n";             
        return false;
    }

    res_int = static_cast<int>(res_long);
    return true; 
}

bool parse_double(const char* text, int line_count, double& res_double) {
    char* end = nullptr;
    errno = 0;
    res_double = std::strtod(text, &end);

    if (errno == ERANGE ||end == text || *end != '\0') { //
        std::cerr   << "ERR: invalid double value at line " << line_count << ".\n";
        return false;
    }

    return true;
}

/****
 * parses frame in line, returns result (true/false) and validated frame 
 * validation includes:
 *  - number of fields must be EXPECTED_FIELD_COUNT
 *  - voltage_v > 0
 *  - temperature_c in [-40, 120] range 
 *  - gps_fix = 0 or 1
 *  - satellites >= 0
 * in case of parsing or validation error, error message is generated
 * with detailed error description and line number with erroneus input
 */ 
bool parse_frame(char line[], int line_count, Frame& frame) {
    char* fields[EXPECTED_FIELD_COUNT] = {};
    const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);
    const std::string errhead = "ERR: invalid input data in line ";

    if (field_count != EXPECTED_FIELD_COUNT) { //invalid data in line
        std::cerr   << errhead << line_count << ": expected "
                    << EXPECTED_FIELD_COUNT << " fields.\n";
        return false;
    }

    if ( 
        !parse_long(fields[0], line_count, frame.timestamp_ms )
        || !parse_int(fields[1], line_count, frame.seq )
        || !parse_double(fields[2], line_count, frame.voltage_v) 
        || !parse_double(fields[3], line_count, frame.current_a)
        || !parse_double(fields[4], line_count, frame.temperature_c)
        || !parse_int(fields[5], line_count, frame.gps_fix)
        || !parse_int(fields[6], line_count, frame.satellites) 
    ) return false;

    if (frame.voltage_v <= 0) {
        std::cerr   << errhead << line_count << ": voltage_v must be > 0.\n";
        return false;  
    }

    if (frame.satellites < 0) {
        std::cerr   << errhead << line_count << ": satellites must be >=0.\n";
        return false;  
    }

    if (!(frame.gps_fix == 0 || frame.gps_fix == 1)) {
        std::cerr   << errhead << line_count << ": gps_fix must be 0 or 1.\n";
        return false;  
    }

    if ((frame.temperature_c < -40.0 || frame.temperature_c >= 120)) {
        std::cerr   << errhead << line_count 
                    << ": temperature_c must be in [-40, 120] range.\n";
        return false;  
    }

    return true;
}

/****
 *   returns frame frequency in Hz (double)
 *   if frame frequency cannot be calculated, returns 0
 */
double compute_frame_rate_hz(const Frame frames[], int frame_count) { 
    //defensive check
    if (frame_count < 2) {
        return 0.0;
    }

    const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;
    if (elapsed_ms <= 0) {
        return 0.0;
    }

    return static_cast<double>(frame_count - 1) * 1000.0 / static_cast<double>(elapsed_ms);
}

/****
 * reads frames from text file path, not more than max_frames,
 * puts frames in array frames and validates input
 * returns number of read frames
 * generates error message if input file cannot be open, or validation fails
 * validation includes:
 *  - timestamp_ms increases every frame
 *  - seq increments every frame
 */ 
int read_frames(const char* path, Frame frames[], int max_frames) {
    std::ifstream input{path};
    if (!input) {
        std::cerr << "ERR: failed to open input file: " << path << '\n';
        return -1; //fixed return for file open error 
    }

    int frame_count = 0;
    int line_count = 0;
    char line[MAX_LINE_LENGTH];
    const std::string errhead = "ERR: invalid frame at line ";

    while (input.getline(line, MAX_LINE_LENGTH)) {
        line_count++;
        if (line[0] == '\0') { //skip empty line
            continue;
        }

        if (frame_count < max_frames) {
            if (parse_frame(line, line_count, frames[frame_count])) {
                if (frame_count > 0) {
                    if (frames[frame_count].timestamp_ms <= frames[frame_count - 1].timestamp_ms) {
                        std::cerr << errhead << line_count << ": timestamp_ms must increase\n";           
                        return -1;
                    }

                    if (frames[frame_count].seq - frames[frame_count - 1].seq != 1) {
                        std::cerr << errhead << line_count << ": seq must increment\n";           
                        return -1;
                    }
                }                
                ++frame_count;
            } else { //err invalid input data
                return -1;  
            }           
        }
    }

    return frame_count;
}

Summary summarize(const Frame frames[], int frame_count) {
    Summary summary{};
    summary.frames_total = frame_count;
    summary.frames_valid = frame_count;
    summary.voltage_min = frames[0].voltage_v;
    summary.voltage_max = frames[0].voltage_v;
    summary.low_voltage_frames = 0;

    double temperature_sum = 0.0;

    for (int i = 0; i < frame_count; ++i) {
        if (frames[i].voltage_v < summary.voltage_min) {
            summary.voltage_min = frames[i].voltage_v;
        }

        if (frames[i].voltage_v > summary.voltage_max) {
            summary.voltage_max = frames[i].voltage_v;
        }

        temperature_sum += frames[i].temperature_c;

        if (frames[i].voltage_v < 22.0) {
            ++summary.low_voltage_frames;
        }
    }

    const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
    summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
    summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
    return summary;
}

void print_summary(const Summary& summary) {
    std::cout << "frames_total " << summary.frames_total << '\n';
    std::cout << "frames_valid " << summary.frames_valid << '\n';
    std::cout << "voltage_min " << summary.voltage_min << '\n';
    std::cout << "voltage_max " << summary.voltage_max << '\n';
    std::cout << "temperature_avg " << summary.temperature_avg << '\n';
    std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
    std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
