#include "image_processing.h"

int main() {
    int lateral_position = 100;
    ImageProcessing imageprocessor(true, 100);
    while (true) {
        imageprocessor.process_next_frame();
    }
}