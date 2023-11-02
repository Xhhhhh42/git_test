/**
 * Created by Yuchen Xia on 01.11.23.
 */

#ifndef MAPBAG_EDITOR_USER_INTERFACE_ADAPTIVE_MEDIEN_FILTER_H
#define MAPBAG_EDITOR_USER_INTERFACE_ADAPTIVE_MEDIEN_FILTER_H

#include <hector_math/types.h>

namespace mapbag_editor_user_interface
{
using namespace hector_math;
using BoolMap = Eigen::Array<bool, Eigen::Dynamic, Eigen::Dynamic>;

GridMap<float> adaptiveMeanFilter( const hector_math::GridMap<float>& input, int startsize, int filterSize ) {
    int width = input.cols();
    int height = input.rows();
    GridMap<float> imageGray = input;
    GridMap<float> output = GridMap<float>::Zero(height, width);
    BoolMap alreadyProcessed(height, width);
    GridMap<float> zmed_copy = GridMap<float>::Zero(height, width);

    int Smax = filterSize;
    for (int k = startsize; k <= Smax; k += 2) {
        GridMap<float> zmin = GridMap<float>::Zero(height, width);
        GridMap<float> zmax = GridMap<float>::Zero(height, width);
        GridMap<float> zmed = GridMap<float>::Zero(height, width);

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                int x1 = std::max(i - k / 2, 0);
                int x2 = std::min(i + k / 2, height - 1);
                int y1 = std::max(j - k / 2, 0);
                int y2 = std::min(j + k / 2, width - 1);

                GridMap<float> subImage = imageGray.block(x1, y1, x2 - x1 + 1, y2 - y1 + 1);

                zmin(i, j) = subImage.minCoeff();
                zmax(i, j) = subImage.maxCoeff();

                std::vector<float> values;
                for (int x = x1; x <= x2; x++) {
                    for (int y = y1; y <= y2; y++) {
                        values.push_back(imageGray(x, y));
                    }
                }

                sort(values.begin(), values.end());
                int medianIndex = values.size() / 2;
                zmed(i, j) = values[medianIndex];
            }
        }
        zmed_copy = zmed;
        BoolMap processUsingLevelB = (zmed.array() > zmin.array()) && (zmax.array() > zmed.array()) &&
                                    (alreadyProcessed.array() == false);
        
        BoolMap zB = (imageGray.array() > zmin.array()) && (zmax.array() > imageGray.array());

        BoolMap outputZxy = (processUsingLevelB.array() == true) && (zB.array() == true );
        BoolMap outputZmed = (processUsingLevelB.array() == true ) && (zB.array() == false);

        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                if (outputZxy(i, j)) { output(i, j) = imageGray(i, j); }
                else if (outputZmed(i, j)) { output(i, j) = zmed(i, j); }
                alreadyProcessed(i, j) = (alreadyProcessed(i, j) || processUsingLevelB(i, j));
            }
        }

        if (alreadyProcessed.array().all()) {
            break;
        }
    }

    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (!alreadyProcessed(i, j)) {
                output(i, j) = zmed_copy(i, j);
            }
        }
    }

    return zmed_copy;
}

} // namespace mapbag_editor_user_interface


#endif // MAPBAG_EDITOR_USER_INTERFACE_ADAPTIVE_MEDIEN_FILTER_H