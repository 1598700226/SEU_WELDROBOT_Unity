using System.Collections;
using System.Collections.Generic;


public class BitmapInterpolation
{
    public static float bilinear(ushort[] data, int width, int height, float x, float y)
    {
        int row = (int)y;
        int col = (int)x;

        float rr = y - row;
        float cc = x - col;

        if (row + 1 >= height && col + 1 >= width)
        {
            return data[row * width + col];
        }
        if (row + 1 >= height)
        {
            return (1 - cc) * data[row * width + col] +
                    cc * data[row * width + col + 1];
        }
        if (col + 1 >= width)
        {
            return (1 - rr) * data[row * width + col] +
                   rr * data[(row + 1) * width + col];
        }

        return (1 - rr) * (1 - cc) * data[row * width + col] +
               (1 - rr) * cc * data[row * width + col + 1] +
               rr * (1 - cc) * data[(row + 1) * width + col] +
               rr * cc * data[(row + 1) * width + col + 1];
    }
}
