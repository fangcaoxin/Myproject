#include "dcp_core.h"
void CorseFilter(IplImage *input, IplImage *s,IplImage *dst_img,int radius) {
	int height = input->height;
	int width = input->width;
	int widthstep = input->widthStep;
	int gwidthstep = input->widthStep;
	int nch = input->nChannels;
	if (nch != 1) {
		printf("the input image should be gray image");

	}
	int st_row, ed_row;
	int st_col, ed_col;

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			st_row = i - radius, ed_row = i + radius;
			st_col = j - radius, ed_col = j + radius;

			st_row = st_row < 0 ? 0 : st_row;
			ed_row = ed_row >= height ? (height - 1) : ed_row;
			st_col = st_col < 0 ? 0 : st_col;
			ed_col = ed_col >= width ? (width - 1) : ed_col;

			int cur = 0;
			//int min = 300;
			int size = (ed_row-st_row+1)*(ed_col-st_col+1);
			int sum = 0,count=0;
			double sigma = 0.0;
			double average = 0.0;
			uchar* patch_value = (uchar *)malloc(size * sizeof(uchar));
			for (int m = st_row; m <= ed_row; m++)
			{
				for (int n = st_col; n <= ed_col; n++)
				{
					
						cur = *(uchar *)(input->imageData + m * widthstep + n);
						patch_value[count++] = cur;
						sum += cur;
					
				}
			}
			average = sum / size;
			for (int i = 0; i < size; i++) {
				sigma += (patch_value[i] - average)*(patch_value[i] - average);
			}
			sigma = sigma / size;
			uchar pixel_value= *(uchar *)(input->imageData + i * widthstep +j);
			uchar s_value = *(uchar *)(s->imageData + i*widthstep + j);
			if ((pixel_value - average)*(pixel_value - average) > 3*sigma&pixel_value>s_value+100) {
				*(uchar *)(dst_img->imageData + i * gwidthstep + j) = 255;
			}
			else {
				*(uchar *)(dst_img->imageData + i * gwidthstep + j) = 0;
			}

			
		}
	}
}