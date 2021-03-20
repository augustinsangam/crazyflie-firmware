#include "exploration/median_filter.hpp"
#include <cstring>

void init_median_filter_i(struct MedianFilterInt *filter, uint8_t size) {
	uint8_t i;
	if (size > MAX_MEDIAN_DATASIZE_I) {
		filter->size = MAX_MEDIAN_DATASIZE_I;
	} else if ((size % 2) == 0) {
		// force filter to have odd number of entries so that
		// returned median is always an entry and not an average
		filter->size = size + 1;
	} else {
		filter->size = size;
	}
	for (i = 0; i < filter->size; i++) {
		filter->data[i] = 150;
		filter->sortData[i] = 150;
	}
	filter->dataIndex = 0;
}

int32_t get_median_filter_i(struct MedianFilterInt *filter) {
	if (filter->size % 2) {
		return filter->sortData[filter->size >> 1];
	} else {
		// this should not be used if init_median_filter was used
		return (filter->sortData[filter->size / 2] +
		        filter->sortData[filter->size / 2 - 1]) /
		       2;
	}
}

int32_t update_median_filter_i(struct MedianFilterInt *filter,
                               int32_t new_data) {
	int temp, i, j; // used to sort array

	// Insert new data into raw data array round robin style
	filter->data[filter->dataIndex] = new_data;
	filter->dataIndex = (filter->dataIndex + 1) % filter->size;

	// Copy raw data to sort data array
	memcpy(filter->sortData, filter->data, sizeof(int32_t) * filter->size);

	// Insertion Sort
	for (i = 1; i < filter->size; i++) {
		temp = filter->sortData[i];
		j = i - 1;
		while (j >= 0 && temp < filter->sortData[j]) {
			filter->sortData[j + 1] = filter->sortData[j];
			j = j - 1;
		}
		filter->sortData[j + 1] = temp;
	}
	// return data value in middle of sorted array
	return get_median_filter_i(filter);
}

void init_median_filter_f(struct MedianFilterFloat *filter, uint8_t size) {
	uint8_t i;
	if (size > MAX_MEDIAN_DATASIZE_F) {
		filter->size = MAX_MEDIAN_DATASIZE_F;
	} else if ((size % 2) == 0) {
		filter->size = size + 1;
	} else {
		filter->size = size;
	}
	for (i = 0; i < filter->size; i++) {
		filter->data[i] = 0.f;
		filter->sortData[i] = 0.f;
	}
	filter->dataIndex = 0;
}

float get_median_filter_f(struct MedianFilterFloat *filter) {
	if (filter->size % 2) {
		return filter->sortData[filter->size >> 1];
	} else {
		// this should not be used if init_median_filter was used
		return (filter->sortData[filter->size / 2] +
		        filter->sortData[filter->size / 2 - 1]) /
		       2;
	}
}

float update_median_filter_f(struct MedianFilterFloat *filter, float new_data) {
	float temp;
	int i, j; // used to sort array

	// Insert new data into raw data array round robin style
	filter->data[filter->dataIndex] = new_data;
	filter->dataIndex = (filter->dataIndex + 1) % filter->size;

	// Copy raw data to sort data array
	memcpy(filter->sortData, filter->data, sizeof(float) * filter->size);

	// Insertion Sort
	for (i = 1; i < filter->size; i++) {
		temp = filter->sortData[i];
		j = i - 1;
		while (j >= 0 && temp < filter->sortData[j]) {
			filter->sortData[j + 1] = filter->sortData[j];
			j = j - 1;
		}
		filter->sortData[j + 1] = temp;
	}
	// return data value in middle of sorted array
	return get_median_filter_f(filter);
}

uint8_t movingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len,
                  int nextNum) {
	// Subtract the oldest number from the prev sum, add the new number
	*ptrSum = *ptrSum - ptrArrNumbers[pos] + nextNum;
	// Assign the nextNum to the position in the array
	ptrArrNumbers[pos] = nextNum;
	// return the average
	return *ptrSum / len;
}
