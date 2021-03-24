#ifndef EXPLORATION_MEDIAN_FILTER_HPP
#define EXPLORATION_MEDIAN_FILTER_HPP

#define MAX_MEDIAN_DATASIZE_I 101
#define MAX_MEDIAN_DATASIZE_F 13
#define MEDIAN_DEFAULT_SIZE 5

#include <cstdint>

/*
TODO: C++ify
template<typename T, std::size_t s> struct MedianFilter {
  T data[s], sortData[s];
  uint8_t dataIndex, size;
};
*/

struct MedianFilterInt {
	int data[MAX_MEDIAN_DATASIZE_I], sortData[MAX_MEDIAN_DATASIZE_I];
	uint8_t dataIndex;
	uint8_t size;
};

struct MedianFilterFloat {
	float data[MAX_MEDIAN_DATASIZE_F], sortData[MAX_MEDIAN_DATASIZE_F];
	uint8_t dataIndex;
	uint8_t size;
};

void init_median_filter_i(struct MedianFilterInt *filter, uint8_t size);

int32_t get_median_filter_i(struct MedianFilterInt *filter);

int32_t update_median_filter_i(struct MedianFilterInt *filter,
                               int32_t new_data);

void init_median_filter_f(struct MedianFilterFloat *filter, uint8_t size);

float get_median_filter_f(struct MedianFilterFloat *filter);

float update_median_filter_f(struct MedianFilterFloat *filter, float new_data);

uint8_t movingAvg(int *ptrArrNumbers, long *ptrSum, int pos, int len,
                  int nextNum);

#endif /* EXPLORATION_MEDIAN_FILTER_HPP */
