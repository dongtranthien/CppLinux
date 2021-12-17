#include <thread>
#include <random>
#include <pthread.h>


#define MAT_SIZE 1000000
#define ROW_SIZE 1000
#define TRACK true

void calc(int* Result_ptr, int* M2_T, int* M1_row_ptr, int start, int end) {
	for (int M1_r = start; M1_r < end; ++M1_r) {
		if (TRACK)
			printf("calculating row %d\n", M1_r);
		
		int* M2_col_ptr = M2_T;
		for (int M2_c = 0; M2_c < ROW_SIZE; ++M2_c) {


			int result = 0;
			for (int i = 0; i < ROW_SIZE; ++i) {
				result += M1_row_ptr[i] * M2_col_ptr[i];
			}
			*Result_ptr++ = result;
			M2_col_ptr += ROW_SIZE;
		}
		M1_row_ptr += ROW_SIZE;

	}
}

int main() {

	int* M1 = new int[MAT_SIZE]; // 1000 x 1000
	int* M2_T = new int[MAT_SIZE]; // Ma tran chuyen vi cua M2 | 1000 x 1000
	int* M_result_sequencep = new int[MAT_SIZE];
	int* M_result_thread = new int[MAT_SIZE];

	////////////////////////////////////////////////////////
	////////// INIT ////////////////////////////////////////
	////////////////////////////////////////////////////////

	srand(0);
	int* M1_ptr = M1;
	int* M2_ptr = M2_T;
	for (int i = 0; i < MAT_SIZE; ++i) {
		*M1_ptr++ = rand() % 1000;
		*M2_ptr++ = rand() % 1000;
	}

	////////////////////////////////////////////////////////
	////////// SEQUENCE CALCULATION ////////////////////////
	////////////////////////////////////////////////////////

	int* M1_row_ptr = M1;
	int* M2_col_ptr = M2_T;
	int* Result_ptr = M_result_sequencep;

	int start = clock();
	for (int M1_r = 0; M1_r < ROW_SIZE; ++M1_r) {
		if (TRACK)
			printf("calculating row %d\n", M1_r);
		M2_col_ptr = M2_T;
		for (int M2_c = 0; M2_c < ROW_SIZE; ++M2_c) {
			int result = 0;
			for (int i = 0; i < ROW_SIZE; ++i) {
				result += M1_row_ptr[i] * M2_col_ptr[i];
			}
			*Result_ptr++ = result;
			M2_col_ptr += ROW_SIZE;

		}
		M1_row_ptr += ROW_SIZE;
		
	}
	int end = clock();

	/////////////////////////
	printf("----\n");
	////////////////////////////////////////////////////////
	////////// THREAD CALCULATION //////////////////////////
	////////////////////////////////////////////////////////
	M1_row_ptr = M1;
	M2_col_ptr = M2_T;
	Result_ptr = M_result_thread;

	int start_thread = clock();

	//calc(int* Result_ptr, int* M2_T, int* M1_row_ptr, int start, int end)
	std::thread t1(calc, Result_ptr + ROW_SIZE * ROW_SIZE / 4 * 0, M2_T, M1 + ROW_SIZE * ROW_SIZE / 4 * 0, ROW_SIZE / 4 * 0, ROW_SIZE / 4 * 1);
	std::thread t2(calc, Result_ptr + ROW_SIZE * ROW_SIZE / 4 * 1, M2_T, M1 + ROW_SIZE * ROW_SIZE / 4 * 1, ROW_SIZE / 4 * 1, ROW_SIZE / 4 * 2);
	std::thread t3(calc, Result_ptr + ROW_SIZE * ROW_SIZE / 4 * 2, M2_T, M1 + ROW_SIZE * ROW_SIZE / 4 * 2, ROW_SIZE / 4 * 2, ROW_SIZE / 4 * 3);
	std::thread t4(calc, Result_ptr + ROW_SIZE * ROW_SIZE / 4 * 3, M2_T, M1 + ROW_SIZE * ROW_SIZE / 4 * 3, ROW_SIZE / 4 * 3, ROW_SIZE / 4 * 4);
	t1.join();
	t2.join();
	t3.join();
	t4.join();

	int end_thread = clock();

	////////////////////////////////////////////////////////
	////////// OUTPUT CALCULATION //////////////////////////
	////////////////////////////////////////////////////////

	printf("Sequence time: %d\n", end - start);
	printf("thread time: %d\n", end_thread - start_thread);
	
	return 0;
}