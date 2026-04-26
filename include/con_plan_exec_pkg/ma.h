/*
Purpose: Munkres Algorithm
Author: Divyansh Singh
*/


#include<iostream>
#include<vector>
#include<cstdlib>
#include<chrono>
#include<unistd.h>


#include "con_plan_exec_pkg/basics.h"


#define COST_INF 9999


using namespace std;
using namespace std::chrono;


typedef vector<bool> bool_vec;
typedef vector<int> vec_int;


class MUNKRES_ALGO
{
	public:
		void show_int_vec(vec_int vec);
		void show_int_mat(int_mat mat, uint row_count, uint col_count);
		void show_bool_vec(bool_vec vec);
		void show_bool_mat(bool_mat mat, uint row_count, uint col_count);
		void show_bool_mat2(int_mat cost_mat, bool_mat mat, uint row_count, uint col_count);
		
		void row_op(int_mat &cost_mat, uint row_count, uint col_count);
		void col_op(int_mat &cost_mat, uint row_count, uint col_count);
		void zstar(int_mat cost_mat, bool_mat &zstar_mat, bool_vec &has_zstar_row, bool_vec &has_zstar_col, vec_int &zstar_row_col, vec_int &zstar_col_row, uint row_count, uint col_count);
		uint cov_zstar_cols(bool_vec cov_col, uint row_count, uint col_count);
		vec_int assign(int_mat cost_mat, bool_mat zstar_mat, uint row_count, uint col_count);
		uint comp_seq(int i, int j, bool_mat &zstar_mat, bool_vec &has_zstar_row, bool_vec &has_zstar_col, vec_int &zstar_row_col, vec_int &zstar_col_row, bool_mat &zprime_mat, bool_vec &has_zprime_row, bool_vec &has_zprime_col, vec_int &zprime_row_col, vec_int &zprime_col_row, bool_vec &cov_row, bool_vec &cov_col, uint row_count, uint col_count);
		void min_uncov_pos_op(int_mat &cost_mat, bool_vec cov_row, bool_vec cov_col, uint row_count, uint col_count);
		vec_int munkres(int_mat cost_mat, uint row_count, uint col_count);
};
