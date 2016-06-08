#ifndef MUNKRE_H
#define MUNKRE_H

#include <opencv2/core/core.hpp>

namespace tuw {
/**
 * This class implements Munkre's assignment algorithm
 * 
 * see http://csclab.murraystate.edu/bob.pilgrim/445/munkres.html
 */
class Munkre {
public:
    /**
     * TODO
     */
    static std::vector<std::pair<size_t, size_t>> find_minimum_assignment ( const cv::Mat_<double> costmatrix );

private:
    /**
     * Different kinds of zeros used in Munkre's assignment algorithm
     */
    enum Zero {
        UNDEF = 0,
        STAR = 1,
        PRIME = 2,
    };

    /**
     * Create an nxm  matrix called the cost matrix in which each element represents e.g. the cost of
     * assigning one of n workers to one of m jobs. Rotate the matrix so that there are at least as
     * many columns as rows and let k=min(n,m).
     */
    static void step_0 (const cv::Mat_<double> costmatrix, cv::Mat_<double> &C, bool &transpose, size_t &k, size_t &step);

    /**
     * For each row of the matrix, find the smallest element and subtract it from every element in its row.
     * Go to Step 2.
     */
    static void step_1 (cv::Mat_<double> &C, size_t &step);

    /**
     * Find a zero (Z) in the resulting matrix.  If there is no starred zero in its row or column,
     * star Z. Repeat for each element in the matrix. Go to Step 3.
     */
    static void step_2 (const cv::Mat_<double> C, cv::Mat_<Zero> &M, std::vector<bool> &row_cover, std::vector<bool> &col_cover, size_t &step);

    /**
     * Cover each column containing a starred zero.
     * If k columns are covered, the starred zeros describe a complete set of unique assignments.
     * In this case, Go to Step 7, otherwise, Go to Step 4.
     */
    static void step_3 (const cv::Mat_<Zero> M, std::vector<bool> &col_cover, const size_t k, size_t &step);

    /**
     * Find an uncovered zero and prime it.
     * If there is no starred zero in the row containing this primed zero, Go to Step 5.
     * Otherwise, cover this row and uncover the column containing the starred zero.
     * Continue in this manner until there are no uncovered zeros left and Go to Step 6.
     */
    static void step_4 (const cv::Mat_<double> C, cv::Mat_<Zero> &M, std::vector<bool> &row_cover, std::vector<bool> &col_cover, std::vector<std::pair<size_t, size_t>> &path, size_t &step);

    /**
     * Construct a series of alternating primed and starred zeros as follows.
     * Let Z0 represent the uncovered primed zero found in Step 4.
     * Let Z1 denote the starred zero in the column of Z0 (if any).
     * Let Z2 denote the primed zero in the row of Z1 (there will always be one).
     * Continue until the series terminates at a primed zero that has no starred zero in its column.
     * Unstar each starred zero of the series, star each primed zero of the series,
     * erase all primes and uncover every line in the matrix.
     * Return to Step 3.
     */
    static void step_5 (cv::Mat_<Zero> M, std::vector<bool> &row_cover, std::vector<bool> &col_cover, std::vector<std::pair<size_t, size_t>> &path, size_t &step);

    /**
     * Add the smallest uncovered value to every element of each covered row, and
     * subtract it from every element of each uncovered column.
     * Return to Step 4 without altering any stars, primes, or covered lines.
     */
    static void step_6 (cv::Mat_<double> C, const std::vector<bool> row_cover, const std::vector<bool> col_cover, size_t &step);

    /**
     * DONE
     * 
     * Assignment pairs are indicated by the positions of the starred zeros in the cost matrix.
     * If C(i,j) is a starred zero, then the element associated with row i is assigned to the
     * element associated with column j.
     */
    static void step_7 (const cv::Mat_<Zero> M, const bool transpose, std::vector<std::pair<size_t, size_t>> &result, size_t &k, size_t &step);

    /**
     * Find an uncovered zero.
     */
    static void find_uncovered_zero (const cv::Mat_<double> C, const std::vector<bool> row_cover, const std::vector<bool> col_cover, int &row, int &col);

    /**
     * Find a starred zero in a row.
     */
    static int find_zero_in_row (const cv::Mat_<Zero> M, const size_t row, const Zero type);

    /**
     * Find a starred zero in a row.
     */
    static int find_zero_in_col (const cv::Mat_<Zero> M, const size_t col, const Zero type);

    /**
     * Prints the current state.
     */
    static void print (const cv::Mat_<double> C, const cv::Mat_<Zero> M, const std::vector<bool> row_cover, const std::vector<bool> col_cover, const size_t step);
};
};

#endif // MUNKRE_H
