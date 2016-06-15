#include "tuw_marker_slam/munkre.h"

using namespace tuw;

std::vector<std::pair<size_t, size_t>> Munkre::find_minimum_assignment ( const cv::Mat_<double> costmatrix ) {
    cv::Mat_<double> C;
    cv::Mat_<Zero> M;
    std::vector<bool> row_cover, col_cover;
    std::vector<std::pair<size_t, size_t>> path, result;
    bool done = false;
    bool transpose;
    size_t k;
    size_t step = 0;

    while (!done) {
        // invariance check
        assert ( 0 <= step && step <= 7);
        assert ( C.cols >= C.rows );
        assert ( (C.rows == M.rows && C.cols == M.cols) || step < 3 );
        assert ( M.rows == row_cover.size() && M.cols == col_cover.size() );

        switch (step) {
            case 0:
                step_0 (costmatrix, C, transpose, k, step);
                break;
            case 1:
                step_1 (C, step);
                break;
            case 2:
                step_2 (C, M, row_cover, col_cover, step);
                break;
            case 3:
                step_3 (M, col_cover, k, step);
                break;
            case 4:
                step_4 (C, M, row_cover, col_cover, path, step);
                break;
            case 5:
                step_5 (M, row_cover, col_cover, path, step);
                break;
            case 6:
                step_6 (C, row_cover, col_cover, step);
                break;
            case 7:
                step_7 (M, transpose, result, k, step);
                done = true;
                break;
            default:
                assert ( false );
                break;
        }

        //print (C, M, row_cover, col_cover, step);
    }

    return result;
}

void Munkre::step_0 (const cv::Mat_<double> costmatrix, cv::Mat_<double> &C, bool &transpose, size_t &k, size_t &step) {
    assert( step == 0 );

    // copy/save original data
    C = costmatrix.clone();

    // C must have at least as many columns as rows
    if ( C.rows > C.cols) {
        C = C.t();
        transpose = true;
    } else {
        transpose = false;
    }

    k = C.rows;
    step = 1;
}

void Munkre::step_1 (cv::Mat_<double> &C, size_t &step) {
    assert( step == 1 );

    for (size_t r = 0; r < C.rows; r++) {
        // find smallest element in current row
        double min = C[r][0];
        for (size_t c = 1; c < C.cols; c++) {
            if ( C[r][c] < min )
                min = C[r][c];
        }

        // subtract smallest element from every element in current row
        for (size_t c = 0; c < C.cols; c++) {
            C[r][c] -= min;
            if ( std::abs ( C[r][c] ) < __DBL_MIN__)
                C[r][c] = 0;
        }
    }

    step = 2;
}

void Munkre::step_2 (const cv::Mat_<double> C, cv::Mat_<Zero> &M, std::vector<bool> &row_cover, std::vector<bool> &col_cover, size_t &step) {
    assert ( step == 2 );

    /* M[r][c] =
     * UNDEF ... C[r][c] is undefined yet
     * STAR  ... C[r][c] is a starred zero
     * PRIME ... C[r][c] is a primed zero
     */
    M = cv::Mat_<Zero> ( C.rows, C.cols, UNDEF );

    /* row_cover[r] =
     * false ... row r contains no starred zero
     * true  ... row r contains a starred zero
     */
    row_cover = std::vector<bool> ( C.rows );

    /* col_cover[c] =
     * false ... column c contains no starred zero
     * true  ... column c contains a starred zero
     */
    col_cover = std::vector<bool> ( C.cols );

    // find zeros and if applicable mark them starred
    for (size_t r = 0; r < C.rows; r++) {
        for (size_t c = 0; c < C.cols; c++) {
            if ( C[r][c] == 0 && !row_cover[r] && !col_cover[c] ) {
                M[r][c] = STAR;
                row_cover[r] = true;
                col_cover[c] = true;
            }
        }
    }

    // clear covers
    for (size_t r = 0; r < row_cover.size(); r++)
        row_cover[r] = false;
    for (size_t c = 0; c < col_cover.size(); c++)
        col_cover[c] = false;

    step = 3;
}

void Munkre::step_3 (const cv::Mat_<Zero> M, std::vector<bool> &col_cover, const size_t k, size_t &step) {
    assert ( step == 3 );

    // cover and count all columns containing a starred zero
    size_t count = 0;
    for (size_t c = 0; c < M.cols; c++) {
        for (size_t r = 0; r < M.rows; r++) {
            if (M[r][c] == 1) {
                col_cover[c] = true;
                count++;
                break;
            }
        }
    }
    
    if (count >= k)
        step = 7;
    else
        step = 4;
}

void Munkre::step_4 (const cv::Mat_<double> C, cv::Mat_<Zero> &M, std::vector<bool> &row_cover, std::vector<bool> &col_cover, std::vector<std::pair<size_t, size_t>> &path, size_t &step) {
    assert ( step == 4 );

    while (true) {
        int row, col;
        find_uncovered_zero(C, row_cover, col_cover, row, col);
        if ( 0 <= row && row < M.rows && 0 <= col && col < M.cols ) {
            // prime the found uncovered zero
            M[row][col] = PRIME;
            size_t col_prime = col;
            
            col = find_zero_in_row(M, row, STAR);
            if (!(0 <= col && col < M.cols)) {
                // no starred zero in the row containing the currently primed zero
                path = std::vector<std::pair<size_t, size_t>> (1);
                path[0] = std::pair<size_t, size_t>((size_t) row, (size_t) col_prime);
                step = 5;
                break;
            } else {
                // cover the row containing the currently primed zero and
                // uncover the column containing the found starred zero
                row_cover[row] = true;
                col_cover[col] = false;
            }
        } else {
            // no uncovered zero left
            step = 6;
            break;
        }
    }
}

void Munkre::step_5 (cv::Mat_<Zero> M, std::vector<bool> &row_cover, std::vector<bool> &col_cover, std::vector<std::pair<size_t, size_t>> &path, size_t &step) {
    assert ( step == 5 );
    assert ( path.size() == 1 );

    while (true) {
        int row, col;
        std::pair<size_t, size_t> Z0, Z1, Z2;

        // Z0...primed zero (of last time)
        Z0 = path[path.size()-1];

        // Z1...starred zero in the column of Z0 (can exist)
        row = find_zero_in_col(M, Z0.second, STAR);
        if ( 0 <= row && row < M.rows ) {
            Z1 = std::pair<size_t, size_t> ((size_t) row, Z0.second);
            path.push_back(Z1);
        } else break;

        // Z2...primed zero in the row of Z1 (must exist).
        col = find_zero_in_row(M, Z1.first, PRIME);
        assert ( 0 <= col && col < M.cols);
        Z2 = std::pair<size_t, size_t> (Z1.first, (size_t) col);
        path.push_back(Z2);
    }

    // augment path
    // - starred zero gets unstarred
    // - primed zero gets starred
    for (size_t i = 0; i < path.size(); i++) {
        assert ( M[path[i].first][path[i].second] == PRIME ||
                 M[path[i].first][path[i].second] == STAR );

        if (M[path[i].first][path[i].second] == STAR)
            M[path[i].first][path[i].second] = UNDEF;
        else
            M[path[i].first][path[i].second] = STAR;
    }

    // erase primes
    for (size_t r = 0; r < M.rows; r++) {
        for (size_t c = 0; c < M.cols; c++) {
            if (M[r][c] == PRIME)
                M[r][c] = UNDEF;
        }
    }
    
    // clear covers
    for (size_t r = 0; r < row_cover.size(); r++)
        row_cover[r] = false;
    for (size_t c = 0; c < col_cover.size(); c++)
        col_cover[c] = false;

    step = 3;
}

void Munkre::step_6 (cv::Mat_<double> C, const std::vector<bool> row_cover, const std::vector<bool> col_cover, size_t &step) {
    assert ( step == 6 );

    // find smallest uncovered element
    double min = std::numeric_limits<double>::infinity();
    for (size_t r = 0; r < C.rows; r++) {
        for (size_t c = 0; c < C.cols; c++) {
            if (!row_cover[r] && !col_cover[c] && C[r][c] < min)
                min = C[r][c];
        }
    }
    assert ( min < std::numeric_limits<double>::infinity() );

    // add found value to every element of each covered row and
    // subtract found value from every element of each uncovered column
    for (size_t r = 0; r < C.rows; r++) {
        for (size_t c = 0; c < C.cols; c++) {
            if (row_cover[r])
                C[r][c] += min;
            if (!col_cover[c])
                C[r][c] -= min;
            if ( std::abs ( C[r][c] ) < __DBL_MIN__)
                C[r][c] = 0;
        }
    }

    step = 4;
}

void Munkre::step_7 (const cv::Mat_<Zero> M, const bool transpose, std::vector<std::pair<size_t, size_t>> &result, size_t &k, size_t &step) {
    assert ( step == 7 );

    // prepare result
    size_t i = 0;
    result = std::vector<std::pair<size_t, size_t>> (k);
    for (size_t r = 0; r < M.rows; r++) {
        for (size_t c = 0; c < M.cols; c++) {
            bool col_finish = false;
            if (M[r][c] == STAR) {
                assert ( !col_finish );
                
                // restore original arrangement 
                if (transpose)
                    result[i] = std::pair<size_t, size_t> (c, r);
                else
                    result[i] = std::pair<size_t, size_t> (r, c);

                col_finish = true;
                i++;
            }
        }
    }
    assert ( i == k );

    step = 8;
}

void Munkre::find_uncovered_zero (const cv::Mat_<double> C, const std::vector<bool> row_cover, const std::vector<bool> col_cover, int &row, int &col) {
    assert ( C.rows == row_cover.size() && C.cols == col_cover.size() );

    row = -1;
    col = -1;
    for (size_t r = 0; r < C.rows; r++) {
        for (size_t c = 0; c < C.cols; c++) {
            if (C[r][c] == 0 && !row_cover[r] && !col_cover[c]) {
                row = r;
                col = c;
                return;
            }
        }
    }
}

int Munkre::find_zero_in_row (const cv::Mat_<Zero> M, const size_t row, const Zero type) {
    for (size_t c = 0; c < M.cols; c++) {
        if (M[row][c] == type)
            return c;
    }

    return -1;
}

int Munkre::find_zero_in_col (const cv::Mat_<Zero> M, const size_t col, const Zero type) {
    for (size_t r = 0; r < M.rows; r++) {
        if (M[r][col] == type)
            return r;
    }

    return -1;
}

void Munkre::print (const cv::Mat_<double> C, const cv::Mat_<Zero> M, const std::vector<bool> row_cover, const std::vector<bool> col_cover, const size_t step) {
    printf("next step %lu:\n", step);

    for (size_t r = 0; r < C.rows; r++) {
        if (r == 0)
            printf("[");
        else
            printf(" ");

        for (size_t c = 0; c < C.cols; c++) {
            if (row_cover[r] || col_cover[c])
                printf("(");
            else
                printf(" ");

            printf("%.2f", C[r][c]);

            if (row_cover[r] || col_cover[c])
                printf(")");
            else
                printf(" ");

            if (M.rows == C.rows && M.cols == C.cols) {
                switch (M[r][c]) {
                    case UNDEF:
                        printf(" ");
                        break;
                    case STAR:
                        printf("*");
                        break;
                    case PRIME:
                        printf("'");
                        break;
                }
            }

            if (c < C.cols - 1)
                printf(", ");
        }

        if (r == C.rows-1)
            printf("]\n");
        else
            printf(";\n");
    }
}
