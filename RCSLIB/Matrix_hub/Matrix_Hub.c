/**
 * @name:Matrix_Hub.c
 * @brief:?????????????
 * @author:https://github.com/Amoiensis/Matrix_hub
 * @version:Amoiensis V1.52
 * @fork_version:CYK-Dot V1.0
*/

#include "matrix.h"

Matrix* M_Sample(Matrix* _mat_source, Matrix* _mat_sample, int mode){/*
 * Sample some row/col from Matrix.
 * ??????/??????/?.
 * */
    Matrix * mat_result = NULL;
    int row, col, row_num, col_num, index_from;
    if (mode == _COLUMN_){
        row_num = _mat_source->row;
        col_num = _mat_sample->column;
        mat_result = M_Zeros(row_num, col_num);
        for(row=0;row<row_num;row++){
            for(col=0;col<col_num;col++){
                index_from = (int)_mat_sample->data[col];
                mat_result->data[row*col_num + col] = _mat_source->data[row*(_mat_source->column) + index_from];
            }
        }
    }else{
        row_num = _mat_sample->row;
        col_num = _mat_source->column;
        mat_result = M_Zeros(row_num, col_num);
        for(row=0;row<row_num;row++){
            index_from = (int)_mat_sample->data[row];
            for(col=0;col<col_num;col++){
                mat_result->data[row*col_num + col] = _mat_source->data[index_from*(_mat_source->column) + col];
            }
        }
    }
    return mat_result;
}

double M_cond(Matrix *_mat, int Setting){ /*
 * Conditon Value of the Matrix.
 * ????? */
    int i,j;
    double matrix_cond = 0;
    if (_mat->column == _mat->row){
        if((Setting == 1) || (Setting == 2) || (Setting == INF) || (Setting == FRO)){
            Matrix * _mat_inv = M_Inverse(_mat);
            matrix_cond = M_norm(_mat,Setting)*M_norm(_mat_inv,Setting);
            M_Free(_mat_inv);
        }else{
            OS_Err_Collector(M_cond_025);
            OS_Commander("pause");
        }
    }else{
        OS_Err_Collector(M_cond_024);
        OS_Commander("pause");
    }
    return matrix_cond;
}

Matrix *Hilbert(int order){/*
 * Generate Hilbert Matrix (create).
 * ????????*/
    Matrix *Hilbert_mat = (Matrix *) OS_Malloc(sizeof(Matrix));
    Hilbert_mat->column = order;
    Hilbert_mat->row = order;
    int i, j;
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc((order*order) * sizeof(MATRIX_TYPE));
    for (i = 0; i < order; i++) {
        for (j = 0; j < order; j++){
            if (i>j){ // [??] ????: ????; ????????;
                data[i*order+j] = data[j*order+i];
            }else{
                data[i*order+j] = 1.0/(i+j+1);
            }
        }
    }
    Hilbert_mat->data = data;
    return Hilbert_mat;
}

int Etrans_free(Etrans_struct *_Etrans_){/*
 * (In Func: M_rank) Free memory for Elementary Transformation.
 * (??: M_rank) ??????????*/
    Etrans_struct *temp_Etrans = _Etrans_;
    do {
        temp_Etrans = temp_Etrans->forward_E_trans;
        if (temp_Etrans != NULL) {
            OS_Free(temp_Etrans->next_E_trans);
        }
    } while (temp_Etrans != NULL);
    return 0;
}

int M_rank(Matrix *_mat){/*
 * Rank of the Matrix
 * ???? */
    (_DETAILED_>=2)?OS_Err_Collector(">>rank(Matrix_%x)=\n... => transform...\n", _mat):0;
	
    M_inv_struct *_Uptri_ = M_Uptri_4inv(_mat);
    MATRIX_TYPE *_mat_uptri = _Uptri_ ->_matrix->data;
    (_DETAILED_>=2)?M_print(_Uptri_->_matrix):0;
    int rank = 0;
    int i, j, row = _mat->row, column = _mat->column;
    for (i=0;i<row;i++){
        for (j=0;j<column;j++){
            if (_mat_uptri[i*column + j] != 0){
                rank ++;
                break;
            }
        }
    }
    OS_Free(_Uptri_->_matrix);
    Etrans_free(_Uptri_->_Etrans_head);
    OS_Free(_Uptri_);
		
    (_DETAILED_>=2)?OS_Err_Collector("... => transform end.\n"):0;
    return rank;
}
Matrix *Matrix_gen(int row, int column, MATRIX_TYPE *data) {/*
 * Generate a new Matrix(struct).
 * ??_????*/
    Matrix *_mat = (Matrix *) OS_Malloc(sizeof(Matrix));
    _mat->row = row;
    _mat->column = column;
    int size = _mat->row * _mat->column;
    _mat->data = (MATRIX_TYPE *) OS_Malloc((size) * sizeof(MATRIX_TYPE));
//    using memory-copy to speed up.
    memcpy(_mat->data, data, (size) * sizeof(MATRIX_TYPE));
//    int i;
//    for (i = 0; i < size; i++) {
//        _mat->data[i] = data[i];
//    }
    return _mat;
}

Matrix *Matrix_copy(Matrix *_mat_sourse) {/*
 * Copy to a new Matrix (create).
 * ????(?????)*/
    Matrix *_mat_copy = Matrix_gen(_mat_sourse->row, _mat_sourse->column, _mat_sourse->data);
    return _mat_copy;
}

Matrix *M_add_sub(MATRIX_TYPE scale_mat_subed, Matrix *_mat_subed, MATRIX_TYPE scale_mat_minus, Matrix *_mat_minus) {/*
 * Addition/ Subtraction between Matrix-s (create).
 * ????? */
    Matrix *_mat_result = NULL;
    if ((_mat_subed->column == _mat_minus->column) && (_mat_subed->row == _mat_minus->row)) {
        _mat_result = Matrix_copy(_mat_subed);
        int size = (_mat_subed->row) * (_mat_subed->column), i;
        for (i = 0; i < size; i++) {
            _mat_result->data[i] = (_mat_result->data[i]) * scale_mat_subed - (_mat_minus->data[i]) * scale_mat_minus;
        }
    } else {
        OS_Err_Collector(M_add_sub_003);
    }
    return _mat_result;
}

Matrix *M_mul(Matrix *_mat_left, Matrix *_mat_right) {/*
 * Matrix multiplication (create new one, abbr. create).
 * _mat_result = _mat_left*_mat_right
 * ???? */
    (_DETAILED_>=3)?OS_Err_Collector(">>Matrix_%x * Matrix_%x =\n", _mat_left, _mat_right):0;
    /*Determine_Matrix_Dimensions*/
    Matrix *_mat_result = NULL;
    if (_mat_left->column != _mat_right->row) {
        OS_Err_Collector(M_mul_001);
    } else {
        _mat_result = (Matrix *) OS_Malloc(sizeof(Matrix));
        int row = _mat_left->row;
        int mid = _mat_left->column;
        int column = _mat_right->column;
        int i, j, k;
        MATRIX_TYPE *_data = (MATRIX_TYPE *) OS_Malloc((row * column) * sizeof(MATRIX_TYPE));
        MATRIX_TYPE temp = 0;
        /*Ergodic*/
        for (i = 0; i < row; i++) {
            for (j = 0; j < column; j++) {
                /*Caculate Element*/
                temp = 0;
                for (k = 0; k < mid; k++) {
                    temp += (_mat_left->data[i * mid + k]) * (_mat_right->data[k * column + j]);
                }
                _data[i * column + j] = temp;
            }
        }
        _mat_result->row = row;
        _mat_result->column = column;
        _mat_result->data = _data;
    }
    (_DETAILED_>=3)?OS_Err_Collector("\tMatrix_%x\n", _mat_result):0;
    return _mat_result;
}

int M_print(Matrix *_mat) {/*
 * Matrix Print, Display.
 * ???? */
    OS_Err_Collector(">>Matrix_%x:\n", _mat);
    int i, j;
    for (i = 0; i < _mat->row; i++) {
        for (j = 0; j < _mat->column; j++) {
            OS_Err_Collector(PRECISION, _mat->data[i * (_mat->column) + j]);
        }
        OS_Err_Collector("\n");
    }
    return 0;
}

Matrix *M_I(int order) {/*
 * Generate a identity Matrix (create).
 * ?????? */
    Matrix *I_mat = (Matrix *) OS_Malloc(sizeof(Matrix));
    I_mat->column = order;
    I_mat->row = order;
    int size = order * order, i;
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc((size) * sizeof(MATRIX_TYPE));
    for (i = 0; i < size; i++) {
        data[i] = 0;
    }
    for (i = 0; i < order; i++) {
        data[i * (order + 1)] = 1;
    }
    I_mat->data = data;
    return I_mat;
}

int M_E_trans(Matrix *_mat, Etrans_struct *_Etrans_, int line_setting) {/*
 * Matrix elementary transformation.
 * line_setting in [_ROW_, _COLUMN_]
 * ?????????
 * line_setting: ???????????????*/
    int line_num, i;
    if (line_setting == _ROW_) {
        /*?????*/
        line_num = _mat->column;
        if (_Etrans_->scale) {
            for (i = 0; i < line_num; i++) {
                _mat->data[(_Etrans_->minuend_line - 1) * (_mat->column) + i] -=
                        (_Etrans_->scale) * (_mat->data[(_Etrans_->subtractor_line - 1) * (_mat->column) + i]);

            }
        } else {
            if ((_Etrans_->minuend_line < 0) && (_Etrans_->subtractor_line < 0)) {/*??*/
                M_Swap(_mat, -(_Etrans_->minuend_line), -(_Etrans_->subtractor_line), line_setting);
            }
        }
    } else {
        /*?????*/
        line_num = _mat->row;
        if (_Etrans_->scale) {
            for (i = 0; i < line_num; i++) {
                _mat->data[(_Etrans_->minuend_line - 1) + (_mat->column) * i] -=
                        (_Etrans_->scale) * (_mat->data[(_Etrans_->subtractor_line - 1) + (_mat->column) * i]);
            }
        } else {
            if ((_Etrans_->minuend_line < 0) && (_Etrans_->subtractor_line < 0)) {/*??*/
                M_Swap(_mat, -(_Etrans_->minuend_line), -(_Etrans_->subtractor_line), line_setting);
            }
        }
    }
    return 0;
}

Matrix *Etrans_4_Inverse(Matrix *_mat_result, Etrans_struct *_Etrans_, int line_setting) {/*
 * Inverse Matrix of elementary transformations (create).
 * ????? ??? ?? */
    Etrans_struct *temp_Etrans = _Etrans_, *temp_Etrans_pre = _Etrans_;
    int temp_num = 0;
    // ?????? @1u2e, github.com/Amoiensis/Matrix_hub/issues/4
    while (temp_Etrans != NULL) {
        temp_num = temp_Etrans->minuend_line;
        temp_Etrans->minuend_line = temp_Etrans->subtractor_line;
        temp_Etrans->subtractor_line = temp_num;
        M_E_trans(_mat_result, temp_Etrans, line_setting);
        // ???????? @1u2e, github.com/Amoiensis/Matrix_hub/issues/4
        temp_Etrans = temp_Etrans->forward_E_trans;
        OS_Free(temp_Etrans_pre);
        temp_Etrans_pre = temp_Etrans;
    }
    return _mat_result;
}

Matrix *Etrans_2_Matrix(Etrans_struct *_Etrans_, int order, int line_setting) {/*
 * Transforms the elementary transformation into Matrix (create).
 * ??????? */
    Etrans_struct *temp_Etrans = _Etrans_;
    Matrix *_mat_result = M_I(order);
    if (_Etrans_ != NULL) {
        while ((temp_Etrans->next_E_trans) != NULL) {
            temp_Etrans = temp_Etrans->next_E_trans;
        }
        do {
            temp_Etrans->scale = (-1) * temp_Etrans->scale;
            M_E_trans(_mat_result, temp_Etrans, line_setting);
            temp_Etrans->scale = (-1) * temp_Etrans->scale;
            temp_Etrans = temp_Etrans->forward_E_trans;
            if (temp_Etrans != NULL) {
                OS_Free(temp_Etrans->next_E_trans);
            }
        } while (temp_Etrans != NULL);
    }
    return _mat_result;
}

Uptri_struct *M_Uptri_(Matrix *_mat_source) {/*
 * Upper-Triangulation transformation on the Matrix (create).
 * ????*/
    Matrix *_mat = Matrix_copy(_mat_source);
    int i, j, k, flag;
    Etrans_struct *_Etrans_temp_last = NULL;
    Etrans_struct *_Etrans_temp_head = NULL;

    /*????*/
    for (i = 0; i < _mat->column; i++) {
        for (j = i + 1; j < _mat->row; j++) {
            flag = 0;
            Etrans_struct *_Etrans_temp = (Etrans_struct *) OS_Malloc(sizeof(Etrans_struct));
            _Etrans_temp->minuend_line = j + 1;
            _Etrans_temp->subtractor_line = i + 1;
            if ((_mat->data[(_mat->column) * i + i]) != 0) {
                _Etrans_temp->scale = (_mat->data[(_mat->column) * j + i]) / (_mat->data[(_mat->column) * i + i]);
            } else {
                _Etrans_temp->scale = 0;
                for (k = i + 1; k < _mat->row; k++) {
                    flag = 1;//?????
                    if ((_mat->data[(_mat->column) * k + i]) != 0) {
                        _Etrans_temp->minuend_line = -(i + 1);
                        _Etrans_temp->subtractor_line = -(k + 1);
                        flag = 2;//???????
                        break;
                    }
                }
                if (flag == 1) {
                    break;
                }
            }
            _Etrans_temp->forward_E_trans = NULL;
            _Etrans_temp->next_E_trans = NULL;
            if (_Etrans_temp_head == NULL) {
                _Etrans_temp_head = _Etrans_temp;
                _Etrans_temp->forward_E_trans = NULL;
            } else {
                _Etrans_temp->forward_E_trans = _Etrans_temp_last;

            }
            if ((i + 1) == _mat->column) {
                _Etrans_temp->next_E_trans = NULL;
            } else {
                if (_Etrans_temp_last != NULL) {
                    _Etrans_temp_last->next_E_trans = _Etrans_temp;
                }
            }
            M_E_trans(_mat, _Etrans_temp, _ROW_);
            _Etrans_temp_last = _Etrans_temp;

            if (flag == 2) {
                i = i - 1;
                break;
            }
        }
    }
    Matrix *trans_mat = Etrans_2_Matrix(_Etrans_temp_head, _mat->row, _ROW_);
    Uptri_struct *_Uptri = (Uptri_struct *) OS_Malloc(sizeof(Uptri_struct));
    _Uptri->trans_matrix = trans_mat;
    _Uptri->Uptri_matrix = _mat;
    (_DETAILED_>=1)?OS_Err_Collector(">>Uptri(Matrix_%x)=\n", _mat_source):0;
    (_DETAILED_>=1)?OS_Err_Collector("\tMatrix_%x * Matrix_%x\n", trans_mat, _mat):0;
    return _Uptri;
}

M_inv_struct *M_Uptri_4inv(Matrix *_mat_source) {/*
 * For inverse, upper-triangulation transformation on the Matrix (create).
 * ????, ????*/
    Matrix *_mat = Matrix_copy(_mat_source);
    int i, j, k, flag;
    Etrans_struct *_Etrans_temp_last = NULL;
    Etrans_struct *_Etrans_temp_head = NULL;
    Etrans_struct *_Etrans_temp_head_store = NULL;
    /*????*/
    for (i = 0; i < _mat->column; i++) {
        for (j = i + 1; j < _mat->row; j++) {
            flag = 0;
            Etrans_struct *_Etrans_temp = (Etrans_struct *) OS_Malloc(sizeof(Etrans_struct));
            _Etrans_temp->minuend_line = j + 1;
            _Etrans_temp->subtractor_line = i + 1;
            if ((_mat->data[(_mat->column) * i + i]) != 0) {
                _Etrans_temp->scale = (_mat->data[(_mat->column) * j + i]) / (_mat->data[(_mat->column) * i + i]);
            } else {
                _Etrans_temp->scale = 0;
                for (k = i + 1; k < _mat->row; k++) {
                    flag = 1;//?????
                    if ((_mat->data[(_mat->column) * k + i]) != 0) {
                        _Etrans_temp->minuend_line = -(i + 1);
                        _Etrans_temp->subtractor_line = -(k + 1);
                        flag = 2;//???????
                        break;
                    }
                }

                if (flag == 1) {
                    break;
                }
            }
            _Etrans_temp->forward_E_trans = NULL;
            _Etrans_temp->next_E_trans = NULL;
            if (_Etrans_temp_head == NULL) {
                _Etrans_temp_head = _Etrans_temp;
                _Etrans_temp->forward_E_trans = NULL;
                // [??] ??????
                _Etrans_temp_head_store =_Etrans_temp_head;
            } else {
                _Etrans_temp->forward_E_trans = _Etrans_temp_last;

            }
            if ((i + 1) == _mat->column) {
                _Etrans_temp->next_E_trans = NULL;
            } else {
                if (_Etrans_temp_last != NULL) {
                    _Etrans_temp_last->next_E_trans = _Etrans_temp;
                }
            }
            M_E_trans(_mat, _Etrans_temp, _ROW_);
            _Etrans_temp_last = _Etrans_temp;
            if (flag == 2) {
                i = i - 1;
                break;
            }
        }
    }
    M_inv_struct *_Uptri = (M_inv_struct *) OS_Malloc(sizeof(M_inv_struct));
    _Uptri->_matrix = _mat;

    _Uptri->_Etrans_head = _Etrans_temp_last;
    return _Uptri;
}

Lowtri_struct *M_Lowtri_(Matrix *_mat_source) {/*
 * Lower-Triangulation transformation on the Matrix (create).
 * ????*/
    Matrix *_mat = Matrix_copy(_mat_source);
    int i, j, k, flag;
    Etrans_struct *_Etrans_temp_last = NULL;
    Etrans_struct *_Etrans_temp_head = NULL;
    for (i = 0; i < _mat->row; i++) {
        for (j = i + 1; j < _mat->column; j++) {
            flag = 0;
            Etrans_struct *_Etrans_temp = (Etrans_struct *) OS_Malloc(sizeof(Etrans_struct));
            _Etrans_temp->minuend_line = j + 1;
            _Etrans_temp->subtractor_line = i + 1;
            if ((_mat->data[(_mat->column) * i + i]) != 0) {
                _Etrans_temp->scale = (_mat->data[(_mat->column) * i + j]) / (_mat->data[(_mat->column) * i + i]);;
            } else {
                _Etrans_temp->scale = 0;
                for (k = i + 1; k < _mat->row; k++) {
                    flag = 1;//?????
                    if ((_mat->data[(_mat->column) * k + i]) != 0) {
                        _Etrans_temp->minuend_line = -(i + 1);
                        _Etrans_temp->subtractor_line = -(k + 1);
                        flag = 2;//???????
                        break;
                    }
                }
                if (flag == 1) {
                    break;
                }
            }
            _Etrans_temp->forward_E_trans = NULL;
            _Etrans_temp->next_E_trans = NULL;
            if (_Etrans_temp_head == NULL) {
                _Etrans_temp_head = _Etrans_temp;
                _Etrans_temp->forward_E_trans = NULL;
            } else {
                _Etrans_temp->forward_E_trans = _Etrans_temp_last;
            }
            if ((i + 1) == _mat->column) {
                _Etrans_temp->next_E_trans = NULL;
            } else {
                if (_Etrans_temp_last != NULL) {
                    _Etrans_temp_last->next_E_trans = _Etrans_temp;
                }
            }
            M_E_trans(_mat, _Etrans_temp, _COLUMN_);
            _Etrans_temp_last = _Etrans_temp;
            if (flag == 2) {
                i = i - 1;
                break;
            }
        }
    }
    Matrix *trans_mat = Etrans_2_Matrix(_Etrans_temp_head, _mat->row, _COLUMN_);
    Lowtri_struct *_Lowtri = (Lowtri_struct *) OS_Malloc(sizeof(Lowtri_struct));
    _Lowtri->trans_matrix = trans_mat;
    _Lowtri->Lowtri_matrix = _mat;
    (_DETAILED_>=1)?OS_Err_Collector(">>Lowtri(Matrix_%x)=\n", _mat_source):0;
    (_DETAILED_>=1)?OS_Err_Collector("\tMatrix_%x * Matrix_%x\n", _mat, trans_mat):0;
    return _Lowtri;
}

M_inv_struct *M_Lowtri_4inv(Matrix *_mat_source) {/*
 * For inverse , lower-triangulation transformation on the Matrix (create).
 * ????_????*/
    Matrix *_mat = Matrix_copy(_mat_source);
    int i, j, k, flag;
    Etrans_struct *_Etrans_temp_last = NULL;
    Etrans_struct *_Etrans_temp_head = NULL;
    for (i = 0; i < _mat->row; i++) {
        for (j = i + 1; j < _mat->column; j++) {
            flag = 0;
            Etrans_struct *_Etrans_temp = (Etrans_struct *) OS_Malloc(sizeof(Etrans_struct));
            _Etrans_temp->minuend_line = j + 1;
            _Etrans_temp->subtractor_line = i + 1;
            if ((_mat->data[(_mat->column) * i + i]) != 0) {
                _Etrans_temp->scale = (_mat->data[(_mat->column) * i + j]) / (_mat->data[(_mat->column) * i + i]);;
            } else {
                _Etrans_temp->scale = 0;
                for (k = i + 1; k < _mat->row; k++) {
                    flag = 1;//?????
                    if ((_mat->data[(_mat->column) * k + i]) != 0) {
                        _Etrans_temp->minuend_line = -(i + 1);
                        _Etrans_temp->subtractor_line = -(k + 1);
                        flag = 2;//???????
                        break;
                    }
                }
                if (flag == 1) {
                    break;
                }
            }
            _Etrans_temp->forward_E_trans = NULL;
            _Etrans_temp->next_E_trans = NULL;
            if (_Etrans_temp_head == NULL) {
                _Etrans_temp_head = _Etrans_temp;
                _Etrans_temp->forward_E_trans = NULL;
            } else {
                _Etrans_temp->forward_E_trans = _Etrans_temp_last;
            }
            if ((i + 1) == _mat->column) {
                _Etrans_temp->next_E_trans = NULL;
            } else {
                if (_Etrans_temp_last != NULL) {
                    _Etrans_temp_last->next_E_trans = _Etrans_temp;
                }
            }
            M_E_trans(_mat, _Etrans_temp, _COLUMN_);
            _Etrans_temp_last = _Etrans_temp;
            if (flag == 2) {
                i = i - 1;
                break;
            }
        }
    }
    M_inv_struct *_Lowtri = (M_inv_struct *) OS_Malloc(sizeof(M_inv_struct));
    _Lowtri->_matrix = _mat;
    _Lowtri->_Etrans_head = _Etrans_temp_last;
    return _Lowtri;
}

Matrix *M_Dia_Inv(Matrix *_mat_source) {/*
 * The inverse of the diagonal Matrix (create).
 * ??????*/
    Matrix *_mat_inv = NULL;
    if (_mat_source->row != _mat_source->column) {
        OS_Err_Collector(M_Dia_Inv_002);
        OS_Commander("pause");
    } else {
        _mat_inv = Matrix_copy(_mat_source);
        MATRIX_TYPE *data = _mat_inv->data;
        int i, order = _mat_source->column;
        for (i = 0; i < order; i++) {
            if((data)[i * (order + 1)] == 0){ // ???
                OS_Err_Collector(M_Dia_Inv_023);
                OS_Commander("pause");
                (data)[i * (order + 1)] = 1 / (data[i * (order + 1)]);
            }else{
                (data)[i * (order + 1)] = 1 / (data[i * (order + 1)]);
            }
        }
    }
    return _mat_inv;
}

Dia_struct *M_Diatri_(Matrix *_mat_source) {/*
 * Diagonalization (create).
 * ???*/
    Dia_struct *_Dia_instance = (Dia_struct *) OS_Malloc(sizeof(Dia_struct));
    Uptri_struct *_Uptri_ = M_Uptri_(_mat_source);
    Lowtri_struct *_Lowtri_ = M_Lowtri_(_Uptri_->Uptri_matrix);
    _Dia_instance->trans_leftmatrix = _Uptri_->trans_matrix;
    _Dia_instance->Diatri_matrix = _Lowtri_->Lowtri_matrix;
    _Dia_instance->trans_rightmatrix = _Lowtri_->trans_matrix;
    (_DETAILED_>=1)?OS_Err_Collector(">>Diag(Matrix_%x)=\n", _mat_source):0;
    (_DETAILED_>=1)?OS_Err_Collector("\tMatrixdl%x * Matrix_d%x * Matrixr_%x\n", _Uptri_->trans_matrix,
                        _Lowtri_->Lowtri_matrix, _Lowtri_->trans_matrix):0;
    // ????
    M_Free(_Uptri_->Uptri_matrix);
    OS_Free(_Uptri_);
    OS_Free(_Lowtri_);
    return _Dia_instance;
}

Matrix *M_Inverse(Matrix *_mat) {/*
 * Inverse (create).
 * ????*/
    M_inv_struct *_Uptri_ = M_Uptri_4inv(_mat);
    M_inv_struct *_Lowtri_ = M_Lowtri_4inv(_Uptri_->_matrix);
    Matrix *_mat_dia_inv = M_Dia_Inv(_Lowtri_->_matrix);
    Matrix *_mat_inv = Etrans_4_Inverse(_mat_dia_inv, _Lowtri_->_Etrans_head, _ROW_);
    _mat_inv = Etrans_4_Inverse(_mat_inv, _Uptri_->_Etrans_head, _COLUMN_);
    (_DETAILED_>=2)?OS_Err_Collector(">>Inv(Matrix_%x)=\n", _mat):0;
    (_DETAILED_>=2)?OS_Err_Collector("\tMatrix_inv_%x\n", _mat_inv):0;
    // ????
    M_Free(_Uptri_->_matrix);
    M_Free(_Lowtri_->_matrix);
    OS_Free(_Uptri_);
    OS_Free(_Lowtri_);
    return _mat_inv;
}

int M_Swap(Matrix *_mat, int _line_1, int _line_2, int line_setting) {/*
 * Swap row or cloumn of the Matrix.
 * ???????*/
    _line_1 = _line_1 - 1;
    _line_2 = _line_2 - 1;
    int i;
    MATRIX_TYPE temp;
    if (line_setting == _ROW_) {
        if ((_line_1 < _mat->row) && (_line_2 < _mat->row)) {
            for (i = 0; i < (_mat->column); i++) {
                temp = _mat->data[_line_1 * (_mat->column) + i];
                _mat->data[_line_1 * (_mat->column) + i] = _mat->data[_line_2 * (_mat->column) + i];
                _mat->data[_line_2 * (_mat->column) + i] = temp;
            }
        } else {
            OS_Err_Collector(M_swap_004);
            OS_Commander("pause");
        }
    } else {
        if ((_line_1 < _mat->column) && (_line_2 < _mat->column)) {
            for (i = 0; i < (_mat->row); i++) {
                temp = _mat->data[_line_1 + (_mat->column) * i];
                _mat->data[_line_1 + (_mat->column) * i] = _mat->data[_line_2 + (_mat->column) * i];
                _mat->data[_line_2 + (_mat->column) * i] = temp;
            }
        } else {
            OS_Err_Collector(M_swap_004);
            OS_Commander("pause");
        }
    }
    return 0;
}

Matrix *M_Cut(Matrix *_mat, int row_head, int row_tail, int column_head, int column_tail) {/*
 * Cut out a part-matrix from the Matrix (create).
 * ??????*/
    Matrix *mat_result = NULL;
    if (row_tail < 0) {
        if (row_tail == _END_) {
            row_tail = _mat->row;
        } else {
            OS_Err_Collector(M_Cut_007);
            OS_Commander("pause");
        }
    }
    if (row_head < 0) {
        if (row_head == _END_) {
            row_head = _mat->row;
        } else {
            OS_Err_Collector(M_Cut_007);
            OS_Commander("pause");
        }
    }
    if (column_tail < 0) {
        if (column_tail == _END_) {
            column_tail = _mat->column;
        } else {
            OS_Err_Collector(M_Cut_007);
            OS_Commander("pause");
        }
    }
    if (column_head < 0) {
        if (column_head == _END_) {
            column_head = _mat->column;
        } else {
            OS_Err_Collector(M_Cut_007);
            OS_Commander("pause");
        }
    }
    if ((row_tail > _mat->row) || (column_tail > _mat->column)) {
        OS_Err_Collector(M_Cut_005);
        OS_Commander("pause");
    } else {
        if ((row_head > row_tail) || (column_head > column_tail)) {
            OS_Err_Collector(M_Cut_006);
            OS_Commander("pause");
        } else {
            row_head = row_head - 1;
            column_head = column_head - 1;
            mat_result = (Matrix *) OS_Malloc(sizeof(Matrix));
            mat_result->row = row_tail - row_head;
            mat_result->column = column_tail - column_head;
            mat_result->data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * (mat_result->row) * (mat_result->column));
            int i, j;
            for (i = 0; i < (row_tail - row_head); i++) {
                // using memory-copy to speed up.
                memcpy(&(mat_result->data[i * (mat_result->column)]),
                       &(_mat->data[(i + row_head) * (_mat->column) + column_head])
                       , sizeof(MATRIX_TYPE)*(column_tail - column_head));
//                for (j = 0; j < (column_tail - column_head); j++) {
//                    mat_result->data[i * (mat_result->column) + j] = _mat->data[(i + row_head) * (_mat->column) +
//                                                                                (j + column_head)];
//                }
            }
        }
    }
    return mat_result;
}

Matrix *M_T(Matrix *_mat_source) {/*
 * Transpose (create).
 * ?? */
    Matrix *_mat = (Matrix *) OS_Malloc(sizeof(Matrix));
    _mat->column = _mat_source->row;
    _mat->row = _mat_source->column;
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * (_mat->column) * (_mat->row));
    _mat->data = data;
    int i, j;
    for (i = 0; i < (_mat->row); i++) {
        for (j = 0; j < _mat->column; j++) {
            data[i * (_mat->column) + j] = _mat_source->data[j * (_mat_source->column) + i];
        }
    }
    return _mat;
}

int M_Free(Matrix *_mat) {/*
 * Free the memory of the Matrix (create).
 * ????,????*/
    if (_mat == NULL){
        OS_Err_Collector("[WARING] NO SUCH MATRIX!\n");
        return 0;
    }
    OS_Free(_mat->data);
    (_DETAILED_>=3)?OS_Err_Collector(">>Matrix_%x has been freed.\n", _mat):0;
    OS_Free(_mat);
    return 0;
}

MATRIX_TYPE M_tr(Matrix *_mat) {/*
 * Trace of Matrix
 * ????*/
    MATRIX_TYPE _tr_mat = 0;
    if (_mat->column == _mat->row) {
        int i;
        for (i = 0; i < _mat->column; i++) {
            _tr_mat += _mat->data[i * (_mat->column + 1)];
        }
    } else {
        OS_Err_Collector(M_tr_008);
        OS_Commander("poause");
    }
    return _tr_mat;
}

MATRIX_TYPE M_det(Matrix *_mat_) {/*
 * Determinant.
 * ???*/
    MATRIX_TYPE _det_mat = 0;
    if (_mat_->column == _mat_->row) {
        Uptri_struct *_Uptri_ = M_Uptri_(_mat_);
        Matrix *_mat = _Uptri_->Uptri_matrix;
        _det_mat = 1;
        int i;
        for (i = 0; i < _mat->column; i++) {
            _det_mat *= _mat->data[i * (_mat->column + 1)];
        }
        // ????
        M_Free(_Uptri_->Uptri_matrix);
        M_Free(_Uptri_->trans_matrix);
        OS_Free(_Uptri_);
    } else {
        OS_Err_Collector(M_det_009);
        OS_Commander("poause");
    }
    return _det_mat;
}

Matrix *M_full(Matrix *_mat, int row_up, int row_down, int column_left, int column_right, MATRIX_TYPE full_data) {/*
 * Full the Matrix with data (create).
 * ????*/
    Matrix *mat_result = NULL;
    mat_result = (Matrix *) OS_Malloc(sizeof(Matrix));
    mat_result->row = (_mat->row + row_up + row_down);
    mat_result->column = (_mat->column + column_left + column_right);
    mat_result->data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * (mat_result->row) * (mat_result->column));
    int i, j;
    for (i = 0; i < mat_result->row; i++) {
        for (j = 0; j < mat_result->column; j++) {
            if ((i >= row_up) && (i < (row_up + _mat->row))) {/*??????,????*/
                if ((j >= column_left) && (j < (column_left + _mat->column))) {
                    // using memory-copy to speed up.
                    memcpy(&(mat_result->data[i * (mat_result->column)+j]),
                           &(_mat->data[(_mat->column) * (i - row_up) + (j - column_left)])
                    , sizeof(MATRIX_TYPE)*((column_left + _mat->column) - column_left));
                    j += ((column_left + _mat->column) - column_left);
//                    mat_result->data[i * (mat_result->column) + j] = _mat->data[(_mat->column) * (i - row_up) +
//                                                                                (j - column_left)];
                } else {
                    mat_result->data[i * (mat_result->column) + j] = full_data;
                }
            } else {
                mat_result->data[i * (mat_result->column) + j] = full_data;
            }
        }
    }
    (_DETAILED_>=2)?(OS_Err_Collector(">>Full<U%d,D%d,L%d,R%d>(Matrix_%x)=\n\tMatrix_%x\n",
                         row_up, row_down, column_left, column_right, _mat, mat_result)):0;
    return mat_result;
}

MATRIX_TYPE M_norm(Matrix *_mat, int Setting) {/*
 * Calculate Matrix's norm-value.
 * Norm (1/ 2/ p/ INF/ FRO).
 * ??/???? Setting in [1/ 2/ p/ INF/ FRO]
 * 1: L1??
 * 2:L2??
 * p: p??
 * INF: ????
 * FRO: FRO?? */
    MATRIX_TYPE *data = _mat->data;
    int row = _mat->row;
    int column = _mat->column;
    MATRIX_TYPE Val_norm = 0;
    int i, j;
    if (row == _ONE_ || column == _ONE_) {/*?????*/
        switch (Setting) {
            case 1: {/*???1??*/
                for (i = 0; i < row; i++) {
                    for (j = 0; j < column; j++) {
                        /*??abs()???,error C2668: “abs”: ???????????
						????fabs().*/
                        Val_norm += fabs(data[i * (column) + j]);
                    }
                }
                break;
            }
            case 2: {/*???2??*/
                for (i = 0; i < row; i++) {
                    for (j = 0; j < column; j++) {
                        Val_norm += data[i * (column) + j] * data[i * (column) + j];
                    }
                }
                Val_norm = pow(Val_norm, 0.5);
                break;
            }
            case INF: {/*???8(inf)????*/
                Matrix *M_temp_0, *M_temp_1;
                M_temp_0 = M_abs(_mat);
                M_temp_1 = M_max(M_temp_0);
                int temp_num = M_temp_1->data[0];
                Val_norm = (M_temp_0)->data[temp_num];
                // ????
                M_Free(M_temp_0);
                M_Free(M_temp_1);
                break;
            }
            default: {/*???p??*/
                for (i = 0; i < row; i++) {
                    for (j = 0; j < column; j++) {
                        Val_norm += pow(data[i * (column) + j], Setting);
                    }
                }
                if (Val_norm < 0) {
                    OS_Err_Collector(M_norm_warm_01);
                }
                Val_norm = pow(Val_norm, 1.0 / Setting);
                break;
            }
        }
    } else {
        /*????*/
        switch (Setting) {
            case 1: {/*???1??*/
                Matrix *M_temp_0, *M_temp_1, *M_temp_2;
                M_temp_0 = M_abs(_mat);
                M_temp_1 = M_sum(M_temp_0);
                M_temp_2 = M_max(M_temp_1);
                int temp_num = M_temp_2->data[0];
                Val_norm = (M_temp_1)->data[temp_num];
                M_Free(M_temp_0);
                M_Free(M_temp_1);
                M_Free(M_temp_2);
                break;
            }
            case 2: {/*???2??*/
                Matrix *M_temp_0, *M_temp_1;
                M_temp_0 = M_T(_mat);
                M_temp_1 = M_mul(M_temp_0, _mat);
                M_eigen_struct *M_temp_1_eigen = M_eigen_max(M_temp_1);
                Val_norm = M_temp_1_eigen->eigen_value;
                M_Free(M_temp_0);
                M_Free(M_temp_1);
                M_Free(M_temp_1_eigen->eigen_matrix);
                OS_Free(M_temp_1_eigen);
                break;
            }
            case INF: {/*???8(inf)????*/
                Matrix *M_temp_0, *M_temp_1, *M_temp_2, *M_temp_;
                M_temp_ = M_T(_mat);
                M_print(M_temp_);
                M_temp_0 = M_abs(M_temp_);
                M_print(M_temp_0);
                M_temp_1 = M_sum(M_temp_0);
                M_print(M_temp_1);
                M_temp_2 = M_max(M_temp_1);
                M_print(M_temp_2);
                int temp_num = M_temp_2->data[0];
                Val_norm = (M_temp_1)->data[temp_num];
                M_Free(M_temp_);
                M_Free(M_temp_0);
                M_Free(M_temp_1);
                M_Free(M_temp_2);
                break;
            }
            case FRO: {/*???F??(Frobenius??)*/
                for (i = 0; i < row; i++) {
                    for (j = 0; j < column; j++) {
                        Val_norm += data[i * (column) + j] * data[i * (column) + j];
                    }
                }
                Val_norm = pow(Val_norm, 0.5);
                break;
            }
            default: {
                OS_Err_Collector(M_norm_022);
                OS_Commander("pause");
                break;
            }
        }
    }
    return Val_norm;
}

Matrix *M_abs(Matrix *_mat_origin) {/*
 * Absolute the value of elements in the Matrix (create).
 * ??????????*/
    Matrix *_mat = (Matrix *) OS_Malloc(sizeof(Matrix));
    _mat->row = _mat_origin->row;
    _mat->column = _mat_origin->column;
    int size = _mat->row * _mat->column;
    _mat->data = (MATRIX_TYPE *) OS_Malloc((size) * sizeof(MATRIX_TYPE));
    int i;
    for (i = 0; i < size; i++) {
        _mat->data[i] = fabs(_mat_origin->data[i]);
    }

    return _mat;
}

Matrix *M_numul(Matrix *_mat, MATRIX_TYPE _num) {/*
 * Number Multiplication (create).
 * ????*/
    MATRIX_TYPE *data = _mat->data;
    int Size_mat = (_mat->row) * (_mat->column), i;
    for (i = 0; i < Size_mat; i++) {
        data[i] = data[i] * _num;
    }
    return _mat;
}

Matrix *M_matFull(Matrix *_mat, int row_up, int column_left, Matrix *_mat_full) {/*
 * Full the Matrix with another Matrix.
 * ????????
 * [?] ???,????,row_up?column_left???_HEAD_(1)*/
    MATRIX_TYPE *data = _mat->data;
    row_up--;
    column_left--;
    int row = _mat->row;
    int column = _mat->column;
    int i, j;
    if ((row_up + _mat_full->row) > row) {
        OS_Err_Collector(M_Mfull_010);
        OS_Commander("pause");
    }
    if ((column_left + _mat_full->column) > column) {
        OS_Err_Collector(M_Mfull_011);
        OS_Commander("pause");
    }
    int full_row = _mat_full->row, full_column = _mat_full->column;
    for (i = 0; i < full_row; i++) {
        // using memory-copy to speed up.
        memcpy(&(data[(row_up + i) * column + (column_left)]),
               &(_mat_full->data[i * full_column])
        , sizeof(MATRIX_TYPE)*(full_column));
//        for (j = 0; j < full_column; j++) {
//            data[(row_up + i) * column + (column_left + j)] = _mat_full->data[i * full_column + j];
//        }
    }
    return _mat;
}

Matrix *M_Zeros(int row, int column) {/*
 * Generation All-Zeros-Matrix (create).
 * ??????*/
    Matrix *Zero_mat = (Matrix *) OS_Malloc(sizeof(Matrix));
    Zero_mat->column = column;
    Zero_mat->row = row;
    int size = row * column;
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc((size) * sizeof(MATRIX_TYPE));
    // using memory-set to speed up.
    memset(data,0,sizeof(MATRIX_TYPE)*size);
//    int i;
//    for (i = 0; i < size; i++) {
//        data[i] = 0;
//    }
    Zero_mat->data = data;
    return Zero_mat;
}

Matrix *M_Ones(int row, int column) {/*
 * Generation All-Ones-Matrix (create).
 * ??????*/
    Matrix *Zero_mat = (Matrix *) OS_Malloc(sizeof(Matrix));
    Zero_mat->row = row;
    Zero_mat->column = column;
    int size = row * column, i;
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc((size) * sizeof(MATRIX_TYPE));
    for (i = 0; i < size; i++) {
        data[i] = 1;
    }
    Zero_mat->data = data;
    return Zero_mat;
}

Matrix *M_find(Matrix *_mat, MATRIX_TYPE value) {/*
 * Find all the positions with a certain value in the Matrix (create).
 * ??????????;??????,???????;
 * Matrix????,?????????);
 * e.g.
 * [code]
    // define mat_1
        MATRIX_TYPE _mat_1[2][2] = { 1,0,0,1 };
        int row = sizeof(_mat_1) / sizeof(_mat_1[0]);
        int column = sizeof(_mat_1[0]) / sizeof(_mat_1[0][0]);
        Matrix*  mat_1 = Matrix_gen(row,column,(double *)_mat_1);
    // find mat_1
        M_print(M_find(mat_1,1));
    [output]
    >>Matrix_b380c0:
    0.00
    3.00
    [explain]
    ??? mat_1 { 1,0,0,1 }??0??3???,??1?
 */
    int size_mat = (_mat->row) * (_mat->column);
    int *position = (int *) OS_Malloc(sizeof(int) * size_mat);
    int num = 0, temp_column, temp_row, i;
    for (i = 0; i < size_mat; i++) {
        if (_mat->data[i] == value) {
            position[num] = i;
            num = num + 1;
        }
    }
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * num);
    for (i = 0; i < num; i++) {
        temp_column = position[i] % (_mat->column);
        temp_row = (position[i] - temp_column) / (_mat->column);
        data[i] = temp_column * (_mat->row) + temp_row;
    }
    Matrix *mat_result = (Matrix *) OS_Malloc(sizeof(Matrix));
    mat_result->row = num;
    mat_result->column = 1;
    mat_result->data = data;
    OS_Free(position);
    return mat_result;
};

Matrix *M_sum(Matrix *_mat) {/*
 * Matrix Column-Summation (create). / Vector element Sum (create) .
 * ??????/?????*/
    Matrix *mat_result = (Matrix *) OS_Malloc(sizeof(Matrix));
    int row = _mat->row, column = _mat->column;
    int i, j, size = row * column;
    if (row == _ONE_ || column == _ONE_) {
        MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE));
        data[0] = 0;
        for (i = 0; i < (size); i++) {
            data[0] = data[0] + _mat->data[i];
        }
        mat_result->row = 1;
        mat_result->column = 1;
        mat_result->data = data;
    } else {
        MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * column);
        for (i = 0; i < (column); i++) {
            data[i] = 0;
            for (j = 0; j < (row); j++) {
                data[i] = data[i] + _mat->data[j * column + i];
            }
        }
        mat_result->row = 1;
        mat_result->column = column;
        mat_result->data = data;
    }
    return mat_result;
}

Matrix *M_min(Matrix *_mat) {/*
 * Minimum-value position for each row in the Matrix (create) . / Vector minimum element position (create).
 * ?????????/????????*/
    Matrix *mat_result = (Matrix *) OS_Malloc(sizeof(Matrix));
    int row = _mat->row, column = _mat->column;
    int i, j, size = row * column;
    if (row == _ONE_ || column == _ONE_) {
        MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE));
        data[0] = Min_position(_mat->data, size);
        mat_result->row = 1;
        mat_result->column = 1;
        mat_result->data = data;
    } else {
        MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * column);
        MATRIX_TYPE *temp_data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * row);
        for (i = 0; i < (column); i++) {
            for (j = 0; j < (row); j++) {
                temp_data[j] = _mat->data[j * column + i];
            }
            data[i] = Min_position(temp_data, row);
        }
        mat_result->row = 1;
        mat_result->column = column;
        mat_result->data = data;
    }
    return mat_result;
}

int Min_position(MATRIX_TYPE *data, int size) {/*
 * Find min-position in a MATRIX_TYPE[*]
 * ?MATRIX_TYPE[*]???,???????(????????)*/
    MATRIX_TYPE Val_min = data[size - 1];
    int min_position = size - 1, i;
    for (i = size - 2; i >= 0; i--) {
        if (data[i] <= Val_min) {
            Val_min = data[i];
            min_position = i;
        }
    }
    return min_position;
}

Matrix *M_max(Matrix *_mat) {/*
 * Maximum-value position for each row in the Matrix (create)./ Vector Maximum element position (create).
 * ?????????/????????*/
    Matrix *_mat_ = Matrix_copy(_mat);
    _mat_ = M_numul(_mat_, -1);
    Matrix *mat_result = M_min(_mat_);
    M_Free(_mat_);
    return mat_result;
}

Matrix *M_minax_val(Matrix *_mat, Matrix *_mat_position) {/*
 * The value of those given (row) positions for each column in the matrix (create).
 * ???????????*/
    Matrix *mat_val = (Matrix *) OS_Malloc(sizeof(Matrix));
    mat_val->row = _mat_position->row;
    mat_val->column = _mat_position->column;
    int i, temp, size_mat = (_mat_position->row) * (_mat_position->column);
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * size_mat);
    for (i = 0; i < size_mat; i++) {
        temp = (_mat_position->data[i]);
        data[i] = _mat->data[temp * (_mat->column) + i];
    }
    mat_val->data = data;
    return mat_val;
}

Matrix *M_logic_equal(Matrix *_mat, MATRIX_TYPE value) {/*
 * Compare every element /pisition of the Matrix with certain value (create).
 * [ More : Return a new Matrix, whose every value is 0/1. ]
 * ???????????,(????,??0/1) */
    int size_mat = (_mat->row) * (_mat->column), i;
    Matrix *mat_logic = (Matrix *) OS_Malloc(sizeof(Matrix));
    mat_logic->row = (_mat->row);
    mat_logic->column = (_mat->column);
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * size_mat);
    for (i = 0; i < size_mat; i++) {
        if (_mat->data[i] == value) {
            data[i] = 1;
        } else {
            data[i] = 0;
        }
    }
    mat_logic->data = data;
    return mat_logic;
}

Matrix *M_logic(Matrix *_mat_left, Matrix *_mat_right, int Operation) {/*
 * Logical operation of corresponding positions of two matrices.
 * ??????????? */
    Matrix *mat_logic = (Matrix *) OS_Malloc(sizeof(Matrix));
    if (_mat_right) {
        if (_mat_left->row != _mat_right->row) {
            OS_Err_Collector(M_logic_012);
            OS_Commander("pause");
        }
        if (_mat_left->column != _mat_right->column) {
            OS_Err_Collector(M_logic_013);
            OS_Commander("pause");
        }
    }
    int size_mat = (_mat_left->row) * (_mat_left->column), i;
    mat_logic->row = (_mat_left->row);
    mat_logic->column = (_mat_left->column);
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * size_mat);
    switch (Operation) {
        case _AND_:
            for (i = 0; i < size_mat; i++) {
                if ((_mat_left->data[i] == 0) || (_mat_right->data[i] == 0)) {
                    data[i] = 0;
                } else {
                    data[i] = 1;
                }
            }
            break;
        case _OR_:
            for (i = 0; i < size_mat; i++) {
                if ((_mat_left->data[i] != 0) || (_mat_right->data[i] != 0)) {
                    data[i] = 1;
                } else {
                    data[i] = 0;
                }
            }
            break;
        case _NOT_:
            for (i = 0; i < size_mat; i++) {
                if (_mat_left->data[i] == 0) {
                    data[i] = 1;
                } else {
                    data[i] = 0;
                }
            }
            break;
        default:
            OS_Err_Collector(M_logic_014);
    }
    mat_logic->data = data;
    return mat_logic;
}

Matrix *M_pmuldiv(Matrix *_mat_left, Matrix *_mat_right, int operation) {/*
 * Hadamard Product : Multiply / Divide every element in the two Matrix-s (create).
 * (Point Mul and Div)
 * ????, ????/?? */
    Matrix *mat_result = (Matrix *) OS_Malloc(sizeof(Matrix));
    if (_mat_left->row != _mat_right->row) {
        OS_Err_Collector(M_pmuldiv_015);
        OS_Commander("pause");
    }
    if (_mat_left->column != _mat_right->column) {
        OS_Err_Collector(M_pmuldiv_016);
        OS_Commander("pause");
    }
    int i, size_mat = (_mat_left->row) * (_mat_left->column);
    mat_result->row = (_mat_left->row);
    mat_result->column = (_mat_left->column);
    MATRIX_TYPE *data = (MATRIX_TYPE *) OS_Malloc(sizeof(MATRIX_TYPE) * size_mat);
    if (operation == _MUL_) {
        for (i = 0; i < size_mat; i++) {
            data[i] = (_mat_left->data[i]) * (_mat_right->data[i]);
        }
    } else {
        if (operation == _DIV_) {
            for (i = 0; i < size_mat; i++) {
                if (_mat_right->data[i] != 0) {
                    data[i] = (_mat_left->data[i]) / (_mat_right->data[i]);
                } else {
                    data[i] = _INFINITE_;
                }

            }
        } else {
            OS_Err_Collector(M_pmuldiv_017);
        }
    }
    mat_result->data = data;
    return mat_result;
}

Matrix *M_setval(Matrix *_mat_ini, Matrix *_mat_val, Matrix *_mat_order, int model) {/*
 * Setting Values of a Matrix with another Matrix.
 * ????????,?????????
 * */
    int i, temp, size_ini = (_mat_ini->column) * (_mat_ini->row);
    int size_val = (_mat_val->column) * (_mat_val->row);
    int size_order = (_mat_order->column) * (_mat_order->row);
    if (model == _ORD4INI_) {/*_ORD4INI_*/
        for (i = 0; i < size_order; i++) {
            if ((_mat_order->data[i]) < size_ini) {
                if (i < size_val) {
                    temp = (_mat_order->data[i]);
                    _mat_ini->data[temp] = _mat_val->data[i];
                } else {
                    OS_Err_Collector(M_setval_019);
                    OS_Commander("pause");
                }
            } else {
                OS_Err_Collector(M_setval_018);
                OS_Commander("pause");
            }
        }
    } else {/*_ORD4VAL_*/
        for (i = 0; i < size_ini; i++) {
            if ((i) < size_order) {
                temp = (_mat_order->data[i]);
                if (temp < size_val) {
                    _mat_ini->data[i] = _mat_val->data[temp];
                } else {
                    OS_Err_Collector(M_setval_019);
                    OS_Commander("pause");
                }
            } else {
                OS_Err_Collector(M_setval_020);
                OS_Commander("pause");
            }
        }
    }

    return _mat_ini;
}

Matrix *M_numul_m(Matrix *_mat, Matrix *_mat_multi) {/*
 * Matrix Number Multiplication (using matrix transfer)
 * _mat_result = _mat_left*_mat_right
 * ?????,??????? */
    MATRIX_TYPE *data = _mat->data;
    int Size_mat = (_mat->row) * (_mat->column), i, j, temp;
    int row = _mat->row;
    int column = _mat->column;
    MATRIX_TYPE Multi;
    for (i = 0; i < row; i++) {
        Multi = _mat_multi->data[i];
        for (j = 0; j < column; j++) {
            temp = i * column + j;
            data[temp] = data[temp] * Multi;
        }
    }
    return _mat;
}

// ??: M_eigen_struct_temp -> M_eigen_struct
M_eigen_struct *M_eigen_max(Matrix *_mat) {/*
 * The maximum eigenvalue/ eigen-vector of the Matrix (create).
 * ?????????(??)
 * _mat_result = Max_eigenvalue(_mat)
 * Reference: [??-????](https://max.book118.com/html/2017/0527/109650252.shtm) */
    M_eigen_struct *M_eigen_max = NULL;
    if (_mat->column == _mat->row) {
        Matrix *mat_z = M_Ones(_mat->column, 1), *mat_temp_1 = NULL, *mat_temp_2 = NULL;
        Matrix *mat_y = NULL, *mat_z_gap = NULL;
        MATRIX_TYPE m_value = 0, mat_z_gap_norm = 1;
        MATRIX_TYPE deta = 1e-7; //????
        int temp_num = 0;

        while (mat_z_gap_norm > deta) {
            mat_y = M_mul(_mat, mat_z);
            mat_temp_1 = M_max(mat_y);//????????
            temp_num = ((mat_temp_1)->data[0]);
            m_value = mat_y->data[temp_num];
            mat_temp_2 = mat_z;//????????
            mat_z = M_numul(mat_y, 1 / m_value);
            mat_z_gap = M_add_sub(1, mat_z, 1, mat_temp_2);//????????
            mat_z_gap_norm = M_norm(mat_z_gap, 2);
            // ????
            M_Free(mat_temp_1);
            M_Free(mat_temp_2);
            M_Free(mat_z_gap);
            M_Free(mat_y);
        }
        M_eigen_max = (M_eigen_struct *) OS_Malloc(sizeof(M_eigen_struct));
        M_eigen_max->eigen_value = m_value;
        M_eigen_max->eigen_matrix = mat_z;
    } else {
        OS_Err_Collector(M_eigen_max_021);
        OS_Commander("pause");
    }
    return M_eigen_max;
}

Matrix ** M_eigen (Matrix *_mat) {/*
 * The eigenvalues and eigen-vectors of the Matrix (create).
 * ???????+????(??)
 * _mat_result = Max_eigenvalue(_mat)
 * Reference:
 * 1. [??-????](https://max.book118.com/html/2017/0527/109650252.shtm)
 * 2. [??-??????](https://blog.csdn.net/w_weixiaotao/article/details/111868086) */
    Matrix ** M_array_eigen_vec = NULL;
    if (_mat->column == _mat->row) {
        M_array_eigen_vec = (Matrix **)OS_Malloc(sizeof(Matrix *)*2); // ??Q/R????
        enum{val=0, vec=1};
        Matrix *eigen_value = M_eigen_val(_mat);
        M_array_eigen_vec[val] = eigen_value;
        int eigen_count, dim = _mat->column, i, j, k, ik, jk;
        Matrix *eigen_vector = NULL, *_mat_ = NULL;
        eigen_vector = M_Zeros(dim,dim);// ??????
        M_array_eigen_vec[vec] = eigen_vector;
        MATRIX_TYPE eigen_value_temp, swap_value_temp;
        MATRIX_TYPE coe; // core of elements, ?????/????
        for(eigen_count=0;eigen_count<dim;eigen_count++){
            _mat_ = Matrix_copy(_mat);
            eigen_value_temp = eigen_value->data[eigen_count];
            // (A-lamda*I)
            for (i = 0; i < dim; i++){
                _mat_->data[i * _mat_->column + i] -= eigen_value_temp; // ??: ???? (A-lamda*I), ?A?I/diag?,??????;
            }
            // ?????????(???): ?????1
            for (i = 0; i < dim-1; i++){
                // Note: ?? 645770225, ????[issue-10](https://github.com/Amoiensis/Matrix_hub/issues/10)
                coe = _mat_->data[i * dim + i];
                k = i;
                for (j = i + 1; j < dim; j++){//????
                    if (fabs(_mat_->data[j * dim + i]) > fabs(coe)){
                        coe = _mat_->data[j * dim + i];
                        k = j;
                    }
                }
                if (fabs(coe) < _APPROXIMATELY_ZERO_){
                    // ?????0
                    continue;
                }
                if (k != i){
                    //?????????
                    for (j = 0; j < dim; j++)
                    {
                        swap_value_temp = _mat_->data[i * dim + j];
                        _mat_->data[i * dim + j] = _mat_->data[k * dim + j];
                        _mat_->data[k * dim + j] = swap_value_temp;
                    }
                }

                coe = _mat_->data[i * dim + i];
                for (j = i; j<dim; j++){
                    _mat_->data[i * dim + j] /= coe; //?????????
                }
                for (ik = i + 1; ik < dim; ik++){
                    coe = _mat_->data[ik * dim + i];
                    for (jk = i; jk < dim; jk++){
                        _mat_->data[ik * dim + jk] -= coe * _mat_->data[i * dim + jk];
                    }
                }
            }
            // ??????1
            MATRIX_TYPE sum1 = 1;
            // Note: ?? 645770225, ????[issue-10](https://github.com/Amoiensis/Matrix_hub/issues/10)
            if (abs(_mat_->data[(dim - 1) * dim + (dim - 1)]) > _APPROXIMATELY_ZERO_){
                // ??????????0
                sum1 = 0;
                eigen_vector->data[(dim - 1) * dim + eigen_count] = 0.0f;
            }else{
                sum1 = 1;
                eigen_vector->data[(dim - 1) * dim + eigen_count] = 1;
            }

            eigen_vector->data[(dim - 1) * dim + eigen_count] = 1;
            for (ik = dim - 2; ik >= 0; ik--){
                MATRIX_TYPE sum2 = 0;
                // Note: ?? 645770225, ????[issue-10](https://github.com/Amoiensis/Matrix_hub/issues/10)
                for (jk = ik + 1; jk < dim; jk++){
                    sum2 += _mat_->data[ik * dim + jk] * eigen_vector->data[jk * dim + eigen_count];
                }
                if (fabs(_mat_->data[ik * dim + ik]) > _APPROXIMATELY_ZERO_){
                    // ?????????0
                    sum2 = -sum2 / _mat_->data[ik * dim + ik];
                }else{
                    sum2 = 1;
                }
                sum1 += sum2 * sum2;
                eigen_vector->data[ik * dim + eigen_count] = sum2;
            }
            M_Free(_mat_);
            sum1 = sqrt(sum1);//?????
            for (i = 0; i < dim; i++){
                // ?????
                eigen_vector->data[i * dim + eigen_count] /= sum1;
            }
        }
    }else{
        OS_Err_Collector(M_eigen_026);
        OS_Commander("pause");
    }
    return M_array_eigen_vec;
}

Matrix* householder(Matrix * _x) {/*
 * Householder transformation for the Vector, return Transformating-Matrix: H (create).
 * Householder??
 * Referemce:
 * 1. [householder??](https://blog.csdn.net/qq_40922398/article/details/112788453)
 */
    Matrix *H = NULL;

//    // ???? 645770225 ???????, ??? [issue-11](https://github.com/Amoiensis/Matrix_hub/issues/11).
//    MATRIX_TYPE x_norm = M_norm(_x, 2);
//    if (x_norm < _APPROXIMATELY_ZERO_){
//        H = M_I(_x->row);
//        OS_Commander("pause");
//        OS_Err_Collector(householder_warm_02);
//        goto END_H;
//    }

    Matrix *y = M_Zeros(_x->row,_x->column);
    y->data[0] = M_norm(_x, 2);
    Matrix *w = NULL;
    if(_x->data[0] > 0){
        w = M_add_sub(1,_x,-1,y);
        M_numul(w,1/M_norm(w, 2));
    }else if (fabs(_x->data[0]) <= _APPROXIMATELY_ZERO_){
        w = _x;
        w = M_numul(w, 1 / M_norm(w, 2));
    }else{
        w = M_add_sub(1,_x,1,y);
        M_numul(w,1/M_norm(w, 2));
    }
    Matrix *I= M_I(_x->row);
    Matrix *w_T = M_T(w);
    Matrix *M_dot = M_mul(w,w_T);
    H = M_add_sub(1,I,2,M_dot);
    M_Free(y);
    M_Free(w);
    M_Free(I);
    M_Free(w_T);
    M_Free(M_dot);
    END_H:;
    return H;
}

Matrix * M_householder(Matrix * _mat) {/*
 * Householder transformation for the Matrix, return Transformated-Matrix: H_Mat (create).
 * Householder??, ??????.
 * */
    // translate to uptri-Hessenberg Matrix.
    Matrix *h_Mat = NULL;
    if (_mat->column == _mat->row) {
        int i,j,k, dim = _mat->column;
        Matrix * Ri = Matrix_copy(_mat);
        Matrix * temp = NULL;
        Matrix *Q = NULL, *Qi = NULL, *Hi = NULL;
        for(i=1;i<dim;i++){
            // ??"_mat"???Ri; ??645770225, issue: https://github.com/Amoiensis/Matrix_hub/issues/8;
            Matrix* x = M_Cut(Ri,i+1,_END_,i,i);
            // householder ????
            Hi = householder(x);
            M_Free(x);
            Qi = M_I(dim);
            for(j=0;j<dim-i;j++){ // Qi[i:, i:] = Hi
                for(k=0;k<dim-i;k++){
                    Qi->data[(j+i)*dim+(k+i)] = Hi->data[j*(dim-i)+k];
                }
            }
            M_Free(Hi);
            if (i == 1){
                Q = Matrix_copy(Qi);
            }else{
                temp = Q;
                Q = M_mul(Qi,temp);
                M_Free(temp);
            }
            temp = Ri;
            Ri = M_mul(Qi,Ri);
            M_Free(temp);
            temp = Ri;
            Ri = M_mul(Ri,Qi);
            M_Free(temp);
            M_Free(Qi);
        }
        h_Mat = Ri;
        M_Free(Q); // ???? Q
    }else{
        OS_Err_Collector(M_householder_027); // ?????
        OS_Commander("pause");
    }
    return h_Mat;
}

Matrix ** M_QR(Matrix * _mat){/*
 * QR Decomposition (create).
 * ??QR??.
 * */
    Matrix ** M_array_Q_R = (Matrix **)OS_Malloc(sizeof(Matrix *)*2); // ??Q/R????
    enum{q=0, r=1};
    M_array_Q_R[q] = NULL;
    M_array_Q_R[r] = NULL;
    int i, j, k, dim = _mat->row;
    Matrix *Q=NULL, *D=NULL, *Qi=NULL, *Hi=NULL, *x=NULL, *temp_1=NULL, *temp_2=NULL;
    Matrix * Ri = Matrix_copy(_mat); // ??
    for(i=0;i<dim;i++){
        // ??"_mat"???Ri; ??645770225, issue: https://github.com/Amoiensis/Matrix_hub/issues/8;
        x = M_Cut(Ri,i+1,_END_,i+1,i+1);
//        x = M_Cut(_mat,i+1,_END_,i+1,i+1);
        Hi = householder(x);
        M_Free(x);
        // Ri[i:, i:] = np.dot(Hi, Ri[i:, i:])
        temp_1 = M_Cut(Ri,i+1,_END_,i+1,_END_);
        temp_2 = M_mul(Hi,temp_1);
        M_Free(temp_1);
        for(j=0;j<dim-i;j++){
            for(k=0;k<dim-i;k++){
                Ri->data[(j+i)*dim+(k+i)] = temp_2->data[j*(dim-i)+k];
            }
        }
        M_Free(temp_2);
        Qi = M_I(dim);
        for(j=0;j<dim-i;j++){ // Qi[i:, i:] = Hi
            for(k=0;k<dim-i;k++){
                Qi->data[(j+i)*dim+(k+i)] = Hi->data[j*(dim-i)+k];
            }
        }
        M_Free(Hi);
        if (i == 0){
            Q = Matrix_copy(Qi);
        }else{
            temp_1 = Q;
            Q = M_mul(Qi,temp_1);
            M_Free(temp_1);
        }
        M_Free(Qi);
    }
    D = M_I(dim);
    for(i=0;i<dim;i++){
        // ??"D->data"????; ?? wtyhainan, issue: https://github.com/Amoiensis/Matrix_hub/issues/9;
//        D->data[i] = (Ri->data[i*dim+i] < 0)?-1:1;
        D->data[i*dim + i] = (Ri->data[i*dim+i]<0)?-1:1;
    }
    M_array_Q_R[r] = M_mul(D,Ri);
    temp_1 = M_T(Q);
    temp_2 = M_Dia_Inv(D);
    M_array_Q_R[q] = M_mul(temp_1, temp_2);
    M_Free(temp_1);
    M_Free(temp_2);
    M_Free(Ri);
    M_Free(D);
    M_Free(Q);
    return M_array_Q_R;
}

Matrix * M_eigen_val(Matrix * _mat){/*
 * The eigenvalues of the Matrix (create).
 * ????????(??).
 */
    (_DETAILED_>=2)?OS_Err_Collector(">>Eigen_Value(Matrix_%x)=\n", _mat):0;
    (_DETAILED_>=2)?OS_Err_Collector("...CACULATING...\n#if need help: use \'help(\"M_eigen_val\")\'#\n"):0;
    double *eigen_val = NULL;
    Matrix ** M_array_Q_R = NULL; // ??Q/R????
    enum{q=0, r=1};
    double eps = 1e-5, delta = 1; // ??????
    int i, dim=_mat->row, epoch = 0;
    Matrix *Ak0, *Ak, *Qk, *Rk, *M_eigen_val;
    Ak = Matrix_copy(_mat);
    while((delta > eps)&&(epoch < _MAX_LOOP_NUM_)){
        M_array_Q_R = M_QR(Ak);
        Qk = M_array_Q_R[q];
        Rk = M_array_Q_R[r];
        Ak0 = Ak;
        Ak = M_mul(Rk, Qk);
        delta = 0;
        for(i=0;i<dim;i++){
            delta += fabs(Ak->data[i*dim+i]-Ak0->data[i*dim+i]);
        }
        M_Free(Ak0);
        M_Free(Qk);
        M_Free(Rk);
        OS_Free(M_array_Q_R);
        (_progress_bar_)?progress_bar(epoch,_MAX_LOOP_NUM_):0;
        epoch++;
    }
    if(epoch >= _MAX_LOOP_NUM_){
        OS_Err_Collector("\n>>ATTENTION: QR Decomposition end with delta = %.3e!(epoch=%d, eps=%.2e)\n", delta,_MAX_LOOP_NUM_,eps);
    }
    M_eigen_val = (Matrix*)OS_Malloc(sizeof(Matrix));
    M_eigen_val->column = dim;
    M_eigen_val->row = 1;
    eigen_val = (double*)OS_Malloc(sizeof(double)*dim);
    for(i=0;i<dim;i++){
        eigen_val[i] = Ak->data[i*dim+i];
    }
    M_eigen_val->data = eigen_val;
    M_Free(Ak);
    (_DETAILED_>=2)?OS_Err_Collector("...END...\n>>Eigen_Value = (Matrix_%x)\n", M_eigen_val):0;
    return M_eigen_val;
}

void progress_bar(int count, int total) {/*
 * show a progress_bar, with count/total
 * ???????
 */
    double num = (int)((1.0*count/total)*50);
    OS_Err_Collector("%% %.2f[", num*2);
    for(int i = 0; i < 50; i++){
        (i < num)?OS_Err_Collector(">"):OS_Err_Collector(" ");
    }
    OS_Err_Collector("]\r");
}

Matrix ** M_SVD(Matrix * _mat){/*
 * SVD Decomposition.
 * ?????SVD??.
 */
    OS_Err_Collector("enterd SVD\n");
    enum{U=0, Dia=1, V=2};
    Matrix ** mat_list_SVD = (Matrix **)OS_Malloc(sizeof(Matrix *)*3);
    OS_Err_Collector("mat_list_SVD malloced\n");

    Matrix * _mat_T = M_T(_mat);
    Matrix * _mat_T_mat = M_mul(_mat_T, _mat);
    Matrix ** M_eigen_val_vec_V = M_eigen(_mat_T_mat);
    enum{val=0, vec=1};
    mat_list_SVD[V] = M_eigen_val_vec_V[vec];
    int i, Dia_len = M_eigen_val_vec_V[val]->column;
    for (i=0;i<Dia_len;i++){
        M_eigen_val_vec_V[val]->data[i] = sqrt(M_eigen_val_vec_V[val]->data[i]);
    }
    mat_list_SVD[Dia] = M_eigen_val_vec_V[val];
    OS_Err_Collector("mat_list_SVD malloced again\n");

    M_Free(_mat_T_mat);
    OS_Err_Collector("_mat_T_mat freed\n");


    Matrix * _mat_mat_T = M_mul(_mat, _mat_T);
    Matrix ** M_eigen_val_vec_U = M_eigen(_mat_mat_T);
    mat_list_SVD[U] = Matrix_copy(M_eigen_val_vec_U[vec]);
    M_Free(_mat_mat_T);
    M_Free(_mat_T);
    M_Free(M_eigen_val_vec_U[val]);
    M_Free(M_eigen_val_vec_U[vec]);
    OS_Free(M_eigen_val_vec_U);
    return mat_list_SVD;
}

Matrix * M_pinv(Matrix * _mat, int _method_){/*
 * left and right inverses / pseudo-inverse of Matrix.
 * Left: M_pinv(mat_A, _INV_L_);
 * Right: M_pinv(mat_A, _INV_R_);
 * pseudo-inverse: M_pinv(mat_A, _method_ in [_INV_L_, _INV_R_, _SVD_]);
 * ???? ?/?????, ????: 1. ????([_INV_L_, _INV_R_]); 2. SVD ??([_SVD_])
 * */
    Matrix * _mat_pinv = NULL, *mat_T = NULL,
    * mat_mat_T = NULL, * mat_T_mat = NULL,
    * inv_mat_T_mat = NULL, *inv_mat_mat_T = NULL,
    * mat_svd_inv_dia_T = NULL, * mat_temp_1 = NULL, * mat_temp_2 = NULL;
    Matrix **mat_list_SVD = NULL;
    MATRIX_TYPE val_temp;
    int i;
    switch (_method_) {
        case _INV_L_:
            mat_T = M_T(_mat);
            mat_T_mat = M_mul(mat_T, _mat);
            inv_mat_T_mat = M_Inverse(mat_T_mat);
            M_Free(mat_T_mat);
            _mat_pinv = M_mul(inv_mat_T_mat, mat_T);
            M_Free(mat_T);
            M_Free(inv_mat_T_mat);
            break;
        case _INV_R_:
            mat_T = M_T(_mat);
            mat_mat_T = M_mul(_mat, mat_T);
            inv_mat_mat_T = M_Inverse(mat_mat_T);
            M_Free(mat_mat_T);
            _mat_pinv = M_mul(mat_T, inv_mat_mat_T);
            M_Free(mat_T);
            M_Free(inv_mat_mat_T);
            break;
        case _SVD_:
            mat_list_SVD = M_SVD(_mat);
            mat_svd_inv_dia_T = M_Zeros(_mat->column, _mat->row);
            enum{U=0, Dia=1, V=2};
            for(i=0;i<_mat->row;i++){
                val_temp = mat_list_SVD[Dia]->data[i];
                if (fabs(val_temp) > _APPROXIMATELY_ZERO_){
                    mat_svd_inv_dia_T->data[i*(mat_svd_inv_dia_T->column)+i] = 1/(val_temp);
                }else{
                    mat_svd_inv_dia_T->data[i*(mat_svd_inv_dia_T->column)+i] = 1/(val_temp+_APPROXIMATELY_ZERO_);
                }
            }
            mat_temp_2 = M_mul(mat_list_SVD[V], mat_svd_inv_dia_T);
            M_Free(mat_svd_inv_dia_T);
            M_Free(mat_list_SVD[V]);
            mat_temp_1 = M_T(mat_list_SVD[U]);
            M_Free(mat_list_SVD[U]);
            _mat_pinv = M_mul(mat_temp_2, mat_temp_1);
            M_Free(mat_temp_1);
            M_Free(mat_temp_2);
            OS_Free(mat_list_SVD);
            break;
        default:
            OS_Err_Collector(M_pinv_028);
            OS_Commander("pause");
    }
    return _mat_pinv;
}
