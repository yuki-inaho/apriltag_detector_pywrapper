import pdb
from opencv_mat cimport *
from libc.string cimport memcpy
import numpy as np
cimport numpy as np  # for np.ndarray

# inspired and adapted from http://makerwannabe.blogspot.ch/2013/09/calling-opencv-functions-via-cython.html
# https://github.com/solivr/cython_opencvMat
cdef Mat np2Mat3D(np.ndarray ary):
    assert ary.ndim == 3 and ary.shape[2] == 3, "ASSERT::3channel RGB only!!"
    ary = np.dstack((ary[..., 2], ary[..., 1], ary[..., 0]))  # RGB -> BGR

    cdef np.ndarray[np.uint8_t, ndim= 3, mode = 'c'] np_buff = np.ascontiguousarray(ary, dtype=np.uint8)
    cdef unsigned int * im_buff = <unsigned int*> np_buff.data
    cdef int r = ary.shape[0]
    cdef int c = ary.shape[1]
    cdef Mat m
    m.create(r, c, CV_8UC3)
    memcpy(m.data, im_buff, r*c*3)
    return m

cdef Mat np2Mat2D(np.ndarray ary):
    assert ary.ndim == 2, "ASSERT::1 channel grayscale only!!"

    cdef np.ndarray[np.uint8_t, ndim= 2, mode = 'c'] np_buff = np.ascontiguousarray(ary, dtype=np.uint8)
    cdef unsigned int * im_buff = <unsigned int*> np_buff.data
    cdef int r = ary.shape[0]
    cdef int c = ary.shape[1]
    cdef Mat m
    m.create(r, c, CV_8UC1)
    memcpy(m.data, im_buff, r*c)
    return m

cdef Mat np2Mat2D_F32(np.ndarray ary):
    assert ary.ndim == 2, "ASSERT::1 channel grayscale only!!"
    assert ary.dtype == np.float32, "ASSERT dtype=float32"

    cdef np.ndarray[np.float32_t, ndim= 2, mode = 'c'] np_buff = np.ascontiguousarray(ary, dtype=np.float32)
    cdef float * im_buff = <float*> np_buff.data
    cdef int r = ary.shape[0]
    cdef int c = ary.shape[1]
    cdef Mat m
    m.create(r, c, CV_32FC1)
    memcpy(m.data, im_buff, r*c*sizeof(float))  # 4 is the size of
    return m


def npto32ftonp(nparr):
    assert nparr.dtype == np.float32, "array dtype must be float32"
    return Mat2np(np2Mat2D_F32(nparr))


cdef Mat np2Mat(np.ndarray ary):
    cdef Mat out
    if ary.ndim == 2:
        if ary.dtype == np.float32:
            out = np2Mat2D_F32(ary)
        elif ary.dtype == np.uint8:
            out = np2Mat2D(ary)
        else:
            raise TypeError("array data type is not valid")
    elif ary.ndim == 3:
        out = np2Mat3D(ary)
    return out


cdef object Mat2np(Mat m):
    # Create buffer to transfer data from m.data
    cdef Py_buffer buf_info
    # Define the size / len of data

    cdef size_t len = m.rows*m.cols*m.elemSize()

    # Fill buffer
    PyBuffer_FillInfo( & buf_info, NULL, m.data, len, False, PyBUF_FULL_RO)
    # Get Pyobject from buffer data
    Pydata  = PyMemoryView_FromBuffer( & buf_info)

    # Create ndarray with data
    # the dimension of the output array is 2 if the image is grayscale
    if m.channels() > 1:
        shape_array = (m.rows, m.cols, m.channels())
    else:
        shape_array = (m.rows, m.cols)

    if m.channels() > 1:
        pyary = np.asarray(Pydata, dtype=np.uint8).reshape(shape_array)
    else:
        pyary = np.frombuffer(
            Pydata.tobytes(), dtype=np.uint16).reshape(shape_array)
    return pyary

    '''
    #if m.depth() == CV_32F :
    if m.channels() == 1 :
        ary = np.ndarray(shape=shape_array, buffer=Pydata, order='c', dtype=np.uint16)
    else :
    #8-bit image
        ary = np.ndarray(shape=shape_array, buffer=Pydata, order='c', dtype=np.uint8)
    if m.channels() == 3:
        # BGR -> RGB
        ary = np.dstack((ary[...,2], ary[...,1], ary[...,0]))

    # Convert to numpy array
    pyarr = np.asarray(ary)
    #pyarr = ary
    return pyarr
    '''


def np2Mat2np(nparray):
    cdef Mat m

    # Convert numpy array to cv::Mat
    m = np2Mat(nparray)

    # Convert cv::Mat to numpy array
    pyarr = Mat2np(m)

    return pyarr


cdef class PyMat:
    cdef Mat mat

    def __cinit__(self, np_mat):
        self.mat = np2Mat(np_mat)

    def get_mat(self):
        return Mat2np(self.mat)
