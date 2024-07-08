/***
 * @Author: David-Willo davidwillo@foxmail.com
 * @Date: 2023-07-30 13:00:05
 * @LastEditTime: 2023-07-30 13:39:51
 * @LastEditors: David-Willo
 * @Jinhao HE (David Willo), IADC HKUST(GZ)
 * @Copyright (c) 2023 by davidwillo@foxmail.com, All Rights Reserved.
 */

#include "xslam/visual-localization/wavelet/CWavelet.h"

#include "algorithm"
#include "cmath"
#include "iostream"
#include "string"

using namespace wavelet;

extern double db1_Lo_D[2] = {0.7071, 0.7071};
extern double db1_Hi_D[2] = {-0.7071, 0.7071};
extern double db1_Lo_R[2] = {0.7071, 0.7071};
extern double db1_Hi_R[2] = {0.7071, -0.7071};


extern double db2_Lo_D[4] = {-0.129409522550921, 0.224143868041857,
                             0.836516303737469, 0.482962913144690};
extern double db2_Hi_D[4] = {-0.482962913144690, 0.836516303737469,
                             -0.224143868041857, -0.129409522550921};
extern double db2_Lo_R[4] = {0.482962913144690, 0.836516303737469,
                             0.224143868041857, -0.129409522550921};
extern double db2_Hi_R[4] = {-0.129409522550921, -0.224143868041857,
                             0.836516303737469, -0.482962913144690};

extern double db3_Lo_D[6] = {0.0352262918821007, -0.0854412738822415,
                             -0.135011020010391, 0.459877502119331,
                             0.806891509313339,  0.332670552950957};
extern double db3_Hi_D[6] = {-0.332670552950957, 0.806891509313339,
                             -0.459877502119331, -0.135011020010391,
                             0.0854412738822415, 0.0352262918821007};
extern double db3_Lo_R[6] = {0.332670552950957,   0.806891509313339,
                             0.459877502119331,   -0.135011020010391,
                             -0.0854412738822415, 0.0352262918821007};
extern double db3_Hi_R[6] = {0.0352262918821007, 0.0854412738822415,
                             -0.135011020010391, -0.459877502119331,
                             0.806891509313339,  -0.332670552950957};

extern double db4_Lo_D[8] = {-0.0106, 0.0329, 0.0308, -0.1870,
                             -0.0280, 0.6309, 0.7148, 0.2304};
extern double db4_Hi_D[8] = {-0.2304, 0.7148, -0.6309, -0.0280,
                             0.1870,  0.0308, -0.0329, -0.0106};
extern double db4_Lo_R[8] = {0.2304,  0.7148, 0.6309, -0.0280,
                             -0.1870, 0.0308, 0.0329, -0.0106};
extern double db4_Hi_R[8] = {-0.0106, -0.0329, 0.0308, 0.1870,
                             -0.0280, -0.6309, 0.7148, -0.2304};

extern double db5_Lo_D[10] = {0.1601,  0.6038, 0.7243,  0.1384,  -0.2423,
                              -0.0322, 0.0776, -0.0062, -0.0126, 0.0033};
extern double db5_Hi_D[10] = {0.0033, 0.0126, -0.0062, -0.0776, -0.0322,
                              0.2423, 0.1384, -0.7243, 0.6038,  -0.1601};
extern double db5_Lo_R[10] = {0.1601,  0.6038, 0.7243,  0.1384,  -0.2423,
                              -0.0322, 0.0776, -0.0062, -0.0126, 0.0033};
extern double db5_Hi_R[10] = {0.0033, 0.0126, -0.0062, -0.0776, -0.0322,
                              0.2423, 0.1384, -0.7243, 0.6038,  -0.1601};

extern double db6_Lo_D[12] = {
    -0.00107730108499558, 0.00477725751101065, 0.000553842200993802,
    -0.0315820393180312,  0.0275228655300163,  0.0975016055870794,
    -0.129766867567096,   -0.226264693965169,  0.315250351709243,
    0.751133908021578,    0.494623890398385,   0.111540743350080};
extern double db6_Hi_D[12] = {
    -0.111540743350080,   0.494623890398385,    -0.751133908021578,
    0.315250351709243,    0.226264693965169,    -0.129766867567096,
    -0.0975016055870794,  0.0275228655300163,   0.0315820393180312,
    0.000553842200993802, -0.00477725751101065, -0.00107730108499558};
extern double db6_Lo_R[12] = {
    0.111540743350080,    0.494623890398385,   0.751133908021578,
    0.315250351709243,    -0.226264693965169,  -0.129766867567096,
    0.0975016055870794,   0.0275228655300163,  -0.0315820393180312,
    0.000553842200993802, 0.00477725751101065, -0.00107730108499558};
extern double db6_Hi_R[12] = {
    -0.00107730108499558, -0.00477725751101065, 0.000553842200993802,
    0.0315820393180312,   0.0275228655300163,   -0.0975016055870794,
    -0.129766867567096,   0.226264693965169,    0.315250351709243,
    -0.751133908021578,   0.494623890398385,    -0.111540743350080};

extern double db7_Lo_D[14] = {0.0004,  -0.0018, 0.0004, 0.0126,  -0.0166,
                              -0.0380, 0.0806,  0.0713, -0.2240, -0.1439,
                              0.4698,  0.7291,  0.3965, 0.0779};
extern double db7_Hi_D[14] = {
    -0.0779, 0.3965,          -0.7291, 0.4698, 0.1439, -0.2240, -0.0713,
    0.0806,  0.0380 - 0.0166, -0.0126, 0.0004, 0.0018, 0.0004};
extern double db7_Lo_R[14] = {0.0779,  0.3965, 0.7291,  0.4698,  -0.1439,
                              -0.2240, 0.0713, 0.0806,  -0.0380, -0.0166,
                              0.0126,  0.0004, -0.0018, 0.0004};
extern double db7_Hi_R[14] = {0.0004, 0.0018,  0.0004,  -0.0126, -0.0166,
                              0.0380, 0.0806,  -0.0713, -0.2240, 0.1439,
                              0.4698, -0.7291, 0.3965,  -0.0779};

extern double db8_Lo_D[16] = {
    -0.0001, 0.0007, -0.0004, -0.0049, 0.0087, 0.0140, -0.0441, -0.0174,
    0.1287,  0.0005, -0.2840, -0.0158, 0.5854, 0.6756, 0.3129,  0.0544};
extern double db8_Hi_D[16] = {
    -0.0544, 0.3129,  -0.6756, 0.5854, 0.0158, -0.2840, -0.0005, 0.1287,
    0.0174,  -0.0441, -0.0140, 0.0087, 0.0049, -0.0004, -0.0007, -0.0001};
extern double db8_Lo_R[16] = {
    0.0544,  0.3129,  0.6756, 0.5854, -0.0158, -0.2840, 0.0005, 0.1287,
    -0.0174, -0.0441, 0.0140, 0.0087, -0.0049, -0.0004, 0.0007, -0.0001};
extern double db8_Hi_R[16] = {
    -0.0001, -0.0007, -0.0004, 0.0049, 0.0087, -0.0140, -0.0441, 0.0174,
    0.1287,  -0.0005, -0.2840, 0.0158, 0.5854, -0.6756, 0.3129,  -0.0544};

extern double db9_Lo_D[18] = {0.0000,  -0.0003, 0.0002,  0.0018,  -0.0043,
                              -0.0047, 0.0224,  0.0003,  -0.0676, 0.0307,
                              0.1485,  -0.0968, -0.2933, 0.1332,  0.6573,
                              0.6048,  0.2438,  0.0381};
extern double db9_Hi_D[18] = {-0.0381, 0.2438, -0.6048, 0.6573,  -0.1332,
                              -0.2933, 0.0968, 0.1485,  -0.0307, -0.0676,
                              -0.0003, 0.0224, 0.0047,  -0.0043, -0.0018,
                              0.0002,  0.0003, 0.0000};
extern double db9_Lo_R[18] = {0.0381,  0.2438,  0.6048,  0.6573,  0.1332,
                              -0.2933, -0.0968, 0.1485,  0.0307,  -0.0676,
                              0.0003,  0.0224,  -0.0047, -0.0043, 0.0018,
                              0.0002,  -0.0003, 0.0000};
extern double db9_Hi_R[18] = {0.0000,  0.0003, 0.0002,  -0.0018, -0.0043,
                              0.0047,  0.0224, -0.0003, -0.0676, -0.0307,
                              0.1485,  0.0968, -0.2933, -0.1332, 0.6573,
                              -0.6048, 0.2438, -0.0381};

extern double db10_Lo_D[20] = {-0.0000, 0.0001,  -0.0001, -0.0007, 0.0020,
                               0.0014,  -0.0107, 0.0036,  0.0332,  -0.0295,
                               -0.0714, 0.0931,  0.1274,  -0.1959, -0.2498,
                               0.2812,  0.6885,  0.5272,  0.1882,  0.0267};
extern double db10_Hi_D[20] = {-0.0267, 0.1882, -0.5272, 0.6885,  -0.2812,
                               -0.2498, 0.1959, 0.1274,  -0.0931, -0.0714,
                               0.0295,  0.0332, -0.0036, -0.0107, -0.0014,
                               0.0020,  0.0007, -0.0001, -0.0001, -0.0000};
extern double db10_Lo_R[20] = {0.0267,  0.1882,  0.5272,  0.6885,  0.2812,
                               -0.2498, -0.1959, 0.1274,  0.0931,  -0.0714,
                               -0.0295, 0.0332,  0.0036,  -0.0107, 0.0014,
                               0.0020,  -0.0007, -0.0001, 0.0001,  -0.0000};
extern double db10_Hi_R[20] = {-0.0000, -0.0001, -0.0001, 0.0007, 0.0020,
                               -0.0014, -0.0107, -0.0036, 0.0332, 0.0295,
                               -0.0714, -0.0931, 0.1274,  0.1959, -0.2498,
                               -0.2812, 0.6885,  -0.5272, 0.1882, -0.0267};

WaveFilter db1, db2, db3, db4, db5, db6, db7, db8, db9, db10;


CWavelet::CWavelet(int dbn) {
  m_msgCL1D.dbn = 3;
  m_msgCL1D.Scale = 0;
  m_msgCL1D.allSize = 0;

  m_msgCL2D.dbn = 3;
  m_msgCL2D.Scale = 0;
  m_msgCL2D.allSize = 0;

  m_bInitFlag1D = false;
  m_bInitFlag2D = false;
  InitFilter();    
  SetFilter(dbn);  
}

CWavelet::~CWavelet() {}


void CWavelet::InitFilter() {
  for (int f = 1; f <= 10; f++) {
    switch (f) {
      case 1:
        db1 = {std::vector<double>(db1_Lo_D, db1_Lo_D + f * 2),
               std::vector<double>(db1_Hi_D, db1_Hi_D + f * 2),
               std::vector<double>(db1_Lo_R, db1_Lo_R + f * 2),
               std::vector<double>(db1_Hi_R, db1_Hi_R + f * 2)};
        break;
      case 2:

        db2 = {std::vector<double>(db2_Lo_D, db2_Lo_D + f * 2),
               std::vector<double>(db2_Hi_D, db2_Hi_D + f * 2),
               std::vector<double>(db2_Lo_R, db2_Lo_R + f * 2),
               std::vector<double>(db2_Hi_R, db2_Hi_R + f * 2)};
        break;
      case 3:

        db3 = {std::vector<double>(db3_Lo_D, db3_Lo_D + f * 2),
               std::vector<double>(db3_Hi_D, db3_Hi_D + f * 2),
               std::vector<double>(db3_Lo_R, db3_Lo_R + f * 2),
               std::vector<double>(db3_Hi_R, db3_Hi_R + f * 2)};
        break;

      case 4:

        db4 = {std::vector<double>(db4_Lo_D, db4_Lo_D + f * 2),
               std::vector<double>(db4_Hi_D, db4_Hi_D + f * 2),
               std::vector<double>(db4_Lo_R, db4_Lo_R + f * 2),
               std::vector<double>(db4_Hi_R, db4_Hi_R + f * 2)};
        break;

      case 5:

        db5 = {std::vector<double>(db5_Lo_D, db5_Lo_D + f * 2),
               std::vector<double>(db5_Hi_D, db5_Hi_D + f * 2),
               std::vector<double>(db5_Lo_R, db5_Lo_R + f * 2),
               std::vector<double>(db5_Hi_R, db5_Hi_R + f * 2)};
        break;

      case 6:

        db6 = {std::vector<double>(db6_Lo_D, db6_Lo_D + f * 2),
               std::vector<double>(db6_Hi_D, db6_Hi_D + f * 2),
               std::vector<double>(db6_Lo_R, db6_Lo_R + f * 2),
               std::vector<double>(db6_Hi_R, db6_Hi_R + f * 2)};
        break;

      case 7:

        db7 = {std::vector<double>(db7_Lo_D, db7_Lo_D + f * 2),
               std::vector<double>(db7_Hi_D, db7_Hi_D + f * 2),
               std::vector<double>(db7_Lo_R, db7_Lo_R + f * 2),
               std::vector<double>(db7_Hi_R, db7_Hi_R + f * 2)};
        break;

      case 8:

        db8 = {std::vector<double>(db8_Lo_D, db8_Lo_D + f * 2),
               std::vector<double>(db8_Hi_D, db8_Hi_D + f * 2),
               std::vector<double>(db8_Lo_R, db8_Lo_R + f * 2),
               std::vector<double>(db8_Hi_R, db8_Hi_R + f * 2)};
        break;

      case 9:

        db9 = {std::vector<double>(db9_Lo_D, db9_Lo_D + f * 2),
               std::vector<double>(db9_Hi_D, db9_Hi_D + f * 2),
               std::vector<double>(db9_Lo_R, db9_Lo_R + f * 2),
               std::vector<double>(db9_Hi_R, db9_Hi_R + f * 2)};
        break;

      case 10:

        db10 = {std::vector<double>(db10_Lo_D, db10_Lo_D + f * 2),
                std::vector<double>(db10_Hi_D, db10_Hi_D + f * 2),
                std::vector<double>(db10_Lo_R, db10_Lo_R + f * 2),
                std::vector<double>(db10_Hi_R, db10_Hi_R + f * 2)};
        break;
      default:
        exit(0);
        break;
    }
  }
}


void CWavelet::SetFilter(int dbn) {
  switch (dbn) {
    case 1:
      m_dbFilter = db1;
      m_dbFilter.filterLen = db1.Lo_D.size();
      break;
    case 2:
      m_dbFilter = db2;
      m_dbFilter.filterLen = db2.Lo_D.size();
      break;
    case 3:
      m_dbFilter = db3;
      m_dbFilter.filterLen = db3.Lo_D.size();
      break;
    case 4:
      m_dbFilter = db4;
      m_dbFilter.filterLen = db4.Lo_D.size();
      break;
    case 5:
      m_dbFilter = db5;
      m_dbFilter.filterLen = db5.Lo_D.size();
      break;
    case 6:
      m_dbFilter = db6;
      m_dbFilter.filterLen = db6.Lo_D.size();
      break;
    case 7:
      m_dbFilter = db7;
      m_dbFilter.filterLen = db7.Lo_D.size();
      break;
    case 8:
      m_dbFilter = db8;
      m_dbFilter.filterLen = db8.Lo_D.size();
      break;
    case 9:
      m_dbFilter = db9;
      m_dbFilter.filterLen = db9.Lo_D.size();
      break;
    case 10:
      m_dbFilter = db10;
      m_dbFilter.filterLen = db10.Lo_D.size();
      break;
    default:
      m_dbFilter = db3;
      m_dbFilter.filterLen = db3.Lo_D.size();
      break;
  }
}


int CWavelet::DWT(double *pSrcData,  
                  int srcLen,        
                  double *pDstCeof  
) {
  

  if (srcLen <
      m_dbFilter.filterLen -
          1) {  
    
    
    std::cout << "Filter length longer than signal";
    exit(1);
  }
  int exLen = (srcLen + m_dbFilter.filterLen - 1) / 2;  
  int k = 0;
  double tmp = 0.0;
  for (int i = 0; i < exLen; i++) {
    pDstCeof[i] = 0.0;
    pDstCeof[i + exLen] = 0.0;
    for (int j = 0; j < m_dbFilter.filterLen; j++) {
      k = 2 * i - j + 1;

      if ((k < 0) && (k >= -m_dbFilter.filterLen + 1))  
        tmp = pSrcData[-k - 1];
      else if ((k >= 0) && (k <= srcLen - 1))  
        tmp = pSrcData[k];
      else if ((k > srcLen - 1) &&
               (k <= (srcLen + m_dbFilter.filterLen - 2)))  
        tmp = pSrcData[2 * srcLen - k - 1];
      else
        tmp = 0.0;
      pDstCeof[i] += m_dbFilter.Lo_D[j] * tmp;
      pDstCeof[i + exLen] += m_dbFilter.Hi_D[j] * tmp;
    }
  }
  return 2 * exLen;
}

bool CWavelet::WaveDec(double *pSrcData, double *pDstCeof) {
  if (pSrcData == NULL || pDstCeof == NULL) return false;

  if (!m_bInitFlag1D) {
    std::cout << "������Ϣ��δ��ʼ�����޷����źŽ��зֽ�"
                 "!"
              << std::endl;
    return false;
  }

  int signalLen = m_msgCL1D.msgLen[0];
  int decLevel = m_msgCL1D.Scale;

  double *pTmpSrc = new double[signalLen];
  double *pTmpDst = new double[m_msgCL1D.msgLen[1] * 2];

  for (int i = 0; i < signalLen; i++) pTmpSrc[i] = pSrcData[i];

  int gap = m_msgCL1D.msgLen[1] * 2;
  for (int i = 1; i <= decLevel; i++) {
    int curSignalLen = m_msgCL1D.msgLen[i - 1];
    DWT(pTmpSrc, curSignalLen, pTmpDst);

    for (int j = 0; j < m_msgCL1D.msgLen[i] * 2; j++)
      pDstCeof[m_msgCL1D.allSize - gap + j] = pTmpDst[j];
    for (int k = 0; k < m_msgCL1D.msgLen[i]; k++) pTmpSrc[k] = pTmpDst[k];
    gap -= m_msgCL1D.msgLen[i];
    gap += m_msgCL1D.msgLen[i + 1] * 2;
  }

  delete[] pTmpDst;
  pTmpDst = NULL;
  delete[] pTmpSrc;
  pTmpSrc = NULL;

  return true;
}


double CWavelet::getThr(double *pDetCoef,  
                        int detLen,  
                        bool is2D  
) {
  double thr = 0.0;
  double sigma = 0.0;

  for (int i = 0; i < detLen; i++) pDetCoef[i] = abs(pDetCoef[i]);

  std::sort(pDetCoef, pDetCoef + detLen);

  if (detLen % 2 == 0 && detLen >= 2)
    sigma = (pDetCoef[detLen / 2 - 1] + pDetCoef[detLen / 2]) / 2 / 0.6745;
  else
    sigma = pDetCoef[detLen / 2] / 0.6745;

  if (!is2D) {
    double N = m_msgCL1D.msgLen[0];
    thr = sigma * sqrt(2.0 * log(N));
  } else {
    double size = m_msgCL2D.msgHeight[0] * m_msgCL2D.msgWidth[0];
    thr = sigma * sqrt(2.0 * log(size));
  }
  return thr;
}


void CWavelet::Wthresh(
    double *pDstCoef,  
    double thr,        
    const int allsize,  
    const int gap,  
    SORH ish        
) {
  
  if (ish)  
  {
    for (int i = gap; i < allsize; i++) {
      if (abs(pDstCoef[i]) < thr) pDstCoef[i] = 0.0;
    }
  } else  
  {
    for (int i = gap; i < allsize; i++) {
      if (abs(pDstCoef[i]) < thr) {
        pDstCoef[i] = 0.0;
      } else {
        if (pDstCoef[i] < 0.0)
          pDstCoef[i] = thr - abs(pDstCoef[i]);
        else
          pDstCoef[i] = abs(pDstCoef[i]) - thr;
      }
    }
  }
}


void CWavelet::DWT2(double *pSrcImage,  
                    int height,       
                    int width,        
                    double *pDstCeof  
) {
  if (!m_bInitFlag2D) {
    std::cout << "������Ϣ��δ��ʼ�����޷����źŽ��зֽ�"
                 "!"
              << std::endl;
    return;
  }

  if (pSrcImage == NULL || pDstCeof == NULL) {
    std::cout << "������Ϣ��dwt2�������ڴ�" << std::endl;
    
    exit(1);
  }

  int exwidth = (width + m_dbFilter.filterLen - 1) / 2 * 2;  
  int exheight = (height + m_dbFilter.filterLen - 1) / 2 * 2;  

  double *tempImage = new double[exwidth * height];

  
  double *tempAhang = new double[width];
  double *tempExhang = new double[exwidth];  
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++)
      tempAhang[j] = pSrcImage[i * width + j];  

    DWT(tempAhang, width, tempExhang);

    for (int j = 0; j < exwidth; j++)
      tempImage[i * exwidth + j] = tempExhang[j];
  }

  
  double *tempAlie = new double[height];  
  double *tempexlie = new double[exheight];  
  for (int i = 0; i < exwidth; i++) {
    
    for (int j = 0; j < height; j++)
      tempAlie[j] = tempImage[j * exwidth + i];  

    
    DWT(tempAlie, height, tempexlie);

    
    for (int j = 0; j < exheight; j++) pDstCeof[j * exwidth + i] = tempexlie[j];
  }

  AdjustData(pDstCeof, exheight, exwidth);  

  delete[] tempAlie;
  tempAlie = NULL;
  delete[] tempexlie;
  tempexlie = NULL;

  delete[] tempAhang;
  tempAhang = NULL;
  delete[] tempExhang;
  tempExhang = NULL;

  delete[] tempImage;
  tempImage = NULL;
}


bool CWavelet::AdjustData(double *pDetCoef,
                          const int height,  
                          const int width  
) {
  if (pDetCoef == NULL) return false;
  double *ptmpdet = new double[height / 2 * width];
  for (int i = 0; i < height / 2 * width; i++) ptmpdet[i] = pDetCoef[i];
  int pos1 = 0;
  int pos2 = height / 2 * width / 2;
  for (int i = 0; i < height / 2; i++) {
    for (int j = 0; j < width; j++) {
      if (j < width / 2)
        pDetCoef[pos1++] = ptmpdet[i * width + j];
      else
        pDetCoef[pos2++] = ptmpdet[i * width + j];
    }
  }
  delete[] ptmpdet;
  ptmpdet = NULL;
  return true;
}


bool CWavelet::WaveDec2(
    double *pSrcData,  
    double *pDstCeof  
) {
  if (!m_bInitFlag2D) {
    std::cout << "������Ϣ��δ��ʼ�����޷���ͼ����зֽ�"
                 "!"
              << std::endl;
    return false;
  }
  if (pSrcData == NULL || pDstCeof == NULL)  
    return false;

  int height = m_msgCL2D.msgHeight[0];
  int width = m_msgCL2D.msgWidth[0];
  int scale = m_msgCL2D.Scale;

  
  double *tempImage = new double[height * width];
  int maxCoefSize = 4 * m_msgCL2D.msgHeight[1] * m_msgCL2D.msgWidth[1];
  double *tempDst = new double[maxCoefSize];

  for (int i = 0; i < height * width; i++) tempImage[i] = pSrcData[i];

  int gap = m_msgCL2D.allSize - maxCoefSize;
  for (int i = 1; i <= scale; i++) {
    DWT2(tempImage, height, width, tempDst);

    
    height = m_msgCL2D.msgHeight[i];
    width = m_msgCL2D.msgWidth[i];

    for (int j = 0; j < height * width; j++)
      tempImage[j] = tempDst[j];  
    
    for (int j = 0, k = gap; j < 4 * height * width; j++, k++)
      pDstCeof[k] = tempDst[j];  
    gap -= 4 * m_msgCL2D.msgWidth[i + 1] * m_msgCL2D.msgHeight[i + 1] -
           height * width;
  }
  delete[] tempDst;
  tempDst = NULL;
  delete[] tempImage;
  tempImage = NULL;
  return true;
}


bool CWavelet::IAdjustData(double *pDetCoef,
                           const int height,  
                           const int width    
) {
  if (pDetCoef == NULL) return false;

  double *ptmpdet = new double[height / 2 * width];
  for (int i = 0; i < height / 2 * width; i++) ptmpdet[i] = pDetCoef[i];
  int pos1 = 0;
  int pos2 = height / 2 * width / 2;
  for (int i = 0; i < height / 2; i++) {
    for (int j = 0; j < width; j++) {
      if (j < width / 2)
        pDetCoef[i * width + j] = ptmpdet[pos1++];
      else
        pDetCoef[i * width + j] = ptmpdet[pos2++];
    }
  }

  delete[] ptmpdet;
  ptmpdet = NULL;

  return true;
}


void CWavelet::IDWT2(double *pSrcCeof,  
                     int dstHeight,  
                     int dstWidth,  
                     double *pDstImage  
) {
  int srcHeight = (dstHeight + m_dbFilter.filterLen - 1) / 2 * 2;
  int srcWidth = (dstWidth + m_dbFilter.filterLen - 1) / 2 * 2;  
  IAdjustData(pSrcCeof, srcHeight, srcWidth);  

  double *tempAline = new double[srcHeight];  
  double *tempdstline = new double[dstHeight];  

  double *pTmpImage = new double[srcWidth * dstHeight];
  
  for (int i = 0; i < srcWidth; i++)  
  {
    
    for (int j = 0; j < srcHeight; j++)
      tempAline[j] = pSrcCeof[j * srcWidth + i];  

    IDWT(tempAline, dstHeight, tempdstline);

    
    for (int j = 0; j < dstHeight; j++)
      pTmpImage[j * srcWidth + i] = tempdstline[j];
  }

  
  double *tempAhang = new double[srcWidth];
  double *tempdsthang = new double[dstWidth];  
  for (int i = 0; i < dstHeight; i++) {
    for (int j = 0; j < srcWidth; j++)
      tempAhang[j] = pTmpImage[i * srcWidth + j];  

    IDWT(tempAhang, dstWidth, tempdsthang);

    for (int j = 0; j < dstWidth; j++)
      pDstImage[i * dstWidth + j] = tempdsthang[j];
  }

  delete[] tempAline;
  tempAline = NULL;
  delete[] tempdstline;
  tempdstline = NULL;

  delete[] tempAhang;
  tempAhang = NULL;
  delete[] tempdsthang;
  tempdsthang = NULL;

  delete[] pTmpImage;
  pTmpImage = NULL;
}


void CWavelet::IDWT(double *pSrcCoef,  
                    int dstLen,  
                    double *pDstData  
) {
  int p = 0;
  int caLen = (dstLen + m_dbFilter.filterLen - 1) / 2;
  for (int i = 0; i < dstLen; i++) {
    pDstData[i] = 0.0;
    for (int j = 0; j < caLen; j++) {
      p = i - 2 * j + m_dbFilter.filterLen - 2;
      
      if ((p >= 0) && (p < m_dbFilter.filterLen))
        pDstData[i] += m_dbFilter.Lo_R[p] * pSrcCoef[j] +
                       m_dbFilter.Hi_R[p] * pSrcCoef[j + caLen];
    }
  }
}


bool CWavelet::WaveRec2(double *pSrcCoef,  
                        double *pDstData   
) {
  if (!m_bInitFlag2D) {
    std::cout << "������Ϣ��δ��ʼ�����޷����źŽ��зֽ�"
                 "!"
              << std::endl;
    return false;
  }
  if (pSrcCoef == NULL || pDstData == NULL)  
    return false;

  int height = m_msgCL2D.msgHeight[0];
  int width = m_msgCL2D.msgWidth[0];
  int decLevel = m_msgCL2D.Scale;

  int maxCeofSize = 4 * m_msgCL2D.msgHeight[1] * m_msgCL2D.msgWidth[1];

  double *pTmpImage = new double[maxCeofSize];

  int minCeofSize =
      4 * m_msgCL2D.msgHeight[decLevel] * m_msgCL2D.msgWidth[decLevel];
  for (int i = 0; i < minCeofSize; i++) pTmpImage[i] = pSrcCoef[i];

  int gap = minCeofSize;
  for (int i = decLevel; i >= 1; i--) {
    int nextheight = m_msgCL2D.msgHeight[i - 1];  
    int nextwidth = m_msgCL2D.msgWidth[i - 1];  
    IDWT2(pTmpImage, nextheight, nextwidth, pDstData);

    if (i > 1)  
    {
      for (int j = 0; j < nextheight * nextwidth; j++)
        pTmpImage[j] = pDstData[j];
      for (int j = 0; j < 3 * nextheight * nextwidth; j++)
        pTmpImage[nextheight * nextwidth + j] = pSrcCoef[gap + j];
      gap += 3 * nextheight * nextwidth;
    }
  }

  delete[] pTmpImage;
  pTmpImage = NULL;

  return true;
}


bool CWavelet::WaveRec(double *pSrcCoef,  
                       double *pDstData   
) {
  if (!m_bInitFlag1D) {
    std::cout << "������Ϣ��δ��ʼ�����޷����źŽ��зֽ�"
                 "!"
              << std::endl;
    return false;
  }
  if (pSrcCoef == NULL || pDstData == NULL)  
    return false;

  
  int signalLen = m_msgCL1D.msgLen[0];
  int decLevel = m_msgCL1D.Scale;

  int det1Len = m_msgCL1D.msgLen[1];
  double *pTmpSrcCoef = new double[det1Len * 2];

  for (int i = 0; i < m_msgCL1D.msgLen[decLevel] * 2; i++)
    pTmpSrcCoef[i] = pSrcCoef[i];

  int gap = m_msgCL1D.msgLen[decLevel] * 2;
  for (int i = decLevel; i >= 1; i--) {
    int curDstLen = m_msgCL1D.msgLen[i - 1];
    IDWT(pTmpSrcCoef, curDstLen, pDstData);
    if (i != 1) {
      for (int j = 0; j < curDstLen; j++) pTmpSrcCoef[j] = pDstData[j];
      for (int k = 0; k < curDstLen; k++)
        pTmpSrcCoef[k + curDstLen] = pSrcCoef[k + gap];
      gap += m_msgCL1D.msgLen[i - 1];
    }
  }

  delete[] pTmpSrcCoef;
  pTmpSrcCoef = NULL;
  return true;
}


bool CWavelet::InitDecInfo2D(const int height,  
                             const int width,  
                             const int Scale,  
                             const int dbn  
) {
  if (dbn != 3) SetFilter(dbn);

  if (height < m_dbFilter.filterLen - 1 || width < m_dbFilter.filterLen - 1) {
    std::cout << "������Ϣ���˲������ȴ����źŵĸ߶Ȼ��߿���!" << std::endl;
    return false;
  }

  int srcHeight = height;
  int srcWidth = width;
  m_msgCL2D.dbn = dbn;
  m_msgCL2D.Scale = Scale;
  m_msgCL2D.msgHeight.resize(Scale + 2);
  m_msgCL2D.msgWidth.resize(Scale + 2);
  
  m_msgCL2D.msgHeight[0] = height;
  m_msgCL2D.msgWidth[0] = width;

  
  for (int i = 1; i <= Scale; i++)  
  {
    int exHeight =
        (srcHeight + m_dbFilter.filterLen - 1) / 2;  
    srcHeight = exHeight;
    m_msgCL2D.msgHeight[i] = srcHeight;

    int exWidth =
        (srcWidth + m_dbFilter.filterLen - 1) / 2;  
    srcWidth = exWidth;
    m_msgCL2D.msgWidth[i] = srcWidth;
  }
  m_msgCL2D.msgHeight[Scale + 1] = srcHeight;
  m_msgCL2D.msgWidth[Scale + 1] = srcWidth;

  
  int tmpAllSize = 0;
  int curPartSize = 0;
  int prePartSize = 0;
  for (int i = 1; i <= Scale; i++) {
    curPartSize = m_msgCL2D.msgHeight[i] * m_msgCL2D.msgWidth[i];
    tmpAllSize += curPartSize * 4 - prePartSize;
    prePartSize = curPartSize;
  }
  m_msgCL2D.allSize = tmpAllSize;
  m_bInitFlag2D = true;
  return true;
}


bool CWavelet::InitDecInfo(const int signalLen,  
                           const int decScale,   
                           const int decdbn      
) {
  if (decdbn != 3) SetFilter(decdbn);

  if (signalLen < m_dbFilter.filterLen - 1) {
    std::cout << "������Ϣ���˲������ȴ����ź�!" << std::endl;
    return false;
  }

  int srcLen = signalLen;
  m_msgCL1D.dbn = decdbn;
  m_msgCL1D.Scale = decScale;
  m_msgCL1D.msgLen.resize(decScale + 2);
  m_msgCL1D.msgLen[0] = srcLen;
  for (int i = 1; i <= decScale; i++) {
    int exLen = (srcLen + m_dbFilter.filterLen - 1) / 2;  
    srcLen = exLen;
    m_msgCL1D.msgLen[i] = srcLen;
  }
  m_msgCL1D.msgLen[decScale + 1] = srcLen;

  for (int i = 1; i < decScale + 2; i++)
    m_msgCL1D.allSize += m_msgCL1D.msgLen[i];

  m_bInitFlag1D = true;  
  return true;
}


bool CWavelet::thrDenoise(double *pSrcNoise,  
                          double *pDstData,   
                          bool isHard  
) {
  if (pSrcNoise == NULL || pDstData == NULL)  {
	std::cout << "input or output buffer empty\n";
	return false;
  }

  if (!m_bInitFlag1D)  
    return false;

  double *pDstCoef = new double[m_msgCL1D.allSize];
  WaveDec(pSrcNoise, pDstCoef);  

  int Det1Len = m_msgCL1D.msgLen[1];
  int gapDet = m_msgCL1D.allSize - Det1Len;
  double *pDet1 = new double[Det1Len];
  for (int i = gapDet, j = 0; i < m_msgCL1D.allSize; i++, j++)
    pDet1[j] = pDstCoef[i];

  int gapApp = m_msgCL1D.msgLen[m_msgCL1D.Scale];  
  double thr = getThr(pDet1, Det1Len);  
  Wthresh(pDstCoef, thr, m_msgCL1D.allSize, gapApp, isHard);  
  WaveRec(pDstCoef, pDstData);  

  delete[] pDstCoef;
  pDstCoef = NULL;
  delete[] pDet1;
  pDet1 = NULL;
  return true;
}
 
bool CWavelet::thrDenoise2D(double *pNoiseImag,   
                            double *pDstImag,     
                            bool isHard) {
  if (pNoiseImag == NULL || pDstImag == NULL) exit(2);
  if (!m_bInitFlag2D) return false;

  double *pImagCoef = new double[m_msgCL2D.allSize];
  WaveDec2(pNoiseImag, pImagCoef);   

  int hlSize = m_msgCL2D.msgHeight[1] * m_msgCL2D.msgWidth[1];
  double *pHlCoef = new double[hlSize];

  int gapHL = m_msgCL2D.allSize - hlSize * 3;   
  for (int i = 0; i < hlSize; i++) pHlCoef[i] = pImagCoef[gapHL + i];

  double thr = getThr(pHlCoef, hlSize, true);

  int gapLL = m_msgCL2D.msgHeight[m_msgCL2D.Scale + 1] *
              m_msgCL2D.msgWidth[m_msgCL2D.Scale + 1];  
  Wthresh(pImagCoef, thr, m_msgCL2D.allSize, gapLL, isHard);  
  WaveRec2(pImagCoef, pDstImag);  

  delete[] pImagCoef;
  pImagCoef = NULL;

  delete[] pHlCoef;
  pHlCoef = NULL;
  return true;
}