#ifndef _WAVELET_H_
#define _WAVELET_H_

#include <vector>

namespace wavelet {

typedef struct tagWaveFilter {
  std::vector<double> Lo_D;
  std::vector<double> Hi_D;
  std::vector<double> Lo_R;
  std::vector<double> Hi_R;
  int filterLen;
} WaveFilter;

typedef struct tagCL1D {
  std::vector<int> msgLen;
  int Scale;
  int dbn;
  int allSize;
} msgCL1D;

typedef struct tagCL2D {
  std::vector<int> msgHeight;
  std::vector<int> msgWidth;
  int Scale;
  int dbn;
  int allSize;
} msgCL2D;

typedef bool SORH;

class CWavelet {
 public:
  CWavelet(int dbn = 3);
  ~CWavelet();

  int DWT(double *pSrcData, int srcLen, double *pDstCeof);

  void IDWT(double *pSrcCoef, int dstLen, double *pDstData);

  void DWT2(double *pSrcImage, int height, int width, double *pImagCeof);

  void IDWT2(double *pSrcCeof, int height, int width, double *pDstImage);

  bool WaveDec(double *pSrcData, double *pDstCeof);

  bool WaveRec(double *pSrcCoef, double *pDstData);

  bool InitDecInfo(const int srcLen, const int Scale, const int dbn = 3);

  bool InitDecInfo2D(const int height, const int width, const int Scale,
                     const int dbn = 3);

  bool thrDenoise(double *pSrcNoise, double *pDstData, bool isHard = true);

  void Wthresh(double *pDstCoef, double thr, const int allsize, const int gap,
               SORH ish);

  bool WaveDec2(double *pSrcData, double *pDstCeof);

  bool WaveRec2(double *pSrcCoef, double *pDstData);

  void InitFilter();

  void SetFilter(int dbn);

  double getThr(double *pDetCoef, int detlen, bool is2D = false);

  bool AdjustData(double *pDetCoef, const int height, const int width);

  bool IAdjustData(double *pDetCoef, const int height, const int width);

  bool thrDenoise2D(double *pNoiseImag, double *pDstImag, bool isHard = true);

 public:
  msgCL1D m_msgCL1D;
  msgCL2D m_msgCL2D;

 private:
  bool m_bInitFlag1D;
  bool m_bInitFlag2D;
  WaveFilter m_dbFilter;
};
}  // namespace wavelet
#endif