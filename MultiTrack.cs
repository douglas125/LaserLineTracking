using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Threading.Tasks;

namespace ImgPathTrack
{
    /// <summary>Track multiple paths in a given image</summary>
    public class MultiTrack
    {
        public static float[,] GetBmpIntens(Bitmap processedBitmap)
        {
            int W = processedBitmap.Width;
            BitmapData bitmapData = processedBitmap.LockBits(new Rectangle(0, 0, processedBitmap.Width, processedBitmap.Height), ImageLockMode.ReadWrite, processedBitmap.PixelFormat);
            float[,] vals = new float[W, processedBitmap.Height];


            int bytesPerPixel = Bitmap.GetPixelFormatSize(processedBitmap.PixelFormat) / 8;
            int byteCount = bitmapData.Stride * processedBitmap.Height;
            byte[] pixels = new byte[byteCount];
            IntPtr ptrFirstPixel = bitmapData.Scan0;
            Marshal.Copy(ptrFirstPixel, pixels, 0, pixels.Length);
            int heightInPixels = bitmapData.Height;
            int widthInBytes = bitmapData.Width * bytesPerPixel;
            int xx, yy;
            yy = 0;
            float corrFac = 1.0f / (3.0f * 255.0f);
            for (int y = 0; y < heightInPixels; y++)
            {
                int currentLine = y * bitmapData.Stride;
                xx = 0;
                for (int x = 0; x < widthInBytes; x = x + bytesPerPixel)
                {
                    int oldBlue = pixels[currentLine + x];
                    int oldGreen = pixels[currentLine + x + 1];
                    int oldRed = pixels[currentLine + x + 2];
                    //float[] rgb = FcnGetColor(vals[xx + W * yy], min, max);

                    // calculate new pixel value
                    vals[xx, yy] = (oldBlue + oldGreen + oldRed) * corrFac; //limit range to [0,1];
                    xx++;
                }
                yy++;
            }
            // copy modified bytes back
            Marshal.Copy(pixels, 0, ptrFirstPixel, pixels.Length);
            processedBitmap.UnlockBits(bitmapData);
            return vals;
        }


        /// <summary>Initializes a multitracker no collision instance</summary>
        /// <param name="imgW">Image width</param>
        /// <param name="imgH">Image height</param>
        /// <param name="nPaths">Number of paths that will be tracked</param>
        /// <param name="WindowSize">Search size - e.g. 1 or 2</param>
        /// <param name="minDistBetweenLines">Minimum distance between lines in pixels</param>
        public MultiTrack(int imgW, int imgH, int nPaths, int WindowSize, int minDistBetweenLines)
        {
            _W = imgW;
            _H = imgH;
            _nPaths = nPaths;
            MINDISTANCEBETWEENLINES = minDistBetweenLines;

            PrecomputePositions(imgW, imgH, nPaths, WindowSize);

        }


        /// <summary>Tracks multiple reinforced paths in an image</summary>
        /// <param name="img"></param>
        /// <returns></returns>
        public List<float[]> PerformMultiTrack(float[,] img)
        {
            if (img.GetLength(0) != _W || img.GetLength(1) != _H) throw new Exception("Wrong dimensions");

            //Integrated rewards
            float[,] reward = new float[_W, allPositions.Count];

            //reward is a composition of all positions - initialize with values from first column img[0,y]
            for (int k = 0; k < allPositions.Count; k++)
                foreach (int pos in allPositions[k]) reward[0, k] += img[0, pos];

            for (int x = 1; x < _W; x++)
            {
                //loop through all possible states
                Parallel.For(0, allPositions.Count, k =>
                //for (int k = 0; k < allPositions.Count; k++)
                {
                    int[] currentPos = allPositions[k];

                    float bestReward = float.MinValue;

                    //go through all possible predecessors
                    foreach (int prevPos in allowedPrevious[k])
                    {
                        float curReward = 0;
                        int[] previousPos = allPositions[prevPos];

                        //we now have previous and current position. We can compute distance and rewards
                        for (int q = 0; q < _nPaths; q++)
                        {
                            float temp = previousPos[q] - currentPos[q];
                            temp = (float)Math.Sqrt(temp * temp + 1.0);
                            curReward += img[x, currentPos[q]] - DISTANCEPENALTY * temp;
                        }
                        bestReward = Math.Max(bestReward, curReward + reward[x - 1, prevPos]);
                    }

                    reward[x, k] = bestReward;
                });
            }

            //backtrack
            int kBest = -1;
            float valBest = float.MinValue;
            for (int k = 0; k < allPositions.Count; k++)
            {
                if (reward[_W - 1, k] > valBest)
                {
                    valBest = reward[_W - 1, k];
                    kBest = k;
                }
            }

            //create backtracking list
            List<float[]> ans = new List<float[]>();
            for (int m = 0; m < _nPaths; m++)
            {
                ans.Add(new float[_W]);
                ans[m][_W - 1] = allPositions[kBest][m];
            }

            

            for (int x = _W - 2; x >= 0; x--)
            {
                int bestPrev = -1;
                float bestPrevVal = float.MinValue;

                int[] DEBUGcur = allPositions[kBest];

                //find best predecessor
                foreach (int prevPos in allowedPrevious[kBest])
                {
                    int[] DEBUGprev = allPositions[prevPos];

                    if (reward[x, prevPos] > bestPrevVal)
                    {
                        bestPrevVal = reward[x, prevPos];
                        bestPrev = prevPos;
                    }
                }

                kBest = bestPrev;
                int[] DEBUGbestprev = allPositions[bestPrev];

                for (int m = 0; m < _nPaths; m++) ans[m][x] = allPositions[kBest][m];
            }

            return ans;
        }

        #region Precomputed data and configurations

        /// <summary>Penalty to apply because of displacement</summary>
        public float DISTANCEPENALTY = 0.1f;

        /// <summary>Minimum allowable distance between lines</summary>
        public int MINDISTANCEBETWEENLINES = 1;

        int _W, _H, _nPaths;

        //possible positions: [0] goes from 0 to < Height-nPaths, [1] goes from [0] to Height-nPaths+1, nPaths-1 goes from [nPaths-2] to Heights-1
        List<int[]> allPositions = new List<int[]>();

        //possible previous states
        List<List<int>> allowedPrevious = new List<List<int>>();
        #endregion 


        /// <summary>Precomputes all allowed positions and predecessors</summary>
        /// <param name="nPaths">Number of paths</param>
        /// <param name="WindowSize">Window size</param>
        private void PrecomputePositions(int W, int H, int nPaths, int WindowSize)
        {
            
            //possible positions: [0] goes from 0 to < Height-nPaths, [1] goes from [0] to Height-nPaths+1, nPaths-1 goes from [nPaths-2] to Heights-1
            allPositions = new List<int[]>();

            //possible previous states
            allowedPrevious = new List<List<int>>();

            //Compare from left to right
            intComparer ic = new intComparer();
            int[] curPosition = new int[nPaths];
            for (int k = 0; k < nPaths; k++) curPosition[k] = k * MINDISTANCEBETWEENLINES;

            while (curPosition[0] < H - (nPaths-1) * MINDISTANCEBETWEENLINES)
            {
                int[] position = (int[])curPosition.Clone();
                allPositions.Add(position);
                allowedPrevious.Add(null);
                incrementPositionWithoutOverlap(ref curPosition, nPaths - 1, H, nPaths);
            }

            Parallel.For(0, allPositions.Count, k =>
            //for (int k = 0; k < allPositions.Count; k++)
            {
                List<int[]> lstPossiblePrev = GetAllValidOrigins(allPositions[k], H, WindowSize);
                List<int> possibleIndexes = new List<int>();
                foreach (int[] possibPrev in lstPossiblePrev)
                {
                    int p = allPositions.BinarySearch(possibPrev, ic);
                    if (p < 0) throw new Exception("Error!");

                    possibleIndexes.Add(p);
                }

                allowedPrevious[k] = possibleIndexes;
            });
        }

        private class intComparer : Comparer <int[]>
        {
            public override int Compare(int[] x, int[] y)
            {
                for (int k=0;k<x.Length;k++)
                {
                    int val = x[k].CompareTo(y[k]);
                    if (val != 0) return val;
                }
                return 0;
            }
        }

        /// <summary>Increments position vector without overlap</summary>
        private int incrementPositionWithoutOverlap(ref int[] pos, int idx, int totalLen, int posLen)
        {

            pos[idx]++;
            if (pos[idx] == totalLen - (posLen - idx - 1)*MINDISTANCEBETWEENLINES && idx > 0) pos[idx] = incrementPositionWithoutOverlap(ref pos, idx - 1, totalLen, posLen) + MINDISTANCEBETWEENLINES;
            return pos[idx];
        }

        /// <summary>Retrieves all possible predecessors of a state</summary>
        /// <param name="curPos">State</param>
        /// <param name="totalLen">Allowable position less than totalLen </param>
        /// <param name="Window">Window size</param>
        /// <returns></returns>
        public List<int[]> GetAllValidOrigins(int[] curPos, int totalLen, int Window)
        {
            List<int[]> ans = new List<int[]>();
            int pLen = curPos.Length;
            int[] curDelta = new int[pLen + 1];
            for (int k = 0; k < pLen; k++) curDelta[k] = -Window;
            while (curDelta[pLen] == 0)
            {
                int[] state = (int[])curPos.Clone();
                for (int k = 0; k < pLen; k++) state[k] += curDelta[k];
                bool acceptState = true;
                if (state[0] < 0 || state[pLen - 1] >= totalLen) acceptState = false;
                
                if (acceptState)
                {
                    for (int k = 1; k < pLen; k++)
                    {
                        if (state[k - 1] + MINDISTANCEBETWEENLINES > state[k])
                        {
                            acceptState = false;
                            break;
                        }
                    
                    }
                }
                if (acceptState) ans.Add(state);
                //increments delta
                for (int i = 0; i <= pLen; i++)
                {
                    curDelta[i]++;
                    if (curDelta[i] > Window) curDelta[i] = -Window;
                    else break;
                }
            }
            return ans;
        }
    }
}
