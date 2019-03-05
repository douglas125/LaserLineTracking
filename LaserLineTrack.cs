using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Runtime.InteropServices;
using System.Drawing;
using System.Drawing.Imaging;
using System.Threading.Tasks;
using OpenCLTemplate;

namespace ImgPathTrack
{
    public static class LaserLineTrack
    {
        /// <summary>Use C# parallel for?</summary>
        public static bool UseParallelFor = true;

        #region Image functions

        public static void Threshold(float[,] M, float level)
        {
            int W = M.GetLength(0);
            int H = M.GetLength(1);

            if (UseParallelFor)
            {
                //for (int i = 0; i < W; i++)
                Parallel.For (0, W, i =>
                {
                    for (int j = 0; j < H; j++)
                    {
                        if (M[i, j] < 0.15f) M[i, j] = 0;
                    }
                });
            }
            else
            {
                for (int i = 0; i < W; i++)
                    for (int j = 0; j < H; j++) if (M[i, j] < 0.15f) M[i, j] = 0;
            }

        }

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
        #endregion

        #region RDP line simplification


        /// <summary>Find inflection points in curve</summary>
        /// <param name="curve">Curve to analyze</param>
        /// <param name="nInflections">Number of inflections - total points includes start and end</param>
        /// <returns></returns>
        public static int[] FindInflectionPoints(float[] curve, int nInflections)
        {
            List<int> interestPts = new List<int>();

            interestPts.Add(0);
            interestPts.Add(curve.Length - 1);

            for (int k = 0; k < nInflections; k++)
            {
                float maxPtError = float.MinValue;
                int idPtError = -1;

                //we have interestPts.Count-1 regions
                //find region that gives worst error and add a point there
                for (int p = 0; p < interestPts.Count - 1; p++)
                {
                    float curErr;
                    int idPt = RDP(curve, interestPts[p], interestPts[p + 1], out curErr);

                    if (curErr > maxPtError)
                    {
                        maxPtError = curErr;
                        idPtError = idPt;
                    }
                }

                for (int kk = 1; kk < interestPts.Count; kk++)
                {
                    if (interestPts[kk - 1] <= idPtError && idPtError < interestPts[kk])
                    {
                        interestPts.Insert(kk, idPtError);
                        break;
                    }
                }
            }


            return interestPts.ToArray();
        }

        /// <summary>Ramer-Douglas-Peucker algorithm to find next point position for piecewise linear approx</summary>
        /// <param name="v">Vector</param>
        /// <param name="x0">First point in vector to consider</param>
        /// <param name="xf">Second point in vector to consider</param>
        /// <returns></returns>
        public static int RDP(float[] v, int x0, int xf, out float maxErr)
        {
            //in each part, line is given by: (with x0 <= t <= xf)
            //y = v[x0] + (t-x0)/(xf-x0) * (v[xf]-v[x0])
            float ydot = (v[xf] - v[x0]) / (float)(xf - x0);

            maxErr = float.MinValue;
            int idMaxErr = -1;

            float temp = 1.0f / (float)Math.Sqrt((v[x0] - v[xf]) * (v[x0] - v[xf]) + (x0 - xf) * (x0 - xf));

            for (int k = x0; k <= xf; k++)
            {
                //double err = Math.Abs(v[k] - (v[x0] + (k - x0) * ydot));
                //distance of point (k, v[k]) to line (x0, v[x0]) (xf, v[xf])

                float err = temp * Math.Abs((v[xf] - v[x0]) * k - (xf - x0) * v[k] + xf * v[x0] - x0 * v[xf]);

                if (err > maxErr)
                {
                    idMaxErr = k;
                    maxErr = err;
                }
            }

            return idMaxErr;
        }


        #endregion

        /// <summary>Search how many pixels up and down?</summary>
        public const int PIXELSTOSEARCH = 2;

        /// <summary>Distance penalty multiplier</summary>
        public static float distancePenalty = 1e-2f;

        /// <summary>Penalty for not going in a straight line</summary>
        public static float changePenalty = 1e-2f;

        /////// <summary>Penalty for bigger variations - related to distance in a way</summary>
        ////public static float variationPenalty = 0;

        /// <summary>Smooth final path to reject noise</summary>
        static int smoothWindowSize = 2;

        public static System.Diagnostics.Stopwatch swIntegrate = new System.Diagnostics.Stopwatch(), swBacktrack = new System.Diagnostics.Stopwatch(), swSmooth = new System.Diagnostics.Stopwatch();

        public static float[] FastTrack(float[,] M, float threshold)
        {
            swIntegrate.Reset(); swBacktrack.Reset(); swSmooth.Reset();

            int W = M.GetLength(0);
            int H = M.GetLength(1);

            swIntegrate.Start();
            float[,] intM = new float[W, H];

            //initialize integration
            for (int y = 0; y < H; y++) intM[0, y] = (M[0, y] > threshold ? M[0, y] : 0);

            //perform integration - looks at 3 rows above and 3 rows below
            for (int x = 1; x < W; x++)
            {
                if (UseParallelFor)
                {
                    Parallel.For(0, H, y =>
                    {
                        //find best path
                        float maxv = intM[x - 1, y] - distancePenalty;
                        for (int k = -PIXELSTOSEARCH; k <= PIXELSTOSEARCH; k++)
                        {
                            if (y + k >= 0 && y + k < H)
                                maxv = Math.Max(maxv, intM[x - 1, y + k] - (float)Math.Sqrt(1.0f + k * k) * distancePenalty - (k != 0 ? changePenalty : 0.0f));
                        }
                        intM[x, y] = maxv + M[x, y];

                    });
                }
                else
                {
                    for (int y = 0; y < H; y++)
                    {
                        //find best path
                        float maxv = intM[x - 1, y] - distancePenalty;
                        for (int k = -PIXELSTOSEARCH; k <= PIXELSTOSEARCH; k++)
                        {
                            if (y + k >= 0 && y + k < H)
                                maxv = Math.Max(maxv, intM[x - 1, y + k] - (float)Math.Sqrt(1.0f + k * k) * distancePenalty - (k != 0 ? changePenalty : 0.0f));
                        }
                        intM[x, y] = maxv + (M[x, y] > threshold ? M[x,y] : 0);
                    }
                }
            }
            swIntegrate.Stop();

            swBacktrack.Start();
            int idMax = 0;
            float valMax = intM[W - 1, 0];

            //find most amplified path
            for (int y = 0; y < H; y++)
            {
                if (valMax < intM[W - 1,y])
                {
                    valMax = intM[W - 1,y];
                    idMax = y;
                }
            }

            //backtrack path which was used
            int[] path = new int[W];
            path[W - 1] = idMax;

            for (int x = W - 2; x >= 0; x--)
            {
                int y = path[x + 1];

                float maxv = -1e8f;
                int idv = -1;

                for (int k = -PIXELSTOSEARCH; k <= PIXELSTOSEARCH; k++)
                {
                    if (y + k >= 0 && y + k < H)
                    {
                        if (intM[x, y + k] > maxv)
                        {
                            maxv = intM[x, y + k];
                            idv = y + k;
                        }
                    }
                }

                path[x] = idv;

            }
            swBacktrack.Stop();

            swSmooth.Start();
            //integrate path
            float[] pathInt = new float[W];
            float[] smoothPath = new float[W];
            pathInt[0] = path[0];
            for (int j = 1; j < W; j++)
            {
                pathInt[j] = pathInt[j - 1] + path[j];
            }

            //smooth path
            for (int j = 0; j < W; j++)
            {
                int idxP0 = j - smoothWindowSize;
                int idxPf = j + smoothWindowSize;
                if (idxP0 < 0) idxP0 = 0;
                if (idxPf > W - 1) idxPf = W - 1;
                smoothPath[j] = (pathInt[idxPf] - pathInt[idxP0]) / (float)(idxPf - idxP0);
            }
            swSmooth.Stop();

            return smoothPath;
        }


        #region OpenCL

        static CLCalc.Program.Variable CLintM, CLthresh, CLx, CLpenalties, CLDim, CLPath;
        static CLCalc.Program.Image2D CLimg;

        static CLCalc.Program.Kernel kernelinitIntM;
        static CLCalc.Program.Kernel kernelintM;
        static CLCalc.Program.Kernel kernelBackTrack;
        static CLCalc.Program.Kernel kernelincCounter;
        static LaserLineTrack()
        {
            if (CLCalc.CLAcceleration == CLCalc.CLAccelerationType.Unknown) CLCalc.InitCL();

            #region Source

            string src = @"

constant int PIXELSTOSEARCH = PXtoSearch;

const sampler_t smp = CLK_NORMALIZED_COORDS_FALSE | //Natural coordinates
                      CLK_ADDRESS_CLAMP | //Clamp to zeros
                      CLK_FILTER_NEAREST; //Don't interpolate

__kernel void initIntM(__read_only image2d_t img,
                       __global    float *   intM,
                       __constant  float *   threshold)
{
      int y = get_global_id(0);
      uint4 imgColor = read_imageui(img, smp, (int2)(0,y));
      
      float intens = ((float)imgColor.x + (float)imgColor.y + (float)imgColor.z) * 0.001307189542f; // *1.0f/(3*255)

      intM[y] = intens > threshold[0] ? intens : 0.0f;
}

//penalties[0] = distancePenalty, penalties[1] = changePenalty
__kernel void integrateM(__read_only image2d_t img,
                         __global    float *   intM,
                         __constant  float *   threshold,
                         __constant  int   *   x,
                         __constant  float *   penalties)
{
      int y = get_global_id(0);
      int H = get_global_size(0);

      uint4 imgColor = read_imageui(img, smp, (int2)(x[0],y));
      float intens = ((float)imgColor.x + (float)imgColor.y + (float)imgColor.z) * 0.001307189542f; // *1.0f/(3*255)

      float maxv = -1e-8f; //intM[H*(x[0] - 1) + y] - penalties[0];
      for (int k = -PIXELSTOSEARCH; k <= PIXELSTOSEARCH; k++)
      {
         if (y + k >= 0 && y + k < H)
            maxv = fmax(maxv, intM[H*(x[0] - 1) + y + k] - sqrt(1.0f + k * k) * penalties[0] - (k != 0 ? penalties[1] : 0.0f));
      }
      intM[y + H*x[0]] = maxv + (intens > threshold[0] ? intens : 0.0f);
}

__kernel void incCounter(__global int * x)
{
   x[0]++;
}

__kernel void backTrack( __global    float *   intM,
                         __global    int   *   path,
                         __constant  int   *   Dimensions)
{
      int W = Dimensions[0];
      int H = Dimensions[1];

      int idMax = 0;
      float valMax = intM[H*(W - 1)];

      //find most amplified path
      for (int y = 0; y < H; y++)
      {
         if (valMax < intM[H*(W - 1) + y])
         {
             valMax = intM[H*(W - 1) + y];
             idMax = y;
         }
     }

     path[W - 1] = idMax;

     for (int x = W - 2; x >= 0; x--)
     {
         int y = path[x + 1];

         float maxv = -1e8f;
         int idv = -1;

         for (int k = -PIXELSTOSEARCH; k <= PIXELSTOSEARCH; k++)
         {
             if (y + k >= 0 && y + k < H)
             {
                 if (intM[H*x + y + k] > maxv)
                 {
                     maxv = intM[H* x + y + k];
                     idv = y + k;
                 }
             }
         }

         path[x] = idv;
     }
}
";

            #endregion

            CLCalc.Program.Compile(src.Replace("PXtoSearch", PIXELSTOSEARCH.ToString()));
            CLthresh = new CLCalc.Program.Variable(new float[1]);
            CLx = new CLCalc.Program.Variable(new int[1]);
            CLDim = new CLCalc.Program.Variable(new int[2]);
            CLpenalties = new CLCalc.Program.Variable(new float[2]);

            kernelinitIntM = new CLCalc.Program.Kernel("initIntM");
            kernelintM = new CLCalc.Program.Kernel("integrateM");
            kernelBackTrack = new CLCalc.Program.Kernel("backTrack");
            kernelincCounter = new CLCalc.Program.Kernel("incCounter");
        }

        public static float[] CLFastTrack(Bitmap bmp, float threshold)
        {
            swIntegrate.Reset(); swBacktrack.Reset(); swSmooth.Reset();

            int W = bmp.Width;
            int H = bmp.Height;

            CLthresh.WriteToDevice(new float[] { threshold });
            CLpenalties.WriteToDevice(new float[] { distancePenalty, changePenalty });
            CLDim.WriteToDevice(new int[] { W, H });

            if (CLimg == null || CLimg.Width!=W || CLimg.Height != H) 
                CLimg = new CLCalc.Program.Image2D(bmp);
            else CLimg.WriteBitmap(bmp);


            swIntegrate.Start();
            float[] intM = new float[W * H]; //intM[x,y] = intM[y + H*x]
            if (CLintM == null || CLintM.OriginalVarLength != W * H) 
                CLintM = new CLCalc.Program.Variable(intM);

            //initialize integration - copy first column
            kernelinitIntM.Execute(new CLCalc.Program.MemoryObject[] { CLimg, CLintM, CLthresh }, H);

            //perform integration
            CLx.WriteToDevice(new int[] { 0 });
            for (int x = 1; x < W; x++)
            {
                //CLx.WriteToDevice(new int[] { x });
                kernelincCounter.Execute(new CLCalc.Program.MemoryObject[] { CLx }, 1);
                kernelintM.Execute(new CLCalc.Program.MemoryObject[] { CLimg, CLintM, CLthresh, CLx, CLpenalties }, H);
            }
            //CLintM.ReadFromDeviceTo(intM);
            swIntegrate.Stop();


            swBacktrack.Start();

            int[] path = new int[W];
            if (CLPath == null || CLPath.OriginalVarLength != W) CLPath = new CLCalc.Program.Variable(path);
            kernelBackTrack.Execute(new CLCalc.Program.MemoryObject[] { CLintM, CLPath, CLDim }, 1);

            CLPath.ReadFromDeviceTo(path);


            //CLintM.ReadFromDeviceTo(intM);
            //int idMax = 0;
            //float valMax = intM[H*(W - 1)];

            ////find most amplified path
            //for (int y = 0; y < H; y++)
            //{
            //    if (valMax < intM[H*(W - 1) + y])
            //    {
            //        valMax = intM[H* (W - 1) + y];
            //        idMax = y;
            //    }
            //}

            ////backtrack path which was used
            //int[] path = new int[W];
            //path[W - 1] = idMax;

            //for (int x = W - 2; x >= 0; x--)
            //{
            //    int y = path[x + 1];

            //    float maxv = -1e8f;
            //    int idv = -1;

            //    for (int k = -PIXELSTOSEARCH; k <= PIXELSTOSEARCH; k++)
            //    {
            //        if (y + k >= 0 && y + k < H)
            //        {
            //            if (intM[H*x + y + k] > maxv)
            //            {
            //                maxv = intM[H*x + y + k];
            //                idv = y + k;
            //            }
            //        }
            //    }

            //    path[x] = idv;

            //}

            swBacktrack.Stop();

            swSmooth.Start();
            //integrate path
            float[] pathInt = new float[W];
            float[] smoothPath = new float[W];
            pathInt[0] = path[0];
            for (int j = 1; j < W; j++)
            {
                pathInt[j] = pathInt[j - 1] + path[j];
            }

            //smooth path
            for (int j = 0; j < W; j++)
            {
                int idxP0 = j - smoothWindowSize;
                int idxPf = j + smoothWindowSize;
                if (idxP0 < 0) idxP0 = 0;
                if (idxPf > W - 1) idxPf = W - 1;
                smoothPath[j] = (pathInt[idxPf] - pathInt[idxP0]) / (float)(idxPf - idxP0);
            }
            swSmooth.Stop();

            return smoothPath;
        }

        #endregion
    }
}
