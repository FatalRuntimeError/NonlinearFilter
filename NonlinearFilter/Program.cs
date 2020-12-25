using System;
using System.Diagnostics;
using System.IO;
using System.Collections.Generic;
using System.Threading;
using static System.Math;
using static NonlinearFilter.MathUtility;

namespace NonlinearFilter
{
	class Program
	{
		static string PlotterFolderPath = "C:/Users/Nikita/source/repos/NonlinearFilter/Plotter/";

		static double dt = 0.01;

		static FilterInfo CreateModel()
		{
			FilterInfo info = new FilterInfo();

			info.XCount = 5;
			info.YCount = 2;

			info.dt = dt;

			info.f = new Func<Vector, double>[info.XCount];
			info.h = new Func<Vector, double>[info.YCount];

			info.Q = new Matrix(5);
			info.G = Matrix.Identity(5);
			info.R = new Matrix(2);

			info.f[0] = (Vector v) => v[4] * Cos(v[2]) * info.dt + v[0];
			info.f[1] = (Vector v) => v[4] * Sin(v[2]) * info.dt + v[1];
			info.f[2] = (Vector v) => v[3] * info.dt + v[2];
			info.f[3] = (Vector v) => v[3];
			info.f[4] = (Vector v) => v[4];

			info.h[0] = (Vector v) => v[0];
			info.h[1] = (Vector v) => v[1];

			info.u = new Func<double, double>[5];
			info.u[0] = (double t) => 0.0;
			info.u[1] = (double t) => 0.0;
			info.u[2] = (double t) => 0.0;
			info.u[3] = (double t) => 0.0;
			info.u[4] = (double t) => 0.0;

			return info;
		}

		static void FilterTest1()
		{
			FilterInfo info = CreateModel();

			Vector[] Measurements = new Vector[10];

			for (int i = 0; i < 10; i++)
			{
				Measurements[i] = new Vector(2);
				Measurements[i][0] = (i + 1) * 0.1;
				Measurements[i][1] = 0.0;
			}

			Matrix InitialP = new Matrix(5);
			for (int i = 0; i < 5; i++)
				InitialP[i, i] = (0.01 * 0.01);

			Vector InitialState = new Vector(5);
			InitialState[0] = 0.0;
			InitialState[1] = 0.0;
			InitialState[2] = 0.0;
			InitialState[3] = 0.0;
			InitialState[4] = 1.0;

			NonlinearFilter filter = new NonlinearFilter(info);

			Vector state = InitialState.Clone();
			Matrix P = InitialP.Clone();

			for (int i = 0; i < 5; i++)
				(state, P) = filter.GetNext(state, P, Measurements[i]);
		}

		static void FilterTest2()
		{
			Random random = new Random();
			FilterInfo info = CreateModel();

			// Initial State
			Vector InitialState = new Vector(5);
			InitialState[0] = 5.0;
			InitialState[1] = 3.0;
			InitialState[2] = 1.0;
			InitialState[3] = 0.0;
			InitialState[4] = 1.0;

			info.u = new Func<double, double>[5];
			info.u[0] = (double t) => 0.0;
			info.u[1] = (double t) => 0.0;
			info.u[2] = (double t) => 0.0;
			info.u[3] = (double t) => 0.0;
			info.u[4] = (double t) => 0.0;

			// Initial covariance	
			Matrix InitialP = new Matrix(5);
			for (int i = 0; i < 5; i++)
			{
				InitialP[i, i] = 0.0001;

				if (i >= 3)
					info.Q[i, i] = 0.0001;
			}

			info.R[0, 0] = (1.0);
			info.R[1, 1] = (1.0);

			// Generate measurements
			Vector[] Measurements = new Vector[100];
			Vector X0 = InitialState;
			Measurements[0] = new Vector(2);
			Measurements[0][0] = X0[0];
			Measurements[0][1] = X0[1];
			Console.WriteLine($"{Measurements[0][0]}, {Measurements[0][1]}");

			for (int i = 1; i < 100; i++)
			{
				Vector X1 = new Vector(X0.N);
				X1[0] = info.f[0](X0);
				X1[1] = info.f[1](X0);
				X1[2] = info.f[2](X0);
				X1[3] = info.f[3](X0);
				X1[4] = info.f[4](X0);

				Measurements[i] = new Vector(2);
				Measurements[i][0] = X1[0];
				Measurements[i][1] = X1[1] * ((-1.0 + 2.0 * random.NextDouble()) * 0.1 + 1);
				X0 = X1.Clone();

				Console.WriteLine($"{Measurements[i][0]}, {Measurements[i][1]}");
			}

			NonlinearFilter filter = new NonlinearFilter(info);

			Vector state = InitialState.Clone();
			Matrix P = InitialP.Clone();

			for (int i = 0; i < 5; i++)
			{
				(state, P) = filter.GetNext(state, P, Measurements[i]);
				Console.WriteLine($"{state[0]}, {state[1]}");
			}
		}

		static void OneStepFilterTest2()
		{
			Random random = new Random();
			FilterInfo info = CreateModel();

			Matrix P = new Matrix(5);

			for (int i = 0; i < 5; i++)
			{
				P[i, i] = 10000;

				if (i >= 3)
					info.Q[i, i] = 0.6;
			}

			Matrix P1 = P.Clone();

			for (int i = 0; i < 2; i++)
				info.R[i, i] = 0.3;

			Vector X = new Vector(5);
			X[0] = 5.0;
			X[1] = 3.0;
			X[2] = 0.0;
			X[3] = 0.0;
			X[4] = 1.0;

			Vector Measurement = new Vector(2);
			OneStepIterationFilter filter = new OneStepIterationFilter(info);
			NonlinearFilter filter1 = new NonlinearFilter(info);

			Vector state = X.Clone();
			Vector state1 = X.Clone();

			using (StreamWriter stream = new StreamWriter(PlotterFolderPath + "data.csv"))
			{
				using (StreamWriter stream1 = new StreamWriter(PlotterFolderPath + "data1.csv"))
				using (StreamWriter stream2 = new StreamWriter(PlotterFolderPath + "cov.csv"))
					for (int i = 0; i < 10000; i++)
					{
						Vector X1 = new Vector(X.N);
						for (int k = 0; k < 5; k++)
							X1[k] = info.f[k](X) + info.u[k](i * info.dt);

						X = X1.Clone();

						Vector vk = new Vector(2);
						for (int k = 0; k < 2; k++)
						{
							vk[k] = SampleGaussian(random, 0.0, Sqrt(info.R[k, k]));
							Measurement[k] = X1[k] + vk[k];
						}

						(state, P) = filter.GetNext(state, P, Measurement);
						(state1, P1) = filter1.GetNext(state1, P1, Measurement);

						stream.WriteLine($"{info.dt * (i + 1)}; {X[1]}; {state[1]}; {Measurement[1]}");
						stream1.WriteLine($"{info.dt * (i + 1)}; {X[1]}; {state1[1]}; {Measurement[1]}");
						stream2.WriteLine($"{info.dt * (i + 1)}; {Abs(X[1] - state1[1]) / X[1]}; {Abs(X[1] - state[1]) / X[1]}");
					}
			}
		}

		static (List<double>, List<double>) FilterAvgTest(int itercount)
		{
			List<double> nonlinearError = new List<double>();
			List<double> onestepError = new List<double>();

			Random random = new Random();
			FilterInfo info = CreateModel();

			Matrix P = new Matrix(5);

			for (int i = 0; i < 5; i++)
			{
				P[i, i] = 0.1;

				if (i >= 3)
					info.Q[i, i] = 0.006;
			}

			Matrix P1 = P.Clone();

			for (int i = 0; i < 2; i++)
				info.R[i, i] = 0.3;

			Vector X = new Vector(5);
			X[0] = 5.0;
			X[1] = 3.0;
			X[2] = 0.0;
			X[3] = 0.0;
			X[4] = 1.0;

			Vector Measurement = new Vector(2);
			OneStepIterationFilter onestep = new OneStepIterationFilter(info);
			NonlinearFilter nonlinear = new NonlinearFilter(info);

			Vector state = X.Clone();
			Vector state1 = X.Clone();

			for (int i = 0; i < itercount; i++)
			{
				Vector X1 = new Vector(X.N);
				for (int k = 0; k < 5; k++)
					X1[k] = info.f[k](X) + info.u[k](i * info.dt);

				X = X1.Clone();

				Vector vk = new Vector(2);
				for (int k = 0; k < 2; k++)
				{
					vk[k] = SampleGaussian(random, 0.0, Sqrt(info.R[k, k]));
					Measurement[k] = X1[k] + vk[k];
				}

				(state, P) = onestep.GetNext(state, P, Measurement);
				(state1, P1) = nonlinear.GetNext(state1, P1, Measurement);

				onestepError.Add(Abs(X[1] - state[1]) / X[1]);
				nonlinearError.Add(Abs(X[1] - state1[1]) / X[1]);
			}

			return (nonlinearError, onestepError);
		}

		static void AvgErrorTest(int count, int itercount)
		{
			List<List<double>> nonlinearErrors = new List<List<double>>();
			List<List<double>> onestepErrors = new List<List<double>>();
			for (int i = 0; i < count; i++)
			{
				(List<double> nonlinearError, List<double> onestepError) = FilterAvgTest(itercount);
				nonlinearErrors.Add(nonlinearError);
				onestepErrors.Add(onestepError);
			}

			double[] avgNonlinearError = new double[itercount];
			double[] avgOnestepError = new double[itercount];

			for (int k = 0; k < itercount; k++)
			{
				for (int i = 0; i < count; i++)
				{
					avgNonlinearError[k] += nonlinearErrors[i][k];
					avgOnestepError[k] += onestepErrors[i][k];
				}

				avgNonlinearError[k] /= count;
				avgOnestepError[k] /= count;
			}

			using (StreamWriter stream = new StreamWriter(PlotterFolderPath + "avgcov.csv"))
			{
				for (int i = 0; i < itercount; i++)
					stream.WriteLine($"{dt * (i + 1)}; {avgNonlinearError[i]}; {avgOnestepError[i]};");
			}

			Plotter.DrawPlot(PlotterFolderPath + "avgcov.csv", 2, new List<string>() { "nonlinear", "onestep" });
		}

		static void Main(string[] args)
		{
			System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)Thread.CurrentThread.CurrentCulture.Clone();
			customCulture.NumberFormat.NumberDecimalSeparator = ".";
			Thread.CurrentThread.CurrentCulture = customCulture;

			AvgErrorTest(200, 1000);

			//OneStepFilterTest2();

			//Thread thread1 = new Thread(() => DrawFilter(PlotterFolderPath + "data1.csv", "nonlinear"));
			//Thread thread2 = new Thread(() => DrawPlot(PlotterFolderPath + "cov.csv", 2));
			//Thread thread3 = new Thread(() => DrawFilter(PlotterFolderPath + "data.csv", "onestep"));
			//thread1.Start();
			//thread2.Start();
			//thread3.Start();
		}
	}
}
