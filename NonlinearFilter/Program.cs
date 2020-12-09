using System;
using System.Collections.Generic;
using static System.Math;

namespace NonlinearFilter
{
	class Program
	{
		static void TestMatrix()
		{
			Matrix a = new Matrix(2, 3);
			Vector e1 = new Vector(2);
			e1[0] = 1;
			e1[1] = 1;

			Vector e2 = new Vector(3);
			e2[0] = 1;
			e2[1] = 1;
			e2[2] = 1;

			a[0, 0] = 1;
			a[0, 1] = 2;
			a[0, 2] = 3;

			a[1, 0] = 1;
			a[1, 1] = 2;
			a[1, 2] = 3;


			Vector c = a * e2;
			Vector c1 = e1 * a;
		}

		static void TestTransposeMatrix()
		{
			Matrix a = new Matrix(2, 3);
			a[0, 0] = 1;
			a[0, 1] = 2;
			a[0, 2] = 3;

			a[1, 0] = 1;
			a[1, 1] = 2;
			a[1, 2] = 3;

			Matrix b = a.Transpose();
		}

		static void TestReverseMatrix1()
		{
			Matrix a = new Matrix(2, 2);
			a[0, 0] = 1;
			a[0, 1] = 1;

			a[1, 0] = 0;
			a[1, 1] = 1;

			Matrix b = a.Reverse();
			Matrix c = a * b;
		}

		static void TestReverseMatrix2()
		{
			Matrix a = new Matrix(5, 5);
			a[0, 0] = 0;
			a[0, 1] = 0;
			a[0, 2] = 3;
			a[0, 3] = 0.8;
			a[0, 4] = 0.15;

			a[1, 0] = 8;
			a[1, 1] = 2;
			a[1, 2] = -1;
			a[1, 3] = 19;
			a[1, 4] = 25.8;

			a[2, 0] = -9;
			a[2, 1] = 8;
			a[2, 2] = 5;
			a[2, 3] = 7.8;
			a[2, 4] = 6.15;

			a[3, 0] = 5;
			a[3, 1] = 4;
			a[3, 2] = 9.7;
			a[3, 3] = -7.6;
			a[3, 4] = -22.9;

			a[4, 0] = 18.32;
			a[4, 1] = -0.1;
			a[4, 2] = 3;
			a[4, 3] = 0.1;
			a[4, 4] = 0.01;

			Matrix b = a.Reverse();
			Matrix c = a * b;
		}

		static FilterInfo CreateModel()
		{
			FilterInfo info = new FilterInfo();

			info.XCount = 5;
			info.YCount = 2;

			info.dt = 0.1;

			info.f = new Func<Vector, double>[info.XCount];
			info.h = new Func<Vector, double>[info.YCount];

			info.Q = new Matrix(5);
			info.R = new Matrix(2);

			info.f[0] = (Vector v) => v[4] * Cos(v[2]) * info.dt + v[0];
			info.f[1] = (Vector v) => v[4] * Sin(v[2]) * info.dt + v[1];
			info.f[2] = (Vector v) => v[3] * info.dt + v[2];
			info.f[3] = (Vector v) => v[3];
			info.f[4] = (Vector v) => v[4];

			info.h[0] = (Vector v) => v[0];
			info.h[1] = (Vector v) => v[1];

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

		static void OneStepFilterTest1()
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
				InitialP[i, i] = 0.01 * 0.01;

				if (i >= 3)
					info.Q[i, i] = 0.01 * 0.01;
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
			
			OneStepIterationFilter filter = new OneStepIterationFilter(info);

			Vector state = InitialState.Clone();
			Matrix P = InitialP.Clone();
			Vector Measurement = new Vector(2);

			Console.WriteLine($"{state[0]}, {state[1]}");

			for (int i = 0; i < 99; i++)
			{
				(state, P) = filter.GetNext(state, P, Measurements[i + 1]);
				Console.WriteLine($"{state[0]}, {state[1]}");
			}
		}

		static void OneStepFilterTest2()
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

			double w = 0.0;
			info.u = new Func<double, double>[5];
			info.u[0] = (double t) => 0.0;
			info.u[1] = (double t) => 0.0;
			info.u[2] = (double t) => 0.0;
			info.u[3] = (double t) =>
			{
				if (t < 1.0)
				{
					w += 0.1;
					return 0.1;
				}
				else if (w > 0)
				{
					w -= 0.1;
					return -0.1;
				}
				else
				{
					return 0.0;
				}
			};
			info.u[4] = (double t) => 0.0;

			// Initial covariance	
			Matrix InitialP = new Matrix(5);
			for (int i = 0; i < 5; i++)
			{
				InitialP[i, i] = 0.01 * 0.01;

				if (i >= 3)
					info.Q[i, i] = 0.01 * 0.01;
			}

			info.R[0, 0] = (0.1 * 0.1);
			info.R[1, 1] = (0.1 * 0.1);

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
				X1[0] = info.f[0](X0) + info.u[0](i * info.dt);
				X1[1] = info.f[1](X0) + info.u[1](i * info.dt);
				X1[2] = info.f[2](X0) + info.u[2](i * info.dt);
				X1[3] = info.f[3](X0) + info.u[3](i * info.dt);
				X1[4] = info.f[4](X0) + info.u[4](i * info.dt);

				Measurements[i] = new Vector(2);
				Measurements[i][0] = X1[0];
				Measurements[i][1] = X1[1] * ((-1.0 + 2.0 * random.NextDouble()) * 0.05 + 1);
				X0 = X1.Clone();

				Console.WriteLine($"{Measurements[i][0]}, {Measurements[i][1]}");
			}

			OneStepIterationFilter filter = new OneStepIterationFilter(info);

			Vector state = InitialState.Clone();
			Matrix P = InitialP.Clone();
			Vector Measurement = new Vector(2);

			Console.WriteLine($"{state[0]}, {state[1]}");

			for (int i = 0; i < 99; i++)
			{
				(state, P) = filter.GetNext(state, P, Measurements[i + 1]);
				Console.WriteLine($"{state[0]}, {state[1]}");
			}
		}

		static void Main(string[] args)
		{
			System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
			customCulture.NumberFormat.NumberDecimalSeparator = ".";
			System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

			OneStepFilterTest2();
		}
	}
}
