using System;
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

			info.InitialState = new Vector(5);
			info.InitialP = new Matrix(5);

			return info;
		}

		static void FilterTest1()
		{
			FilterInfo info = new FilterInfo();
			info.Q = new Matrix(5);
			info.R = new Matrix(2);

			info.Measurements = new Vector[10];

			for (int i = 0; i < 10; i++)
			{
				info.Measurements[i] = new Vector(2);
				info.Measurements[i][0] = (i + 1) * 0.1;
				info.Measurements[i][1] = 0.0;
			}

			info.XCount = 5;
			info.YCount = 2;

			info.dt = 0.1;

			info.f = new Func<Vector, double>[info.XCount];
			info.h = new Func<Vector, double>[info.YCount];

			info.f[0] = (Vector v) => v[4] * Cos(v[2]) * info.dt + v[0];
			info.f[1] = (Vector v) => v[4] * Sin(v[2]) * info.dt + v[1];
			info.f[2] = (Vector v) => v[3] * info.dt + v[2];
			info.f[3] = (Vector v) => v[3];
			info.f[4] = (Vector v) => v[4];

			info.h[0] = (Vector v) => v[0];
			info.h[1] = (Vector v) => v[1];

			info.InitialP = new Matrix(5);
			for (int i = 0; i < 5; i++)
				info.InitialP[i, i] = (0.01 * 0.01);

			info.InitialState = new Vector(5);
			info.InitialState[0] = 0.0;
			info.InitialState[1] = 0.0;
			info.InitialState[2] = 0.0;
			info.InitialState[3] = 0.0;
			info.InitialState[4] = 1.0;

			NonlinearFilter filter = new NonlinearFilter(info);
			filter.Solve(10);
		}

		static void FilterTest2()
		{
			Random random = new Random();
			FilterInfo info = CreateModel();

			// Initial State
			info.InitialState[0] = 5.0;
			info.InitialState[1] = 3.0;
			info.InitialState[2] = 1.0;
			info.InitialState[3] = 0.0;
			info.InitialState[4] = 1.0;

			info.u = new Func<double, double>[5];
			info.u[0] = (double t) => 0.0;
			info.u[1] = (double t) => 0.0;
			info.u[2] = (double t) => 0.0;
			info.u[3] = (double t) => 0.0;
			info.u[4] = (double t) => 0.0;

			// Initial covariance	
			for (int i = 0; i < 5; i++)
			{
				info.InitialP[i, i] = 0.0001;

				if (i >= 3)
					info.Q[i, i] = 0.0001;
			}

			info.R[0, 0] = (1.0);
			info.R[1, 1] = (1.0);

			// Generate measurements
			info.Measurements = new Vector[100];
			Vector X0 = info.InitialState;
			info.Measurements[0] = new Vector(2);
			info.Measurements[0][0] = X0[0];
			info.Measurements[0][1] = X0[1];
			Console.WriteLine($"{info.Measurements[0][0]}, {info.Measurements[0][1]}");

			for (int i = 1; i < 100; i++)
			{
				Vector X1 = new Vector(X0.N);
				X1[0] = info.f[0](X0);
				X1[1] = info.f[1](X0);
				X1[2] = info.f[2](X0);
				X1[3] = info.f[3](X0);
				X1[4] = info.f[4](X0);

				info.Measurements[i] = new Vector(2);
				info.Measurements[i][0] = X1[0];
				info.Measurements[i][1] = X1[1] * ((-1.0 + 2.0 * random.NextDouble()) * 0.1 + 1);
				X0 = X1.Clone();

				Console.WriteLine($"{info.Measurements[i][0]}, {info.Measurements[i][1]}");
			}

			NonlinearFilter filter = new NonlinearFilter(info);
			filter.Solve(70);

			foreach (Vector x in filter.X)
				Console.WriteLine($"{x[0]}, {x[1]}");
		}

		static void OneStepFilterTest1()
		{
			Random random = new Random();
			FilterInfo info = CreateModel();

			// Initial State
			info.InitialState[0] = 5.0;
			info.InitialState[1] = 3.0;
			info.InitialState[2] = 0.0;
			info.InitialState[3] = 0.0;
			info.InitialState[4] = 1.0;

			info.u = new Func<double, double>[5];
			info.u[0] = (double t) => 0.0;
			info.u[1] = (double t) => 0.0;
			info.u[2] = (double t) => 0.0;
			info.u[3] = (double t) => 0.0;
			info.u[4] = (double t) => 0.0;

			// Initial covariance	
			for (int i = 0; i < 5; i++)
			{
				info.InitialP[i, i] = 0.0001;

				if (i >= 3)
					info.Q[i, i] = 0.0001;
			}

			info.R[0, 0] = (1.0);
			info.R[1, 1] = (1.0);

			// Generate measurements
			info.Measurements = new Vector[100];
			Vector X0 = info.InitialState;
			info.Measurements[0] = new Vector(2);
			info.Measurements[0][0] = X0[0];
			info.Measurements[0][1] = X0[1];
			Console.WriteLine($"{info.Measurements[0][0]}, {info.Measurements[0][1]}");

			for (int i = 1; i < 100; i++)
			{
				Vector X1 = new Vector(X0.N);
				X1[0] = info.f[0](X0);
				X1[1] = info.f[1](X0);
				X1[2] = info.f[2](X0);
				X1[3] = info.f[3](X0);
				X1[4] = info.f[4](X0);

				info.Measurements[i] = new Vector(2);
				info.Measurements[i][0] = X1[0];
				info.Measurements[i][1] = X1[1] * ((-1.0 + 2.0 * random.NextDouble()) * 0.1 + 1);
				X0 = X1.Clone();

				Console.WriteLine($"{info.Measurements[i][0]}, {info.Measurements[i][1]}");
			}

			OneStepIterationFilter filter = new OneStepIterationFilter(info);
			filter.Solve(70);

			foreach (Vector x in filter.X)
				Console.WriteLine($"{x[0]}, {x[1]}");
		}

		static void Main(string[] args)
		{
			System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
			customCulture.NumberFormat.NumberDecimalSeparator = ".";
			System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

			OneStepFilterTest1();
			//FilterTest2();
		}
	}
}
