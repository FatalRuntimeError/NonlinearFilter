using System;
using static System.Math;

namespace NonlinearFilter
{
	class MathUtility
	{
		public static double Derivative(Func<Vector, double> f, Vector point, int varN)
		{
			Vector newPoint = point.Clone();
			newPoint[varN] += 0.0001;

			return (f(newPoint) - f(point)) / 0.0001;
		}

		public static bool NotNull(double a)
		{
			return Abs(a) >= 1.0e-9;
		}

		public static double SampleGaussian(Random random, double mean, double stddev)
		{
			double x1 = 1 - random.NextDouble();
			double x2 = 1 - random.NextDouble();

			double y1 = Sqrt(-2.0 * Log(x1)) * Cos(2.0 * PI * x2);
			return y1 * stddev + mean;
		}
	}
}
