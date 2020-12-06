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
	}
}
