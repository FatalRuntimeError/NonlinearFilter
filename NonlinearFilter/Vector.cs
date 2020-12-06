using System;

namespace NonlinearFilter
{
	class Vector
	{
		public int N { get; set; }

		double[] elements;

		public Vector(int n)
		{
			N = n;
			elements = new double[n];
		}

		public Vector Clone()
		{
			Vector v = new Vector(N);
			for (int i = 0; i < N; i++)
				v[i] = this[i];

			return v;
		}

		public double this[int i]
		{
			get
			{
				if (i >= N || i < 0)
					throw new Exception("Bad vector index");
				return elements[i];
			}

			set
			{
				if (i >= N || i < 0)
					throw new Exception("Bad vector index");

				elements[i] = value;
			}
		}

		public static Vector operator +(Vector a, Vector b)
		{
			if (a.N != b.N)
				throw new Exception("Matrices's sizes are not equal");

			Vector c = new Vector(a.N);

			for (int i = 0; i < a.N; i++)
					c[i] = a[i] + b[i];

			return c;
		}

		public static Vector operator -(Vector a, Vector b)
		{
			if (a.N != b.N)
				throw new Exception("Matrices's sizes are not equal");

			Vector c = new Vector(a.N);

			for (int i = 0; i < a.N; i++)
					c[i] = a[i] - b[i];

			return c;
		}

		public static double operator *(Vector a, Vector b)
		{
			if (a.N!= b.N)
				throw new Exception("Matrices's sizes are not equal");

			double c = 0.0;

			for (int i = 0; i < a.N; i++)
						c += a[i] * b[i];

			return c;
		}

		public static Vector operator *(Vector a, double value)
		{
			Vector c = new Vector(a.N);

			for (int i = 0; i < a.N; i++)
					c[i] = a[i] * value;

			return c;
		}
	}
}
