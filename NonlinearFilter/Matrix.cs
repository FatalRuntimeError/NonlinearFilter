using System;

namespace NonlinearFilter
{
	class Matrix
	{
		public int N { get; set; }
		public int M { get; set; }

		double[,] elements;

		public Matrix(int n, int m)
		{
			N = n;
			M = m;
			elements = new double[n, m];
		}

		public Matrix(int n)
		{
			N = M = n;
			elements = new double[n, n];
		}

		public Matrix Clone()
		{
			Matrix clone = new Matrix(N, M);
			for (int i = 0; i < N; i++)
				for (int j = 0; j < M; j++)
					clone[i, j] = this[i, j];

			return clone;
		}

		public double this[int i, int j]
		{
			get
			{
				if (i >= N || j >= M || i < 0 || j < 0)
					throw new Exception("Bad matrix index");
				return elements[i, j];
			}

			set
			{
				if (i >= N || j >= M || i < 0 || j < 0)
					throw new Exception("Bad matrix index");

				elements[i, j] = value;
			}
		}

		public Matrix Transpose()
		{
			Matrix transposed = new Matrix(M, N);
			for (int i = 0; i < N; i++)
				for (int j = 0; j < M; j++)
					transposed[j, i] = this[i, j];

			return transposed;
		}

		public Matrix Reverse()
		{
			if (N != M)
				throw new Exception("Matrix can't be reversed. N != M");

			Matrix temp = new Matrix(N, 2 * M);
			for (int i = 0; i < N; i++)
			{
				temp[i, i + N] = 1.0;
				for (int j = 0; j < M; ++j)
					temp[i, j] = this[i, j];
			}

			int newM = 2 * M;

			// Direct pass
			for (int k = 0; k < N; k++)
			{
				for (int i = k; i < N; i++)
				{
					if (MathUtility.NotNull(temp[i, k]))
					{
						for (int m = 0; m < newM; m++)
							(temp[i, m], temp[k, m]) = (temp[k, m], temp[i, m]);

						break;
					}
				}

				double akk = temp[k, k];

				for (int j = 0; j < newM; j++)
					temp[k, j] /= akk;

				for (int i = k + 1; i < N; i++)
				{
					double value = temp[i, k];
					for (int j = k; j < newM; j++)
						temp[i, j] -= temp[k, j] * value;
				}
			}

			// Reverse pass
			for (int k = N - 1; k >= 0; k--)
				for (int i = k - 1; i >= 0; i--)
				{
					double value = temp[i, k];
					for (int j = 0; j < newM; j++)
						temp[i, j] -= temp[k, j] * value;
				}


			Matrix reverse = new Matrix(N);
			for (int i = 0; i < N; i++)
				for (int j = 0; j < N; j++)
					reverse[i, j] = temp[i, j + N];

			return reverse;
		}

		public static Matrix Identity(int N)
		{
			Matrix I = new Matrix(N);
			for (int i = 0; i < N; i++)
				I[i, i] = 1.0;

			return I;
		}

		public static Matrix operator +(Matrix a, Matrix b)
		{
			if (a.N != b.N || a.M != b.M)
				throw new Exception("Matrices's sizes are not equal");

			Matrix c = new Matrix(a.N, a.M);

			for (int i = 0; i < a.N; i++)
				for (int j = 0; j < a.M; j++)
					c[i, j] = a[i, j] + b[i, j];

			return c;
		}

		public static Matrix operator -(Matrix a, Matrix b)
		{
			if (a.N != b.N || a.M != b.M)
				throw new Exception("Matrices's sizes are not equal");

			Matrix c = new Matrix(a.N, a.M);

			for (int i = 0; i < a.N; i++)
				for (int j = 0; j < a.M; j++)
					c[i, j] = a[i, j] - b[i, j];

			return c;
		}

		public static Matrix operator *(Matrix a, Matrix b)
		{
			if (a.M != b.N)
				throw new Exception("Matrices's sizes are not equal");

			Matrix c = new Matrix(a.N, b.M);

			for (int i = 0; i < a.N; i++)
				for (int j = 0; j < b.M; j++)
					for (int k = 0; k < a.M; k++)
						c[i, j] += a[i, k] * b[k, j];

			return c;
		}

		public static Matrix operator *(Matrix a, double value)
		{
			Matrix c = new Matrix(a.N, a.N);

			for (int i = 0; i < a.N; i++)
				for (int j = 0; j < a.M; j++)
					c[i, j] = a[i, j] * value;

			return c;
		}

		public static Vector operator *(Matrix a, Vector v)
		{
			if (a.M != v.N)
				throw new Exception("Matrices's sizes are not equal");

			Vector e = new Vector(a.N);

			for (int i = 0; i < a.N; i++)
					for (int k = 0; k < a.M; k++)
					e[i] += a[i, k] * v[k];

			return e;
		}

		public static Vector operator *(Vector v, Matrix a)
		{
			if (v.N != a.N)
				throw new Exception("Matrices's sizes are not equal");

			Vector e = new Vector(a.M);

			for (int i = 0; i < a.M; i++)
				for (int k = 0; k < v.N; k++)
					e[i] += v[k] * a[k, i];

			return e;
		}
	}
}
