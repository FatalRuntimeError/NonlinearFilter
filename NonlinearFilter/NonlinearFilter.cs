using System;
using System.Collections.Generic;

namespace NonlinearFilter
{
	struct FilterInfo
	{
		public int XCount;
		public int YCount;

		public Vector InitialState;
		public Matrix InitialP;

		public double dt;
		public Vector[] Measurements;

		public Func<Vector, double>[] f;
		public Func<double, double>[] u;

		public Func<Vector, double>[] h;

		public Matrix Q;
		public Matrix R;
	}

	class NonlinearFilter
	{
		public FilterInfo Info;
		public NonlinearFilter(FilterInfo info) => Info = info;

		public List<Vector> X = new List<Vector>();
		public List<Matrix> P = new List<Matrix>();

		double CurrentTime = 0.0;

		public void Solve(int iterCount)
		{
			Vector state_k_k = Info.InitialState;
			Matrix P_k_k = Info.InitialP;

			X.Add(state_k_k);
			P.Add(P_k_k);

			for (int i = 0; i < iterCount; i++)
			{
				CurrentTime = (i + 1) * Info.dt;

				Matrix F = CalculateF(state_k_k);

				// First step
				Vector state_k1_k = GetNextState(state_k_k);
				Matrix P_k1_k = F * P_k_k * F.Transpose() + Info.Q;

				// Second step
				Matrix H = CalculateH(state_k1_k);

				Vector y_k1 = Info.Measurements[i + 1];
				Vector h = Geth(state_k1_k);

				Matrix K = P_k1_k * H.Transpose() * (H * P_k1_k * H.Transpose() + Info.R).Reverse();
				Vector state_k1_k1 = state_k1_k + K * (y_k1 - h);

				Matrix P_k1_k1 = (Matrix.Identity(K.N) - K * H) * P_k1_k;

				X.Add(state_k1_k1);
				P.Add(P_k1_k1);

				state_k_k = state_k1_k1;
				P_k_k = P_k1_k1;
			}
		}

		Matrix CalculateF(Vector state)
		{
			Matrix F = new Matrix(Info.XCount);

			for (int i = 0; i < Info.XCount; i++)
				for (int j = 0; j < Info.XCount; j++)
					F[i, j] = MathUtility.Derivative(Info.f[i], state, j);

			return F;
		}

		Matrix CalculateH(Vector state)
		{
			Matrix H = new Matrix(Info.YCount, Info.XCount);

			for (int i = 0; i < Info.YCount; i++)
				for (int j = 0; j < Info.XCount; j++)
					H[i, j] = MathUtility.Derivative(Info.h[i], state, j);

			return H;
		}

		Vector GetNextState(Vector state)
		{
			Vector newstate = new Vector(state.N);
			for (int i = 0; i < newstate.N; i++)
				newstate[i] = Info.f[i](state) + Info.u[i](CurrentTime);

			return newstate;
		}

		Vector Geth(Vector state)
		{
			Vector h = new Vector(Info.YCount);
			for (int i = 0; i < h.N; i++)
				h[i] = Info.h[i](state);

			return h;
		}
	}

	class OneStepIterationFilter
	{
		public FilterInfo Info;
		public OneStepIterationFilter(FilterInfo info) => Info = info;

		public List<Vector> X = new List<Vector>();
		public List<Matrix> P = new List<Matrix>();

		double CurrentTime = 0.0;

		public void Solve(int iterCount)
		{
			Vector state_k_k = Info.InitialState;
			Matrix P_k_k = Info.InitialP;

			X.Add(state_k_k);
			P.Add(P_k_k);

			for (int i = 0; i < iterCount; i++)
			{
				CurrentTime = (i + 1) * Info.dt;

				Vector state_k1_k;
				Matrix P_k1_k;
				// Step 1
				{
					Matrix F = CalculateF(state_k_k);
					state_k1_k = GetNextState(state_k_k);
					P_k1_k = F * P_k_k * F.Transpose() + Info.Q;
				}

				Vector state_k1_k1;
				Matrix P_k1_k1;
				// Step 2
				{
					Matrix H = CalculateH(state_k1_k);
					Vector y_k1 = Info.Measurements[i + 1];
					Vector h = Geth(state_k1_k);
					Matrix K = P_k1_k * H.Transpose() * (H * P_k1_k * H.Transpose() + Info.R).Reverse();

					state_k1_k1 = state_k1_k + K * (y_k1 - h);
					P_k1_k1 = (Matrix.Identity(K.N) - K * H) * P_k1_k;
				}

				Vector state_k_k1;
				// Step 3
				{
					Matrix H = CalculateH(state_k1_k1);
					Matrix F = CalculateF(state_k_k);
					Vector y_k1 = Info.Measurements[i + 1];
					Matrix K = P_k_k * F.Transpose() * H.Transpose() * (H * P_k1_k * H.Transpose() + Info.R).Reverse();
					state_k_k1 = state_k_k + K * (y_k1 - H * state_k1_k);
				}

				Vector new_state_k1_k;
				Matrix new_P_k1_k;
				// Step 4
				{
					Vector state_p_k = state_k_k1;
					Vector state_p_k1 = GetNextState(state_p_k);
					Matrix F = CalculateF(state_p_k);
					new_state_k1_k = state_p_k1 + F * (state_k_k - state_p_k);
					new_P_k1_k = F * P_k_k * F.Transpose() + Info.Q;
				}

				Vector new_state_k1_k1;
				Matrix new_P_k1_k1;
				// Step 5
				{
					Matrix H = CalculateH(new_state_k1_k);
					Vector y_k1 = Info.Measurements[i + 1];
					Vector h = Geth(new_state_k1_k);
					Matrix K = new_P_k1_k * H.Transpose() * (H * new_P_k1_k * H.Transpose() + Info.R).Reverse();

					new_state_k1_k1 = new_state_k1_k + K * (y_k1 - h);
					new_P_k1_k1 = (Matrix.Identity(K.N) - K * H) * new_P_k1_k;
				}

				X.Add(new_state_k1_k1);
				P.Add(new_P_k1_k1);

				state_k_k = new_state_k1_k1;
				P_k_k = new_P_k1_k1;
			}
		}

		Matrix CalculateF(Vector state)
		{
			Matrix F = new Matrix(Info.XCount);

			for (int i = 0; i < Info.XCount; i++)
				for (int j = 0; j < Info.XCount; j++)
					F[i, j] = MathUtility.Derivative(Info.f[i], state, j);

			return F;
		}

		Matrix CalculateH(Vector state)
		{
			Matrix H = new Matrix(Info.YCount, Info.XCount);

			for (int i = 0; i < Info.YCount; i++)
				for (int j = 0; j < Info.XCount; j++)
					H[i, j] = MathUtility.Derivative(Info.h[i], state, j);

			return H;
		}

		Vector GetNextState(Vector state)
		{
			Vector newstate = new Vector(state.N);
			for (int i = 0; i < newstate.N; i++)
				newstate[i] = Info.f[i](state) + Info.u[i](CurrentTime);

			return newstate;
		}

		Vector Geth(Vector state)
		{
			Vector h = new Vector(Info.YCount);
			for (int i = 0; i < h.N; i++)
				h[i] = Info.h[i](state);

			return h;
		}
	}
}
