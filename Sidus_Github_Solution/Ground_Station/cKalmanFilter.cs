using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Ground_Station
{
    class cKalmanFilter
    {

        //Class Members
        public float [,] X_prev;
        public float [,] X;
        public float [,] P_prev;
        public float [,] P;
        public float [,] U;
        public float [,] K;
        public float [,] H;
        public float [,] B;
        public float [,] A;
        public float [,] R;
        public float [,] Z;
        //Constructors
        public cKalmanFilter(float init_pos, float init_vel, float init_acc, float pos_error, float vel_error, float deltaTime)
        {
            X_prev = new float[2,1];
            X = new float[2,1];
            P_prev = new float[2,2];
            P = new float[2,2];
            U = new float[1,1];
            K = new float[2, 1];
            H = new float[1, 2];
            B = new float[2, 1];
            A = new float[2, 2];
            R = new float[1, 1];
            Z = new float[1, 1];

            X_prev[0, 0] = init_pos;
            X_prev[1, 0] = init_vel;

            A[0, 0] = 1;
            A[0, 1] = deltaTime;
            A[1, 0] = 0;
            A[1, 1] = 1;

            B[0, 0] = 0.5f*deltaTime * deltaTime;
            B[1, 0] = deltaTime;

            U[0, 0] = init_acc;

            P_prev[0, 0] = pos_error * pos_error;
            P_prev[0, 1] = 0;
            P_prev[1, 0] = 0;
            P_prev[1, 1] = vel_error * vel_error;

            H[0, 0] = 1;
            H[0, 1] = 0;

            R[0, 0] = pos_error * pos_error;
        }

        public void update(float pos, float acc)
        {

            //Process Number 1
            X[0, 0] = A[0, 0] * X_prev[0, 0] + A[0, 1] * X_prev[1, 0] + B[0, 0] * U[0, 0];
            X[1, 0] = A[1, 0] * X_prev[0, 0] + A[1, 1] * X_prev[1, 0] + B[1, 0] * U[0, 0];


            //Process Number 2&3
            P[0, 0] = A[0, 0] * P_prev[0, 0] + A[0, 1] * P_prev[1, 0];
            P[0, 1] = A[0, 0] * P_prev[0, 1] + A[0, 1] * P_prev[1, 1];
            P[1, 0] = A[1, 0] * P_prev[0, 0] + A[1, 1] * P_prev[1, 0];
            P[1, 1] = A[1, 0] * P_prev[0, 1] + A[1, 1] * P_prev[1, 1];

            P[0, 0] = P[0, 0] * A[0, 0] + P[0, 1] * A[0, 1] + 0.2f;
            P[0, 1] = P[0, 0] * A[1, 0] + P[0, 1] * A[1, 1];
            P[1, 0] = P[1, 0] * A[0, 0] + P[1, 1] * A[0, 1];
            P[1, 1] = P[1, 0] * A[1, 0] + P[1, 1] * A[1, 1] + 0.4f;

            //Process Number 4
            float[,] temp_nom = new float[2, 1];
            float[,] temp_denom = new float[1, 1];

            temp_nom[0, 0] = P[0, 0] * H[0, 0] + P[0, 1] * H[0, 1];
            temp_nom[1, 0] = P[1, 0] * H[0, 0] + P[1, 1] * H[0, 1];

            temp_denom[0, 0] = H[0, 0] * temp_nom[0, 0] + H[0, 1] * temp_nom[1, 0];

            K[0, 0] = temp_nom[0, 0] / (temp_denom[0, 0] + R[0, 0]);
            K[1, 0] = temp_nom[1, 0] / (temp_denom[0, 0] + R[0, 0]);

            //Process Number 5
            Z[0, 0] = pos;

            //Process Number 6
            float temp = Z[0, 0] - (H[0,0]*X[0,0] + H[0,1]*X[1,0]);

            X[0, 0] = X[0, 0] + K[0, 0] * temp;
            X[1, 0] = X[1, 0] + K[1, 0] * temp;

            //Process Number 7
            float[,] temp2x2 = new float[2, 2];

            temp2x2[0, 0] = 1 - K[0, 0] * H[0, 0];
            temp2x2[0, 1] = 0 - K[0, 0] * H[0, 1];
            temp2x2[1, 0] = 0 - K[1, 0] * H[0, 0];
            temp2x2[1, 1] = 1 - K[1, 0] * H[0, 1];

            P[0, 0] = temp2x2[0, 0] * P[0, 0] + temp2x2[0, 1] * P[1, 0];
            P[0, 1] = temp2x2[0, 0] * P[0, 1] + temp2x2[0, 1] * P[1, 1];
            P[1, 0] = temp2x2[1, 0] * P[0, 0] + temp2x2[1, 1] * P[1, 0];
            P[1, 1] = temp2x2[1, 0] * P[0, 1] + temp2x2[1, 1] * P[1, 1];

            //Process Number 8
            P_prev[0, 0] = P[0, 0];
            P_prev[0, 1] = P[0, 1];
            P_prev[1, 0] = P[1, 0];
            P_prev[1, 1] = P[1, 1];

            X_prev[0, 0] = X[0, 0];
            X_prev[1, 0] = X[1, 0];

            U[0, 0] = acc; 
        }
    }
}
