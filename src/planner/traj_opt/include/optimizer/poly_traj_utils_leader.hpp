#pragma once

#include "root_finder.hpp"

#include <iostream>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace poly_traj_leader
{


    template <int dim>
    class Piece
    {
    private:
        double duration;
        Eigen::Matrix<double, dim, 6> coeffMat;

    public:
        Piece() = default;

        Piece(double dur, const Eigen::Matrix<double, dim, 6> &cMat)
            : duration(dur), coeffMat(cMat) {}

        inline int getDim() const
        {
            return dim;
        }

        inline int getOrder() const
        {
            return 5;
        }

        inline double getDuration() const
        {
            return duration;
        }

        inline const Eigen::Matrix<double, dim, 6> &getCoeffMat() const
        {
            return coeffMat;
        }

        inline Eigen::Matrix<double, dim, 5> getVelCoeffMat() const
        {
            Eigen::Matrix<double, dim, 5> velCoeffMat;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                velCoeffMat.col(i) = n * coeffMat.col(i);
                n++;
            }
            return velCoeffMat;
        }

        inline Eigen::Matrix<double, dim, 1> getPos(const double &t) const
        {
            Eigen::Matrix<double, dim, 1> pos; pos.setZero();
            double tn = 1.0;
            for (int i = 5; i >= 0; i--)
            {
                pos += tn * coeffMat.col(i);
                tn *= t;
            }
            return pos;
        }

        inline Eigen::Matrix<double, dim, 1> getVel(const double &t) const
        {
            Eigen::Matrix<double, dim, 1> vel; vel.setZero();
            double tn = 1.0;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                vel += n * tn * coeffMat.col(i);
                tn *= t;
                n++;
            }
            return vel;
        }

        inline Eigen::Matrix<double, dim, 1> getAcc(const double &t) const
        {
            Eigen::Matrix<double, dim, 1> acc; acc.setZero();
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = 3; i >= 0; i--)
            {
                acc += m * n * tn * coeffMat.col(i);
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        inline Eigen::Matrix<double, dim, 1> getJer(const double &t) const
        {
            Eigen::Matrix<double, dim, 1> jer; jer.setZero();
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = 2; i >= 0; i--)
            {
                jer += l * m * n * tn * coeffMat.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        inline Eigen::Matrix<double, dim, 6> normalizePosCoeffMat() const
        {
            Eigen::Matrix<double, dim, 6> nPosCoeffsMat;
            double t = 1.0;
            for (int i = 5; i >= 0; i--)
            {
                nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
                t *= duration;
            }
            return nPosCoeffsMat;
        }

        inline Eigen::Matrix<double, dim, 5> normalizeVelCoeffMat() const
        {
            Eigen::Matrix<double, dim, 5> nVelCoeffMat;
            int n = 1;
            double t = duration;
            for (int i = 4; i >= 0; i--)
            {
                nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
                t *= duration;
                n++;
            }
            return nVelCoeffMat;
        }

        inline Eigen::Matrix<double, dim, 4> normalizeAccCoeffMat() const
        {
            Eigen::Matrix<double, dim, 4> nAccCoeffMat;
            int n = 2;
            int m = 1;
            double t = duration * duration;
            for (int i = 3; i >= 0; i--)
            {
                nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
                n++;
                m++;
                t *= duration;
            }
            return nAccCoeffMat;
        }

        inline double getMaxVelRate() const
        {
            Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return 0.0;
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxVelRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getVel((*it) * duration).squaredNorm();
                        maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                    }
                }
                return sqrt(maxVelRateSqr);
            }
        }

        inline double getMaxAccRate() const
        {
            Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return 0.0;
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxAccRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getAcc((*it) * duration).squaredNorm();
                        maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                    }
                }
                return sqrt(maxAccRateSqr);
            }
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            double sqrMaxVelRate = maxVelRate * maxVelRate;
            if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
                getVel(duration).squaredNorm() >= sqrMaxVelRate)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(2));
                double t2 = duration * duration;
                coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            double sqrMaxAccRate = maxAccRate * maxAccRate;
            if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
                getAcc(duration).squaredNorm() >= sqrMaxAccRate)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(2));
                double t2 = duration * duration;
                double t4 = t2 * t2;
                coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

      

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    template <int dim>
    class Trajectory
    {
    private:
        // typedef std::vector<Piece<dim>> Pieces;
        std::vector<Piece<dim>> pieces;

    public:
        Trajectory() = default;

        Trajectory(const std::vector<double> &durs,
                   const std::vector<Eigen::Matrix<double, dim, 6>> &cMats)
        {
            int N = std::min(durs.size(), cMats.size());
            pieces.reserve(N);
            for (int i = 0; i < N; i++)
            {
                pieces.emplace_back(durs[i], cMats[i]);
            }
        }

        inline int getPieceNum() const
        {
            return pieces.size();
        }

        inline Eigen::VectorXd getDurations() const
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        inline double getTotalDuration() const
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        inline Eigen::MatrixXd getPositions() const
        {
            int N = getPieceNum();
            Eigen::MatrixXd positions(dim, N + 1);
            for (int i = 0; i < N; i++)
            {
                positions.col(i) = pieces[i].getCoeffMat().col(5);
            }
            positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
            return positions;
        }

        inline const Piece<dim> &operator[](int i) const
        {
            return pieces[i];
        }

        inline Piece<dim> &operator[](int i)
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline void reserve(const int &n)
        {
            pieces.reserve(n);
            return;
        }

        inline void emplace_back(const Piece<dim> &piece)
        {
            pieces.emplace_back(piece);
            return;
        }

        inline void emplace_back(const double &dur,
                                 const Eigen::Matrix<double, dim, 6> &cMat)
        {
            pieces.emplace_back(dur, cMat);
            return;
        }

        inline void append(const Trajectory &traj)
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        inline int locatePieceIdx(double &t) const
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            return idx;
        }

        inline Eigen::Matrix<double, dim, 1> getPos(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getPos(t);
        }

        inline Eigen::Matrix<double, dim, 1> getVel(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getVel(t);
        }

        inline Eigen::Matrix<double, dim, 1> getAcc(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getAcc(t);
        }

        inline Eigen::Matrix<double, dim, 1> getJer(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getJer(t);
        }

        inline Eigen::Matrix<double, dim, 1> getJuncPos(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(5);
            }
            else
            {
                return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Matrix<double, dim, 1> getJuncVel(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(4);
            }
            else
            {
                return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Matrix<double, dim, 1> getJuncAcc(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(3) * 2.0;
            }
            else
            {
                return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
            }
        }

        inline double getMaxVelRate() const
        {
            int N = getPieceNum();
            double maxVelRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxVelRate();
                maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
            }
            return maxVelRate;
        }

        inline double getMaxAccRate() const
        {
            int N = getPieceNum();
            double maxAccRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxAccRate();
                maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
            }
            return maxAccRate;
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
            }
            return feasible;
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
            }
            return feasible;
        }

        // GaaiLam
        inline Piece<dim> getPiece(int i) const
        {
            return pieces[i];
        }


        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.
    // A is an N*N band matrix with lower band width lowerBw
    // and upper band width upperBw.
    // Banded LU factorization has O(N) time complexity.
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)
        {
            // In case of re-creating before destroying
            destroy();
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);
            return;
        }

        inline void destroy()
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

        inline void operator=(const BandedSystem &bs)
        {
            ptrData = nullptr;
            create(bs.N, bs.lowerBw, bs.upperBw);
            memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        double *ptrData = nullptr;

    public:
        // Reset the matrix to zero
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        inline double &operator()(const int &i, const int &j)
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solve(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solveAdj(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    template <int dim>
    class MinJerkOpt
    {
    public:
        inline void operator=(const MinJerkOpt &mjo)
        {
            N = mjo.N;
            headPVA = mjo.headPVA;
            tailPVA = mjo.tailPVA;
            T1 = mjo.T1;
            A = mjo.A;
            b = mjo.b;
            T2 = mjo.T2;
            T3 = mjo.T3;
            T4 = mjo.T4;
            T5 = mjo.T5;
            gdC = mjo.gdC;
        }
        ~MinJerkOpt() { A.destroy(); }

    private:
        int N;
        Eigen::Matrix<double, dim, 3> headPVA;
        Eigen::Matrix<double, dim, 3> tailPVA;
        Eigen::VectorXd T1;
        BandedSystem A;
        Eigen::MatrixXd b;

        // Temp variables
        Eigen::VectorXd T2;
        Eigen::VectorXd T3;
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;
    public:
        Eigen::MatrixXd gdC;

    private:
        template <typename EIGENVEC>
        inline void addGradJbyT(EIGENVEC &gdT) const
        {
            for (int i = 0; i < N; i++)
            {
                gdT(i) += 36.0 * b.row(6 * i + 3).squaredNorm() +
                          288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                          576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                          720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                          2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                          3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
            }
            return;
        }

        template <typename EIGENMAT>
        inline void addGradJbyC(EIGENMAT &gdC) const
        {
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) += 240.0 * b.row(6 * i + 3) * T3(i) +
                                      720.0 * b.row(6 * i + 4) * T4(i) +
                                      1440.0 * b.row(6 * i + 5) * T5(i);
                gdC.row(6 * i + 4) += 144.0 * b.row(6 * i + 3) * T2(i) +
                                      384.0 * b.row(6 * i + 4) * T3(i) +
                                      720.0 * b.row(6 * i + 5) * T4(i);
                gdC.row(6 * i + 3) += 72.0 * b.row(6 * i + 3) * T1(i) +
                                      144.0 * b.row(6 * i + 4) * T2(i) +
                                      240.0 * b.row(6 * i + 5) * T3(i);
            }
            return;
        }

        inline void solveAdjGradC(Eigen::MatrixXd &gdC) const
        {
            A.solveAdj(gdC);
            return;
        }

        template <typename EIGENVEC>
        inline void addPropCtoT(const Eigen::MatrixXd &adjGdC, EIGENVEC &gdT) const
        {
            Eigen::MatrixXd B1(6, dim), B2(3, dim);

            Eigen::Matrix<double,1,dim> negVel, negAcc, negJer, negSnp, negCrk;

            for (int i = 0; i < N - 1; i++)
            {
                negVel = -(b.row(i * 6 + 1) +
                           2.0 * T1(i) * b.row(i * 6 + 2) +
                           3.0 * T2(i) * b.row(i * 6 + 3) +
                           4.0 * T3(i) * b.row(i * 6 + 4) +
                           5.0 * T4(i) * b.row(i * 6 + 5));
                negAcc = -(2.0 * b.row(i * 6 + 2) +
                           6.0 * T1(i) * b.row(i * 6 + 3) +
                           12.0 * T2(i) * b.row(i * 6 + 4) +
                           20.0 * T3(i) * b.row(i * 6 + 5));
                negJer = -(6.0 * b.row(i * 6 + 3) +
                           24.0 * T1(i) * b.row(i * 6 + 4) +
                           60.0 * T2(i) * b.row(i * 6 + 5));
                negSnp = -(24.0 * b.row(i * 6 + 4) +
                           120.0 * T1(i) * b.row(i * 6 + 5));
                negCrk = -120.0 * b.row(i * 6 + 5);

                B1 << negSnp, negCrk, negVel, negVel, negAcc, negJer;

                gdT(i) += B1.cwiseProduct(adjGdC.block<6, dim>(6 * i + 3, 0)).sum();
            }

            negVel = -(b.row(6 * N - 5) +
                       2.0 * T1(N - 1) * b.row(6 * N - 4) +
                       3.0 * T2(N - 1) * b.row(6 * N - 3) +
                       4.0 * T3(N - 1) * b.row(6 * N - 2) +
                       5.0 * T4(N - 1) * b.row(6 * N - 1));
            negAcc = -(2.0 * b.row(6 * N - 4) +
                       6.0 * T1(N - 1) * b.row(6 * N - 3) +
                       12.0 * T2(N - 1) * b.row(6 * N - 2) +
                       20.0 * T3(N - 1) * b.row(6 * N - 1));
            negJer = -(6.0 * b.row(6 * N - 3) +
                       24.0 * T1(N - 1) * b.row(6 * N - 2) +
                       60.0 * T2(N - 1) * b.row(6 * N - 1));

            B2 << negVel, negAcc, negJer;

            gdT(N - 1) += B2.cwiseProduct(adjGdC.block<3, dim>(6 * N - 3, 0)).sum();

            return;
        }//here

        template <typename EIGENMAT>
        inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP) const
        {
            for (int i = 0; i < N - 1; i++)
            {
                // gdInP.col(i) += adjGdC.row(6 * i + 5).transpose();
                gdInP.col(i) = adjGdC.row(6 * i + 5).transpose(); // zxzx
            }
            return;
        }


    public:
        inline void reset(const Eigen::Matrix<double, dim, 3> &headState,
                          const Eigen::Matrix<double, dim, 3> &tailState,
                          const int &pieceNum)
        {
            N = pieceNum;
            headPVA = headState;
            tailPVA = tailState;
            T1.resize(N);
            A.create(6 * N, 6, 6);
            b.resize(6 * N, dim);
            gdC.resize(6 * N, dim);
            return;
        }

        inline void generate(const Eigen::MatrixXd &inPs,
                             const Eigen::VectorXd &ts)
        {
            if (inPs.cols() == 0)
            {

                T1(0) = ts(0);
                double t1_inv = 1.0 / T1(0);
                double t2_inv = t1_inv * t1_inv;
                double t3_inv = t2_inv * t1_inv;
                double t4_inv = t2_inv * t2_inv;
                double t5_inv = t4_inv * t1_inv;
                Eigen::Matrix<double, dim, 6> coeffMatReversed;
                coeffMatReversed.col(5) = 0.5 * (tailPVA.col(2) - headPVA.col(2)) * t3_inv -
                                          3.0 * (headPVA.col(1) + tailPVA.col(1)) * t4_inv +
                                          6.0 * (tailPVA.col(0) - headPVA.col(0)) * t5_inv;
                coeffMatReversed.col(4) = (-tailPVA.col(2) + 1.5 * headPVA.col(2)) * t2_inv +
                                          (8.0 * headPVA.col(1) + 7.0 * tailPVA.col(1)) * t3_inv +
                                          15.0 * (-tailPVA.col(0) + headPVA.col(0)) * t4_inv;
                coeffMatReversed.col(3) = (0.5 * tailPVA.col(2) - 1.5 * headPVA.col(2)) * t1_inv -
                                          (6.0 * headPVA.col(1) + 4.0 * tailPVA.col(1)) * t2_inv +
                                          10.0 * (tailPVA.col(0) - headPVA.col(0)) * t3_inv;
                coeffMatReversed.col(2) = 0.5 * headPVA.col(2);
                coeffMatReversed.col(1) = headPVA.col(1);
                coeffMatReversed.col(0) = headPVA.col(0);
                b = coeffMatReversed.transpose();
            }
            else
            {
                T1 = ts;
                T2 = T1.cwiseProduct(T1);
                T3 = T2.cwiseProduct(T1);
                T4 = T2.cwiseProduct(T2);
                T5 = T4.cwiseProduct(T1);

                A.reset();
                b.setZero();

                A(0, 0) = 1.0;
                A(1, 1) = 1.0;
                A(2, 2) = 2.0;
                b.row(0) = headPVA.col(0).transpose();
                b.row(1) = headPVA.col(1).transpose();
                b.row(2) = headPVA.col(2).transpose();

                for (int i = 0; i < N - 1; i++)
                {
                    A(6 * i + 3, 6 * i + 3) = 6.0;
                    A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                    A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                    A(6 * i + 3, 6 * i + 9) = -6.0;
                    A(6 * i + 4, 6 * i + 4) = 24.0;
                    A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                    A(6 * i + 4, 6 * i + 10) = -24.0;
                    A(6 * i + 5, 6 * i) = 1.0;
                    A(6 * i + 5, 6 * i + 1) = T1(i);
                    A(6 * i + 5, 6 * i + 2) = T2(i);
                    A(6 * i + 5, 6 * i + 3) = T3(i);
                    A(6 * i + 5, 6 * i + 4) = T4(i);
                    A(6 * i + 5, 6 * i + 5) = T5(i);
                    A(6 * i + 6, 6 * i) = 1.0;
                    A(6 * i + 6, 6 * i + 1) = T1(i);
                    A(6 * i + 6, 6 * i + 2) = T2(i);
                    A(6 * i + 6, 6 * i + 3) = T3(i);
                    A(6 * i + 6, 6 * i + 4) = T4(i);
                    A(6 * i + 6, 6 * i + 5) = T5(i);
                    A(6 * i + 6, 6 * i + 6) = -1.0;
                    A(6 * i + 7, 6 * i + 1) = 1.0;
                    A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                    A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                    A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                    A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                    A(6 * i + 7, 6 * i + 7) = -1.0;
                    A(6 * i + 8, 6 * i + 2) = 2.0;
                    A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                    A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                    A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                    A(6 * i + 8, 6 * i + 8) = -2.0;

                    b.row(6 * i + 5) = inPs.col(i).transpose();
                }

                A(6 * N - 3, 6 * N - 6) = 1.0;
                A(6 * N - 3, 6 * N - 5) = T1(N - 1);
                A(6 * N - 3, 6 * N - 4) = T2(N - 1);
                A(6 * N - 3, 6 * N - 3) = T3(N - 1);
                A(6 * N - 3, 6 * N - 2) = T4(N - 1);
                A(6 * N - 3, 6 * N - 1) = T5(N - 1);
                A(6 * N - 2, 6 * N - 5) = 1.0;
                A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
                A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
                A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
                A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
                A(6 * N - 1, 6 * N - 4) = 2;
                A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
                A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
                A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

                b.row(6 * N - 3) = tailPVA.col(0).transpose();
                b.row(6 * N - 2) = tailPVA.col(1).transpose();
                b.row(6 * N - 1) = tailPVA.col(2).transpose();

                A.factorizeLU();
                A.solve(b);

                return;
            }
        }

        inline const Eigen::MatrixXd &get_b() const
        {
            return b;
        }

        inline const Eigen::VectorXd &get_T1() const
        {
            return T1;
        }

        inline Eigen::MatrixXd &get_gdC()
        {
            return gdC;
        }

        inline double getTrajJerkCost() const
        {
            double objective = 0.0;
            for (int i = 0; i < N; i++)
            {
                objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                             144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                             192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                             240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                             720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                             720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
            }
            return objective;
        }

        inline Trajectory<dim> getTraj(void) const
        {
            Trajectory<dim> traj;
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i), b.block<6, dim>(6 * i, 0).transpose().rowwise().reverse());
            }
            return traj;
        }

       

        template <typename EIGENVEC, typename EIGENMAT>
        inline void getGrad2TP(EIGENVEC &gdT,
                               EIGENMAT &gdInPs)
        {
            solveAdjGradC(gdC);
            addPropCtoT(gdC, gdT);
            addPropCtoP(gdC, gdInPs);
        }

        template <typename EIGENVEC>
        inline void initGradCost(EIGENVEC &gdT,
                                 double &cost)
        {
            // printf( "gdInPs=%d\n", gdInPs.size() );

            gdC.setZero();
            cost = getTrajJerkCost();
            addGradJbyT(gdT);
            addGradJbyC(gdC);
        }



        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} //namespace poly_traj
