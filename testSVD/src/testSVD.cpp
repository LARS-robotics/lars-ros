#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace Eigen;
using Eigen::MatrixXd;

using namespace std;
using std::setw;
using std::setprecision;


int main(int argc, char** argv)
{
	MatrixXd J_pseudoInv(2,2);     //  NULL space projection
	MatrixXd A = MatrixXd::Random(2,2);

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU |Eigen::ComputeThinV);
	J_pseudoInv = svd.matrixV()*svd.singularValues().inverse()*svd.matrixU().transpose();
	
	cout << "Hello World! "<<J_pseudoInv<<endl;
	cout << "J_pseudoInv*A "<<J_pseudoInv*A<<endl;
	
	return 0;
}
