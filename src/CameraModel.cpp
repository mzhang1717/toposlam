#include <toposlam/CameraModel.h>
#include <toposlam/SmallMatrixOpts.h>
#include <TooN/SVD.h>
#include <eigen3/unsupported/Eigen/Polynomials>

CameraModel::CameraModel()
{
}

CameraModel::CameraModel(TooN::Vector<9> v9Params, TooN::Vector<2> ImageSize)//, CVD::ImageRef irCalibSize, CVD::ImageRef irFullScaleSize, CVD::ImageRef irImageSize)
{
  mv9CameraParams = v9Params;
  mv2ImageSize = ImageSize;
  
  //mv2CalibSize = CVD::vec(irCalibSize);
  //mv2FullScaleSize = CVD::vec(irFullScaleSize);
  //mv2ImageSize = CVD::vec(irImageSize);
  //mbCalibrationMode = false;
  
  RefreshParams();
}

CameraModel::~CameraModel()
{
}

void CameraModel::RefreshParams()
{
	// The parameters (by index) are:
	// 0 - a0 coefficient 
	// 1 - a2 coefficient 
	// 2 - a3 coefficient 
	// 3 - a4 coefficient 
	// 4 - center of projection xc 
	// 5 - center of projection yc 
	// 6 - affine transform param c 
	// 7 - affine transform param d
	// 8 - affine transform param e
	
	// Fill the 4th order polynomial coefficient vector
	mv5PolyCoeffs[0] = mv9CameraParams[0];
	mv5PolyCoeffs[1] = 0;
	mv5PolyCoeffs[2] = mv9CameraParams[1];
	mv5PolyCoeffs[3] = mv9CameraParams[2];
	mv5PolyCoeffs[4] = mv9CameraParams[3];
	
	ROS_DEBUG_STREAM("TaylorCamera: Poly coeffs: "<<mv5PolyCoeffs);	
	
	// The centers of projection xc,yc are found when the camera is calibrated,
	mv2Center[0] = mv9CameraParams[4]; // - (mv2CalibSize[0] - mv2FullScaleSize[0])/2;
	mv2Center[1] = mv9CameraParams[5]; // - (mv2CalibSize[1] - mv2FullScaleSize[1])/2;
	
	ROS_DEBUG_STREAM("TaylorCamera: Full scale center: "<< mv2Center);
	
	// Find the furthest corner from the projection center  
	TooN::Vector<2> v2Corner;
	v2Corner[0]= std::max(mv2Center[0], mv2ImageSize[0] - mv2Center[0] - 1);
	v2Corner[1]= std::max(mv2Center[1], mv2ImageSize[1] - mv2Center[1] - 1);    
    
	// Largest radius is computed using full scale size because if the current image size is small due to
	// binning, it still captures the same volume of space as it would if it were bigger but with loss of information
	// Since the camera parameters are calibrated with no binning, all polynomial computations must be done with
	// a full scale image
	mdLargestRadius = sqrt(v2Corner*v2Corner);
	
	// At what stage does the model become invalid?
	mdMaxRho = 1.0 * mdLargestRadius; // (no reason to allow more)
	
	ROS_DEBUG_STREAM("Max rho: "<<mdMaxRho);
	
	// If the theta of a projection is less than this, it is outside the valid model region
	mdMinTheta =  std::atan( PolyVal(mv5PolyCoeffs, mdMaxRho)/ mdMaxRho);   
	
    mbUsingInversePoly = true;
    
    mvxPolyInvCoeffs = FindInvPolyUsingRoots(mv5PolyCoeffs, -1, 0.0001);
    
    if(mvxPolyInvCoeffs.size() == 0)  // Couldn't get good inverse polynomial
    {
		ROS_ERROR_STREAM("TaylorCamera: Couldn't find inverse polynomial with degree <= " << MAX_INV_DEGREE <<", will be solving for roots with Newton's method which is slow"); 
		  
		mbUsingInversePoly = false;
		
		// Get a linear inverse model
		
		mv2LinearInvCoeffs = FindInvPolyUsingRoots(mv5PolyCoeffs, 1);
		ROS_INFO_STREAM("TaylorCamera: Linear inverse polynomial coefficients: "<<mv2LinearInvCoeffs);
		
		mv4PolyDerivCoeffs = mv5PolyCoeffs.slice(1,4);
		for(int i=0; i < mv4PolyDerivCoeffs.size(); ++i)
			mv4PolyDerivCoeffs[i] *= (i+1);
		
		ROS_INFO_STREAM("TaylorCamera: Derivative polynomial coefficients: "<<mv4PolyDerivCoeffs);
    }
    else
    {
		ROS_INFO_STREAM("TaylorCamera: Inverse polynomial coefficients: "<<mvxPolyInvCoeffs);
    }
    
	
		// Create affine transformation matrix from given parameters and image scale
	mm2Affine[0][0] = mv9CameraParams[6];
	mm2Affine[0][1] = mv9CameraParams[7];
	mm2Affine[1][0] = mv9CameraParams[8];
	mm2Affine[1][1] = 1.0;
	
	ROS_DEBUG_STREAM("TaylorCamera: Affine transform: "<< mm2Affine);
	
	mm2AffineInv = opts::M2Inverse(mm2Affine);   // From SmallMatrixOpts.h	
}

// Find the normalized Gaussian value of a vector of values
TooN::Vector<> CameraModel::CenterAndScale(TooN::Vector<> vxX, double dMean, double dStd)
{
	return (vxX - TooN::Ones(vxX.size())*dMean) / dStd;
}

// Find the normalized Gaussian value of a scalar value
double CameraModel::CenterAndScale(double dVal, double dMean, double dStd)
{
	return (dVal - dMean) / dStd;
}

// Fit a polynomial of specified degree to the given input vectors
// See header file for more details and the doxygen HTML page for a nicer view of the equation
TooN::Vector<> CameraModel::PolyFit(TooN::Vector<> vxX, TooN::Vector<> vxY, int nDegree)
{
	TooN::Vector<> v0Empty(0);
	
	if(vxX.size() != vxY.size())
		return v0Empty;
	
	if(nDegree < 1)
		return v0Empty;
	
	int nDimensions = vxX.size();
	
	// Uses the Vandermonde matrix method of fitting a least squares polynomial to a set of data
	TooN::Matrix<> mxVandermondeTrans(nDegree+1, nDimensions);
	
	mxVandermondeTrans[0] = TooN::Ones(nDimensions);
	mxVandermondeTrans[1] = vxX;
	
	for(int i=2; i <= nDegree; ++i)
	{
		mxVandermondeTrans[i] = mxVandermondeTrans[i-1] * vxX.as_diagonal();
	}
	
	// create the SVD decomposition
	TooN::SVD<> svd(mxVandermondeTrans.T());
	
	TooN::Vector<> a = svd.backsub(vxY);
	
	return a;
}

// Evalute a polynomial with given coefficients at a specified location
template<int Size, typename Precision, typename Base> double CameraModel::PolyVal(const TooN::Vector<Size, Precision, Base>& vxCoeff, double dEvalAt)
{
	// Polynomials are stored with the coefficient of zero in the first spot
	// This is the opposive of how matlab's polyval function takes the coefficients
	double val=0;
	for(int i=vxCoeff.size()-1; i > 0; i--)
	{
		val += vxCoeff[i];	
		val *= dEvalAt;
	}
	
	val += vxCoeff[0];
	return val;
}

// Find the inverse of the polynomial describing the Taylor camera model
// See the header and/or doxygen HTML file for more info
TooN::Vector<> CameraModel::FindInvPolyUsingRoots(const TooN::Vector<5>& v5PolyCoeffs, int nSpecifiedDegree, double dErrorLimit)
{
	// We're going to generate a series of closely spaced thetas, and find the corresponding rho for
	// each theta using a polynomial root finder in the Eigen package. After discrading all thetas
	// where no valid rho was found, a polynomial is fitted to the resulting data, keeping in mind
	// that the thetas are now the x values and the rhos are the y values. This polynomial is the
	// inverse of the one passed into the function, to a certain error tolerance.
	
	// Generate the vector of thetas
	double dThetaStart = -M_PI/2 + 0.001;
	double dThetaEnd = M_PI/2 - 0.001;
	double dThetaStep = 0.01;
	int nThetaNum = std::ceil((dThetaEnd-dThetaStart)/dThetaStep) + 1;
	
	std::vector<double> vThetaVector(nThetaNum);
	std::vector<double> vRhoVector(nThetaNum);
	int nDeleteNum = 0;
	
	vThetaVector[0] = dThetaStart;  
	for(unsigned i=1; i < vThetaVector.size(); ++i)
	{
		vThetaVector[i] = vThetaVector[i-1] + dThetaStep;
	}
	
	// Find rho from the theta vector by solving for roots of polynomial equation
	for(unsigned i=0; i < vRhoVector.size(); ++i)
	{
		Eigen::Matrix<double,5,1> polynomial;
		polynomial << v5PolyCoeffs[0], v5PolyCoeffs[1] - tan(vThetaVector[i]), v5PolyCoeffs[2], v5PolyCoeffs[3], v5PolyCoeffs[4];
		
		Eigen::PolynomialSolver<double,4> psolve( polynomial );
		std::vector<double> vRealRoots;
		psolve.realRoots( vRealRoots );
		
		for(int j=vRealRoots.size()-1; j >= 0; --j)
		{
			if(vRealRoots[j] < 0.0 || vRealRoots[j] > mdMaxRho)
				vRealRoots.erase(vRealRoots.begin() + j);
		}
		
		//std::cout << "Roots number: " << vRealRoots.size() << std::endl;
		//std::cout << "Roots: " << vRealRoots << std::endl;
		
		if(vRealRoots.size() != 1)  // Zero or more than one real root means theta outside range of model
		{
			vRhoVector[i] = -9999;  // mark with a recognizable value
			nDeleteNum++;
		}
		else 
			vRhoVector[i] = vRealRoots[0];
	}
	  
	// Create the vectors that will be fitted with a polynomial
	TooN::Vector<> vxTheta(nThetaNum - nDeleteNum);
	TooN::Vector<> vxRho(nThetaNum - nDeleteNum);
	std::vector<double> vError(nThetaNum - nDeleteNum);
	
	// Fill the vectors and compute mean of valid thetas
	int j = 0;
	mdThetaMean = 0;
	for(unsigned i=0; i < vRhoVector.size(); ++i)
	{
		if(vRhoVector[i] == -9999)
			continue;
		
		vxTheta[j] = vThetaVector[i];
		vxRho[j] = vRhoVector[i];
		mdThetaMean += vThetaVector[i];
		
		j++;
	}
	
	mdThetaMean /= vxTheta.size();
	
	// Now get standard deviation
	TooN::Vector<> vxThetaShifted = vxTheta - TooN::Ones(vxTheta.size()) * mdThetaMean;
	mdThetaStd = sqrt((vxThetaShifted * vxThetaShifted) / vxThetaShifted.size());
	
	// Center and scale the thetas to make sure the polynomial is well conditioned
	vxTheta = CenterAndScale(vxTheta, mdThetaMean, mdThetaStd);
	
	if(nSpecifiedDegree < 0)  // We should find the best polynomial fit (within limits)
	{
		int nDegree = 2;  // Start with 2nd order poly
		double dMaxError = 1e10;
		
		while(dMaxError > dErrorLimit && nDegree <= MAX_INV_DEGREE)
		{
			//std::cout << "vxTheta: " << vxTheta << std::endl;
			//std::cout << "vxRho: " << vxTheta << std::endl;
			//std::cout << "nDegree: " << vxTheta << std::endl;
			
			TooN::Vector<> vxInvCoeffs = PolyFit(vxTheta, vxRho, nDegree);
			
			// Evaluate the fit at each theta
			for(int i=0; i < vxTheta.size(); ++i)
			{
				double rho_fit = PolyVal(vxInvCoeffs, vxTheta[i]);
				vError[i] = fabs(vxRho[i] - rho_fit);
			}
			
			dMaxError = *(std::max_element(vError.begin(), vError.end()));
			
			if(dMaxError <= dErrorLimit)
				return vxInvCoeffs;
			
			++nDegree;
		}
		
		ROS_ERROR_STREAM("Hit degree limit, max error was: "<<dMaxError<<" and the limit was: "<<dErrorLimit);
		
		// If we're here, we've hit the degree limit so return an empty
		// vector, which will indicate a bad fit
		TooN::Vector<> v0Empty(0);
		return v0Empty;
	}
	
	// Otherwise we're given a specified degree, so just find a fit and return the coeffs
	return PolyFit(vxTheta, vxRho, nSpecifiedDegree);
}

// Project from the camera reference frame to image pixels,
// while storing intermediate calculation results in member variables
TooN::Vector<2> CameraModel::Project(const TooN::Vector<3>& v3CamFrame)
{
	mv3LastCam = v3CamFrame;
	
	double dNorm = sqrt(mv3LastCam[0]*mv3LastCam[0] + mv3LastCam[1]*mv3LastCam[1]);
	double dTheta, dTanTheta;
	
	if(dNorm == 0)   // Special case, need to short circuit everything because rho will be 0
	{
		dTheta = M_PI_2;
		dTanTheta = NAN;
	}
	else
	{
		dTanTheta = mv3LastCam[2]/dNorm;
		dTheta = atan(dTanTheta);
	}
	
	//ROS_DEBUG_STREAM("dNorm: "<<dNorm);
	//ROS_DEBUG_STREAM("dTanTheta: "<<dTanTheta<<" dTheta: "<<dTheta);
	
	mbInvalid = (dTheta < mdMinTheta);
	
	if(dNorm == 0)
	{
		mdLastRho = 0;
		mdLastCosPhi = 0; 
		mdLastSinPhi = 0;
	}
	else
	{
	// If we're in calibration mode we want to find roots of polynomial directly
	// because bad roots indicate a bad polynomial and we need to set the invalid flag
	//if(mbCalibrationMode)
	//{
		//Eigen::Matrix<double,5,1> polynomial;
		//polynomial << mv5PolyCoeffs[0], mv5PolyCoeffs[1] - dTanTheta, mv5PolyCoeffs[2], mv5PolyCoeffs[3], mv5PolyCoeffs[4];
		
		//Eigen::PolynomialSolver<double,4> psolve( polynomial );
		//std::vector<double> vRealRoots;
		//psolve.realRoots( vRealRoots );
		
		//for(int i=vRealRoots.size()-1; i >= 0; --i)
		//{
			//if(vRealRoots[i] < 0.0 || vRealRoots[i] > mdMaxRho)
				//vRealRoots.erase(vRealRoots.begin() + i);
		//}
		
		//if(vRealRoots.size() != 1)  // no roots or more than 1 root means a bad polynomial
		//{
			//mbInvalid = true;
			//return makeVector(-1,-1);
		//}
		
		//ROS_ASSERT(vRealRoots.size() > 0);
		//mdLastRho = vRealRoots[0];
	//}
	//else  // Otherwise we're running live, want to find rho as fast as possible
	//{
	if(mbUsingInversePoly)
		mdLastRho = PolyVal(mvxPolyInvCoeffs, CenterAndScale(dTheta, mdThetaMean, mdThetaStd));  // If we have a good inverse poly, that's the fastest
	else
	{
		// Otherwise use Newton's method. Get an approximate solution from linear inverse poly
		double rho_approx = PolyVal(mv2LinearInvCoeffs, CenterAndScale(dTheta, mdThetaMean, mdThetaStd));
		mdLastRho = FindRootWithNewton(mv5PolyCoeffs, mv4PolyDerivCoeffs, dTanTheta, rho_approx);
	}
	//}
	
	mdLastCosPhi = mv3LastCam[0]/dNorm; 
	mdLastSinPhi = mv3LastCam[1]/dNorm;
	}
	
	//ROS_DEBUG_STREAM("mdLastRho: "<<mdLastRho);
	//ROS_DEBUG_STREAM("mdLastCosPhi: "<<mdLastCosPhi<<" mdLastSinPhi: "<<mdLastSinPhi);
	
	// On the sensor plane, before affine transform to image plane
	mv2LastDistCam[0] = mdLastCosPhi * mdLastRho; 
	mv2LastDistCam[1] = mdLastSinPhi * mdLastRho;
	
	mv2LastIm = mm2Affine*mv2LastDistCam + mv2Center;
	
	mbInvalid |= !(PointInRectangle(mv2LastIm, mv2ImageSize));
	
	return mv2LastIm;	
}

// Uses Newton's method to find the roots of the given 4th order camera model polynomial. To save on repeated work, the derivative also needs to be supplied.
// Note that this doesn't find the roots of an arbitrary polynomial (even if it's degree 4), see the function PolyVal for the specific equation being evaluted.
// This function will assert on nMaxIter, because if Newton's method doesn't find a root within thins many iterations, either the starting position is way off
// or something else is wrong, but in any case the whole system will work very poorly and slowly so you might as well stop and fix the problem.
double CameraModel::FindRootWithNewton(TooN::Vector<5> v5Coeffs, TooN::Vector<4> v4CoeffsDeriv, double dTanTheta, double dRhoInit, double dErrorLimit, int nMaxIter)
{
  v5Coeffs[1] -= dTanTheta;
  v4CoeffsDeriv[0] -= dTanTheta;
  
  double dRho = dRhoInit;
  double dRhoPrev = dRhoInit;
  double dError = 1;
  int i = 0;
  
  // Main loop of Newton's method
  while(dError > dErrorLimit)
  {
    dRho = dRhoPrev - PolyVal(v5Coeffs, dRhoPrev) / PolyVal(v4CoeffsDeriv, dRhoPrev);
    dError = abs(dRho - dRhoPrev);
    dRhoPrev = dRho;
    
    ++i;
    ROS_ASSERT(i <= nMaxIter); // shouldn't take this long to converge
  }
  
  return dRho;
}


/// Inverse projection operation, projects to the image plane of a pinhole camera model
/** @param v2ImFrame The pixel coordinate to un-project 
*  @return The resulting homogeneous vector (u, v, 1) in Cartesian coordinates */
TooN::Vector<3> CameraModel::UnProject(const TooN::Vector<2>& v2ImFrame)
{
	TooN::Vector<2> mv2LastDistCam = mm2AffineInv * (v2ImFrame - mv2Center);
	
	double mdLastRho = std::sqrt(mv2LastDistCam*mv2LastDistCam);
	
	double dz = PolyVal(mv5PolyCoeffs, mdLastRho);
	
   // std::cout << "dz= " << dz << std::endl;
   // std::cout << "A = " << mm2AffineInv << std::endl;
    
	TooN::Vector<3> mv3LastCam = TooN::makeVector(0.0, 0.0, 1.0);      ///< Last point projected, in camera frame
	if (dz != 0)
	{
		mv3LastCam[0] = mv2LastDistCam[0]/dz;
		mv3LastCam[1] = mv2LastDistCam[1]/dz;
	}	
	
	return mv3LastCam;	
}

/** @brief Test if a point is within the bounds of a rectangle, assumes (0,0) for top left corner
 * 
 *  Similar to STL begin/end tests, this function returns true if the point is on the 
 *  left/top edge of the rectangle but false if it's on the right/bottom edge. This way
 *  we can specify the true dimensions of the rectangle in irRectExtents without having to 
 *  subtract one from each 
 *  @param v2Point The point coordinates to test
 *  @param v2RectExtents The width and height of the rectangle
 *  @return Is the point inside the rectangle?  */
bool CameraModel::PointInRectangle(TooN::Vector<2> v2Point, TooN::Vector<2> v2RectExtents)
{
   if(v2Point[0] >= 0 && v2Point[0] < v2RectExtents[0] &&
     v2Point[1] >= 0 && v2Point[1] < v2RectExtents[1])
     return true;
  else
    return false;
}
