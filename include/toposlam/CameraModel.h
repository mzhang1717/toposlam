#ifndef CAMERAMODEL_
#define CAMERAMODEL_

#include <TooN/TooN.h>
#include <ros/assert.h>
#include <toposlam/Types.h>

const int MAX_INV_DEGREE = 30; ///< The maximum degree of the inverse of the 4th order polynomial defining the camera model

class CameraModel
{
public:
	CameraModel();
	
	CameraModel(TooN::Vector<9> v9Params, TooN::Vector<2> ImageSize);
	
	~CameraModel();

	// Various projection functions
	/// Projects from camera frame to pixel coordinates, with distortion
	/** @param v3CamFrame The point to project given in the camera frame in Cartesian coordinates
	*  @return The pixel coordinates of the projection */  
	TooN::Vector<2> Project(const TooN::Vector<3>& v3CamFrame); 
	
	/// Inverse projection operation, projects to the unit sphere
	/** @param v2ImFrame The pixel coordinate to un-project 
	*  @return The resulting unit-length vector in Cartesian coordinates */
	TooN::Vector<3> UnProject(const TooN::Vector<2>& v2ImFrame); 
	
	TooN::Vector<2> GetCenter() const {return mv2Center; };
	TooN::Vector<2> GetSize() const {return mv2ImageSize; };

protected:
	// Polynomial functions
	/** @brief Fit a polynomial of specified degree to the given input vectors
	* 
	* The coefficients of the polynomial of degree n (\f$ y = a_0 + a_1*x + a_2*x^2 + ... + a_n*x^n \f$) are computed using a least squares fit
	* @param vxX Input points x, of arbitrary length, stacked as a column vector
	* @param vxY Input points y, of the same length as x, stacked as a column vector
	* @param nDegree The degree of the polynomial that will be computed
	* @return The coefficients of the polynomial, with the lowest degree coefficient first */
	TooN::Vector<> PolyFit(TooN::Vector<> vxX, TooN::Vector<> vxY, int nDegree);

	/** @brief Find the inverse of the polynomial describing the Taylor camera model
	* 
	* This function solves the specific problem of inverting the 4th order Taylor camera polynomial \f$ tan\theta \rho = a_0 + a_2\rho^2 + a_3\rho^3 + a_4\rho^4 \f$
	* into the function \f$ \rho = g(\theta) \f$, where \f$ g \f$ is a polynomial. The degree of \f$ g \f$ can either be specified, in which case the error
	* of the inverse fit could be large, or it could be found automatically by attempting ever higher order inverse fits until the fit error drops below
	* a threshold. The maximum inverse polynomial degree is given by MAX_INV_DEGREE, and if the inverse fit has a high error at this degree then the camera
	* will not use the inverse polynomial but instead find roots using Newton's method during projections (but this is slow).
	* @param v5PolyCoeffs The 5 coefficients of the 4th order camera polynomial
	* @param nSpecifiedDegree The degree of the inverse polynomial to fit. Set to -1 (or leave at default) to get a fit with maximum error less than dErrorLimit
	* @param dErrorLimit The maximum permissible error when attempting to find the best inverse polynomial fit 
	* @return The vector of coefficients of \f$ g \f$, ordered with the lowest degree first*/
	TooN::Vector<> FindInvPolyUsingRoots(const TooN::Vector<5>& v5PolyCoeffs, int nSpecifiedDegree = -1, double dErrorLimit = 0.1);
	
	/** @brief Finds the root of the camera polynomial 
	* 
	* Uses Newton's method to find the roots of the given 4th order camera model polynomial. To save on repeated work, the derivative also needs to be supplied.
	* Note that this doesn't find the roots of an arbitrary polynomial (even if it's degree 4), see the function PolyVal for the specific equation being evaluted.
	* This function will assert on nMaxIter, because if Newton's method doesn't find a root within this many iterations, either the starting position is way off
	* or something else is wrong, but in any case the whole system will work very poorly and slowly so you might as well stop and fix the problem.
	* @param v5Coeffs The coefficients of the 4th order camera polynomial
	* @param v4CoeffsDeriv The coefficients of the derivative of the polynomial
	* @param dTanTheta The tangent of the angle for which we want to find \f$ \rho \f$
	* @param dRhoInit The initial guess for \f$ \rho \f$
	* @param dErrorLimit The maximum error allowed in the solution
	* @param nMaxIter The maximum number of iterations allowed
	* @return The final \f$ \rho \f$ found */
	double FindRootWithNewton(TooN::Vector<5> v5Coeffs, TooN::Vector<4> v4CoeffsDeriv, double dTanTheta, double dRhoInit, double dErrorLimit = 0.01, int nMaxIter = 50);
	
	/// Find the normalized Gaussian value of a vector of values
	/** @param vxX Vector of values
	*  @param dMean Mean of Gaussian
	*  @param dStd Standard deviation of Gaussian */
	TooN::Vector<> CenterAndScale(TooN::Vector<> vxX, double dMean, double dStd);
	
	/// Find the normalized Gaussian value of a value
	/** @param dVal Value
	*  @param dMean Mean of Gaussian
	*  @param dStd Standard deviation of Gaussian */
	double CenterAndScale(double dVal, double dMean, double dStd);

	/// Evalute a polynomial with given coefficients at a specified location
	/** @param vxCoeff The vector of coefficients, ordered with the lowest degree first
	*  @param dEvalAt The x value where the polynomial will be evaluated
	*  @return The resulting y value */
	template<int Size, typename Precision, typename Base> double PolyVal(const TooN::Vector<Size, Precision, Base>& vxCoeff, double dEvalAt);

	/// Applies current settings by calculating new internal parameters
	void RefreshParams();
	
	bool PointInRectangle(TooN::Vector<2> v2Point, TooN::Vector<2> v2RectExtents);

	// Variables
	/** @brief The current camera parameters
	* 
	* The parameters (by index) are: \n
	* 0 - a0 coefficient \n
	* 1 - a2 coefficient \n
	* 2 - a3 coefficient \n
	* 3 - a4 coefficient \n
	* 4 - center of projection xc \n
	* 5 - center of projection yc \n
	* 6 - affine transform param c \n
	* 7 - affine transform param d \n
	* 8 - affine transform param e \n */
	TooN::Vector<9> mv9CameraParams; 	
	
	// Cached from the last project/unproject:
	TooN::Vector<3> mv3LastCam;      ///< Last point projected, in camera frame
	TooN::Vector<2> mv2LastIm;       ///< Last image coordinates
	TooN::Vector<2> mv2LastDistCam;  ///< Last distorted sensor plane coordinate (converted to mv2LastIm through affine transform and a translation)
	
	double mdLastRho;      ///< Last \f$ \rho \f$ value
	double mdLastCosPhi;   ///< Cosine of last \f$ \phi \f$
	double mdLastSinPhi;   ///< Sine of last \f$ \phi \f$

	// Cached from last RefreshParams:
	TooN::Vector<2> mv2Center;               ///< Camera center of projection //*//
	TooN::Matrix<2> mm2Affine;               ///< The affine transformation matrix
	TooN::Matrix<2> mm2AffineInv;            ///< Inverse of the affine transformation		
	
	TooN::Vector<2> mv2ImageSize;            ///< The current image size
	//TooN::Vector<2> mv2FullScaleSize;        ///< The area in unbinned pixels that the current image size takes up
	//TooN::Vector<2> mv2CalibSize;            ///< The image sized used when the camera was calibrated. Always in unbinned pixels
	TooN::Vector<5> mv5PolyCoeffs;           ///< The 4th order polynomial coefficients
	TooN::Vector<4> mv4PolyDerivCoeffs;      ///< The coefficients of the derivatives of the polynomial
	//TooN::Vector<5> mv5PolyDerivModCoeffs;   ///< Used for calculating projection derivatives
	
	bool mbInvalid;         ///< Was the last projection invalid?
	double mdMaxRho;        ///< Largest radius for which we consider projection valid
	
	double mdLargestRadius; ///< Largest radius in the image
	double mdMinTheta;      ///< Minimum ray incidence angle allowed for valid projection
	double mdOnePixelAngle; ///< Angle covered by a single pixel at the center of the image
	
	// Related to inverting the polynomial
	bool mbUsingInversePoly;    ///< True if good inverse polynomial found, false otherwise
	TooN::Vector<2> mv2LinearInvCoeffs;  ///< Linear inverse fit coefficient to use for getting starting point for Newton's method
	TooN::Vector<TooN::Resizable> mvxPolyInvCoeffs;  ///< Inverse polynomial coefficients, ideally we'll use these
	double mdThetaMean;  ///< Used to scale theta for computing inverse polynomial to avoid bad conditioning
	double mdThetaStd;   ///< Used to scale theta for computing inverse polynomial to avoid bad conditioning	
};

#endif
