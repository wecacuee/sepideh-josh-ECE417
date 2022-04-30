/*#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
*/

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>




// void onMouse(int event, int x, int y, int flags, void *userdata) {

//     std::vector<Eigen::Vector3d>* points_clicked =
//         static_cast<std::vector<Eigen::Vector3d>* >(userdata);
//     if (cv::EVENT_LBUTTONUP == event) {
//         Eigen::Vector3d pt; pt << x, y, 1;
//         std::cout << "u = [" << pt << "]\n";
//         points_clicked->push_back(pt);
//     }
// }

// // here would need to replace points_clicked from using a mouse callback to using the auto-detect library
// std::vector<Eigen::Vector3d>
// collectPoints(const cv::Mat& img) {
//     cv::imshow("img", img);
//     std::vector<Eigen::Vector3d> points_clicked;
//     cv::setMouseCallback("img", onMouse, &points_clicked);
//     std::cout << "Please click on 4 points\n";
//     cv::waitKey(-1);
//     return points_clicked;
// }


//ms need to be real-world measurements
Eigen::Matrix3d
findHomography(const std::vector<Eigen::Vector3d>& Ms,
               const std::vector<Eigen::Vector3d>& ms)
{
    if (Ms.size() < 4 || ms.size() < 4) {
        std::cerr << "Need at least four points got " << Ms.size() << " and " << ms.size() << "\n";
        throw std::runtime_error("Need atleast four points");
    }
    std::cout << "Size of Ms and ms is: " << Ms.size() << " "<< ms.size() << "\n";
    std::cout << "Ms is: " << Ms[3] << "\n";
    std::cout << "ms is: " << ms[3] << "\n";

    Eigen::MatrixXd A(2*Ms.size(), 9); A.setZero();

    for (int i = 0; i < Ms.size(); ++i) {
        if (i < 4)
            std::cout << "Computing homography using points from M_i = [" << Ms[i].transpose() << "] to m_i = [" << ms[i].transpose() << "]\n";
        // [[Mᵀ       0    -uᵢMᵀ]]
        //  [0        Mᵀ   -vᵢMᵀ]]
        A.block(2*i, 0, 1, 3) = Ms[i].transpose();// TODO Replace this with correct formula
        A.block(2*i, 6, 1, 3) = -ms[i](0) * Ms[i].transpose(); // TODO Replace this with correct formula 
        A.block(2*i+1, 3, 1, 3) = Ms[i].transpose(); // TODO Replace this with correct formula
        A.block(2*i+1, 6, 1, 3) = -ms[i](1) * Ms[i].transpose(); // TODO Replace this with correct formula
    }    

    //Gets V from USV
    auto svd = A.jacobiSvd(Eigen::ComputeFullV);
    // y = v₉
    // Eigen::VectorXd nullspace = svd.matrixV().row(8); // TODO Replace this with correct formula
    Eigen::VectorXd nullspace = svd.matrixV().col(8);//.transpose(); // TODO Replace this with correct formula
    std::cout << "Rows and cols" << nullspace.rows() << nullspace.cols() << "\n";
    Eigen::Matrix3d H;

    H.row(0) << nullspace.block(0,0,3,1).transpose();
    H.row(1) << nullspace.block(3,0,3,1).transpose();
    H.row(2) << nullspace.block(6,0,3,1).transpose(); 
    return H;
}


bool
testHomographyFit(const std::vector<Eigen::Vector3d>& Ms,
                  const std::vector<Eigen::Vector3d>& ms,
                  const Eigen::Matrix3d& H)
{
    for (int i=0; i < Ms.size(); ++i) {
        Eigen::Vector3d ms_got = H * Ms[i];
        ms_got /= ms_got(2);
        if (ms_got.isApprox(ms[i], 1e-2)) {
            std::cout << "Good fit for " << i << "\n";
        } else {
            std::cout << "Bad fit for " << i << "\n";
            std::cout << "ms_got[" << i << "] = " << ms_got << "\n";
            std::cout << "ms[" << i << "] = " << ms[i] << "\n";
            return false;
        }
    }
    return true;
}

Eigen::MatrixXd
applyHomography(const Eigen::Matrix3d& H,
                const Eigen::MatrixXd& img) {
    Eigen::MatrixXd new_img(img.rows(), img.cols());
    Eigen::Vector3d u;
    Eigen::Vector3d up;
    for (int new_row = 0; new_row < new_img.rows(); ++new_row) {
        for (int new_col = 0; new_col < new_img.cols(); ++new_col) {
            u << new_col + 0.5, new_row + 0.5, 1;
            /**** Apply homography for each pixel ***/
            // u' = H * u
            up = H * u;  // TODO replace with correct formula
            up /= up(2);
            /**** Apply homography for each pixel ***/
            int row = round(up(1));
            int col = round(up(0));
            if (0 <= row && row < img.rows()
                && 0 <= col && col < img.cols()) {
                new_img(new_row, new_col) = img(row, col);
            }
        }
    }
    return new_img;
}

void eigen_imshow(const Eigen::MatrixXd& eigen_new_img) {
    cv::Mat cv_new_img;
    cv::eigen2cv(eigen_new_img, cv_new_img);
    cv_new_img.convertTo(cv_new_img, CV_8U);
    cv::imshow("new_img", cv_new_img);
    cv::waitKey(-1);
}

Eigen::VectorXd generate_vij(const Eigen::Matrix3d& H, int i, int j) {
    Eigen::Vector3d hi = H.col(i);
    Eigen::Vector3d hj = H.col(j);
    Eigen::VectorXd vij(6);
    vij << hi(0) * hj(0),            // B_11,
        hi(0)*hj(1) + hi(1) * hj(0), // B_12
        hi(1)*hj(1),                 // B_22
        hi(0)*hj(2) + hi(2)*hj(0),   // B_13
        hi(1)*hj(2) + hi(1)*hj(2),   //  B_23
        hi(2)*hj(2) ; //B_33
    return  vij;
}

// std::vector<Eigen::Matrix4d>
Eigen::Matrix3d
Estimate_B(const std::vector<Eigen::Matrix3d>& Hs){
    
    int n = Hs.size();
    std::cout<<"n is "<<n <<"\n";

    std::vector<Eigen::Matrix4d> Ts;
    Eigen::MatrixXd V(2*n,6);
    Eigen::Matrix3d A;
    A.setZero();
    V.setZero();
    int i=0;
    for (int i=0;i<n;i++){

        Eigen::Matrix3d H = Hs[i];

        Eigen::VectorXd v11 = generate_vij(H, 0, 0);
        Eigen::VectorXd v12 = generate_vij(H, 0, 1);
        Eigen::VectorXd v22 = generate_vij(H, 1, 1);
        Eigen::VectorXd v1122 = v11 - v22;
        // std::cout<<H(0,0)*H(1,0)<<"\n";

        /// //v12(0)=H(0,0)*H(1,0);
        /// //,(H(0,0)*H(1,1))+(H(0,1)*H(1,0));
        /// // Other than that all of the equations appear correct based on my understanding of the notation in the paper
        /// v12<< H(0,0)*H(1,0), // B_00
        ///     (H(0,0)*H(1,1))+(H(0,1)*H(1,0)), // B_01
        ///     H(0,1)*H(1,1),  // B_11
        ///     (H(0,2)*H(1,0))+(H(0,0)*H(1,2)), // B_
        ///     (H(0,2)*H(1,1))+(H(0,1)*H(1,2)),
        ///     H(0,2)*H(1,2);
        /// v12 = 

        /// v11<< H(0,0)*H(0,0),(H(0,0)*H(0,1))+(H(0,1)*H(0,0)),H(0,1)*H(0,1),
        /// (H(0,2)*H(0,0))+(H(0,0)*H(0,2)), (H(0,2)*H(0,1))+(H(0,1)*H(0,2)), H(0,2)*H(0,2);

        /// v22<< H(1,0)*H(1,0),(H(1,0)*H(1,1))+(H(1,1)*H(1,0)),H(1,1)*H(1,1),
        /// (H(1,2)*H(1,0))+(H(1,0)*H(1,2)), (H(1,2)*H(1,1))+(H(1,1)*H(1,2)), H(1,2)*H(1,2);
        
        int s = V.size();
        //std::cout<<v12;
        V.row(2*i)<<v12.transpose();
        v1122=v11-v22;
        V.row((2*i)+1)<<v1122.transpose();
    }

    //std::cout<<V;
    auto svd = V.jacobiSvd(Eigen::ComputeFullV);
    std::cout << "rank of V: " << svd.rank() << "\n";
    Eigen::VectorXd b = svd.matrixV().col(5);
    std::cout<<"b is: \n" << b << "\n";
    
    
	//testing B
	Eigen::Matrix3d B;
	B.row(0)<<b(0),b(1),b(3);
	B.row(1)<<b(1),b(2),b(4);
	B.row(2)<<b(3),b(4),b(5);

    std::cout << "B = " << B << "\n";

    for (int i=0;i<n;i++){

        Eigen::Matrix3d H = Hs[i];
        std::cout<<"\n h1^T B h2  = "<<H.col(0).transpose()*B*H.col(1)<<"\n";
        std::cout<<"\n h1^T B h1 - h2^T H h2  = "<<H.col(0).transpose()*B*H.col(0)
            - H.col(1).transpose()*B*H.col(1)
                 <<"\n";
    }

    
    double mineigval = B.eigenvalues().real().minCoeff();
    std::cout << "min eig = "  << mineigval;
    B = B + std::abs(mineigval*2) *  Eigen::Matrix3d::Identity();
    //estimating A
    Eigen::LDLT<Eigen::Matrix3d>  lltOfB(B);
    std::cout << "B = " << B << "\n";
    Eigen::Vector3d dsqrt = lltOfB.vectorD().array().sqrt();
    Eigen::Matrix3d Dsqrt = Eigen::DiagonalMatrix<double, 3>(dsqrt);
    Eigen::Matrix3d L = lltOfB.matrixL() * Dsqrt;
    std::cout << "L = " << L << "\n";
    std::cout << "LLᵀ = " << L * L.transpose() << "̱\n";
    Eigen::Matrix3d Ap = L.inverse().transpose();
    // Ap /= Ap(2,2);
    std::cout << "Ap = " << Ap / Ap(2,2) << "\n";

    //v0 = (B12B13 − B11B23)/(B11B22 − B12^2)
    double v0=((b(1)*b(3))-(b(0)*b(4)))/((b(0)*b(2))-(b(1)*b(1)));
    std::cout<< "v0 = " << v0 << "\n";
    //λ = B33 − [B13^2 + v0(B12B13 − B11B23)]/B11
    double lambda=b(5)-( ( (b(3)*b(3)) + (v0*((b(1)*b(3)) - (b(0)*b(4)))) )/b(0) );
    std::cout<< "lambda is: " << lambda << "\n";
    //alfa=square(lambda/B11)
    double alfa=sqrt((lambda/b(0)));
    std::cout<<"alpha is: "<<lambda/b(0)<<"\n";
    //beta=square((lambda*B11)/((B11*B22)-(B12*B12)))
    double beta=sqrt(( (lambda*b(0)) / ( (b(0)*b(2)) - (b(1)*b(1)) ) ));
    std::cout<< "beta = " << beta << "\n";
    //gamma=-(B12*alfa*alfa*beta) / lambda
    double gamma=-(b(1)*alfa*alfa*beta) / lambda;
    //std::cout<<gamma;
    //u0=((lambda*v0)/beta) - ((B13*alfa*alfa) / lambda)
    double u0=((gamma*v0)/beta) - ((b(3)*alfa*alfa) / lambda);
    //std::cout<<u0;
    

    A.row(0)<< alfa, gamma,u0;
    A.row(1)<< 0, beta, v0;
    A.row(2)<< 0, 0, 1;
    
    std::cout<<"\nA is: " << Ap << "\n";

    Eigen::Matrix3d Apinv =  Ap.inverse();
    std::cout << "Apinv = " << Apinv << "\n";
    std::cout << "L = " << Apinv.transpose() << "\n";
    Eigen::Matrix3d Bp = lambda * Apinv.transpose() * Apinv;
    std::cout << "Bp = " << Bp << "\n";

    std::cout << "B = " << B << "\n";

    for (int i=0;i<n;i++){
        Eigen::Matrix3d H = Hs[i];
        std::cout<<"\n h1^T A⁻ᵀA⁻¹ h2  = "<<H.col(0).transpose()*Bp*H.col(1)<<"\n";
        std::cout<<"\n h1^T A⁻ᵀA⁻¹ h1 - h2^T A⁻ᵀA⁻¹ h2  = "<<H.col(0).transpose()*Bp*H.col(0)
            - H.col(1).transpose()*B*H.col(1)
                 <<"\n";
    }
        
    Eigen::Matrix4d T;  
    
    // for ( int i=0;i<n;i++){
    //     //λ = 1/||A−1h1|| = 1/||A−1h2||
    //     Eigen::Matrix3d H;
    //     H=Hs[i];
    //     Eigen::VectorXd vec=A.inverse()*H.col(0);
    //     double lambda2=1/(vec.norm());
    //     std::cout<<A;
    //     Eigen::VectorXd r1=lambda2*A.inverse()*H.col(0);
    //     Eigen::VectorXd r2=lambda2*A.inverse()*H.col(1);
    //     //Eigen::VectorXd r3=r1.cross(r2);
    //     //std::cout<<r1;
    //     //std::cout<<r2;
    //     //.cross(r2);
    //     Eigen::VectorXd t=lambda2*A.inverse()*H.col(2);
        
        
    //     //estimating Rotation matrix from r1,r2,r3
    //     Eigen::Matrix3d Q;
    //     Q.col(0)=r1;
    //     Q.col(1)=r2;
    //     //Q.col(2)=r3;
        
        
    //     //R=UV^T
    //     //JacobiSVD<Matrix3d, ComputeThinU | ComputeThinV> svd2(Q);
    //     auto svd2 = Q.jacobiSvd(Eigen::ComputeFullV|Eigen::ComputeFullU);
    //     Eigen::Matrix3d R;
    //     R=(svd2.matrixU())*(svd2.matrixV().transpose());
    //     T.block(0,0,3,3)=R;
    //     T.block(0,3,3,1)=t;
    //     T.row(4)<<0,0,0;
    //     Ts.push_back(T);
    // }
    // //Ts.push_back(A);
    // return Ts;//need to be corrected
    return A;
}

int main(int argc, char** argv) {

    // Hs stores each homography matrix in a vector, Ms are the model Eigen points
    std::vector<Eigen::Matrix3d> Hs;
    std::vector<Eigen::Vector3d> Ms;
    std::string fpath;
    std::cout << "Argc is: " << argc << "\n";

    // 8x6 inner corners on chess board
    cv::Size patternSize(8,6); 

    // Chessboard's internal model coordinates of the corners (with 23 mm square size)
    std::vector<cv::Point2f> corners_model;
    for( int i = 0; i < 6; i++ ) {
        for( int j = 0; j < 8; j++ ) {
            corners_model.push_back(cv::Point2f(float(j*0.023),float(i*0.023)));
        }
    }
    // Convert corners into Eigen types to perform homography math (may not be needed, can try to do math in native C++ types or cv types once it works)
    Eigen::Vector3d corner;
    for( int i = 0; i < corners_model.size(); i++ ) {
        corner << corners_model[i].x , corners_model[i].y, 1;
        Ms.push_back(corner);
    }



    int n = argc - 1;
    //Loop over images to calculate Hs
    for (int i=0; i < n; ++i) {
        std::cout << "For loop begun: " << i << "\n";                
        //get image path
        if (argc < (4)) {
            // std::cerr << "Need input image path \n";
            throw std::runtime_error("Not enough file names provided");
        } else {
            fpath = argv[1+i];
        }

        std::cout << "file path : " << fpath << "\n";

        // Ideally I should just resize and imwrite() the images once and remove the resize after, but this works for now (the images are like, 4000x3000 pixels for some reason)
        // Grayscale image used for getting corners, color used for displaying them with drawChessboardCorners, and eigen used for showing homography math application
        cv::Mat img = cv::imread(fpath, cv::IMREAD_GRAYSCALE);
        cv::Mat img2 = img.clone();
        //  cv::Mat img2 = cv::imread(fpath, cv::IMREAD_GRAYSCALE);
        // cv::resize(img, img, cv::Size(img.cols * 0.4,img.rows * 0.4), 0, 0, cv::INTER_LINEAR);
        // cv::resize(img2, img2, cv::Size(img2.cols * 0.4,img2.rows * 0.4), 0, 0, cv::INTER_LINEAR);

        cv::Mat imgcol;
        cv::cvtColor(img, imgcol, cv::COLOR_GRAY2BGR);

        cv::Mat img_64f;
        img2.convertTo(img_64f, CV_64F);
        Eigen::MatrixXd eigen_img;
        cv::cv2eigen(img_64f, eigen_img);

        // Image corners in cv and Eigen
        std::vector<Eigen::Vector3d> ms;
        std::vector<cv::Point2f> corners_image;

        // Gets cv image corner coordinates and then displays them. Click any button to move past it, and then close to move to the next image
        bool found = cv::findChessboardCorners(img, patternSize, corners_image);
        // std::cout << "Found is: "<<found<<"\n";
        // std::cout << img.type() <<"\n";
        // std::cout << corners_image.size() << "\n";
        cv::drawChessboardCorners(imgcol, patternSize, corners_image, found);
        cv::namedWindow("Chessboard corners detection",cv::WINDOW_NORMAL);
        cv::resizeWindow("Chessboard corners detection", 800, 600);
        // cv::imshow("Chessboard corners detection", imgcol);
        // cv::waitKey(-1);

        //! Add automatic close of image window here

        for( int k = 0; k < corners_image.size(); k++ ) {
                corner << corners_image[k].x , corners_image[k].y, 1;
                ms.push_back(corner);
        }

        if (ms.size() < 4) {
            std::cerr << "Need at least four points got " << ms.size() << "\n";
            throw std::runtime_error("Need at least four points");
        }

        // The lab part, using points to get H                    
        // std::vector<cv::Point2d> us_cv;
        // std::vector<cv::Point2d> ups_cv;
        // for (int i = 0; i < Ms.size(); ++i) {
        //     us_cv.emplace_back(Ms[i](0), Ms[i](1));
        //     ups_cv.emplace_back(ms[i](0),  Ms[i](1));
        // }
        // cv::Mat H_cv =  cv::findHomography(us_cv, ups_cv);
        // Eigen::Matrix3d H;
        // cv::cv2eigen(H_cv, H);
        Eigen::Matrix3d H = findHomography(Ms, ms);
        std::cout << " H: " << H << "\n";
        // Check that applying the H matrix to Ms gets ms
        testHomographyFit(Ms, ms, H);

        // storing H for future use after for loop
        Hs.push_back(H);
        std::cout << "H is" << H / H(2,2) << "\n";
        std::cout << "H scaled is" << H / H(2,2) << "\n";

        // apply the homography to the rest of the image and then show the resulting before and after images
        //  Eigen::MatrixXd eigen_new_img = applyHomography(H, eigen_img);
        //  Eigen::MatrixXd joined_img(std::max(eigen_img.rows(), eigen_new_img.rows()),
        //                             eigen_img.cols() + eigen_new_img.cols());
        //  joined_img.block(0, 0, eigen_img.rows(), eigen_img.cols()) = eigen_img;
        //  joined_img.block(0, eigen_img.cols(), eigen_new_img.rows(), eigen_new_img.cols()) =
        //      eigen_new_img;
        // eigen_imshow(joined_img);
        std::cout << "For loop ended: " << i << "\n";        
    }
    std::cout << "Hs collected, getting B\n";
    Eigen::Matrix3d A;
    A.setZero();
    A = Estimate_B(Hs);

    std::cout << "A at end is: " << A <<"\n";
    return 0;
}

















