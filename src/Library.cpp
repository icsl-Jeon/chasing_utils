//
// Created by jbs on 21. 5. 11..
//

#include <chasing_utils/Library.h>

using namespace chasing_utils;
using namespace chasing_utils::lib;

/**
 * Parse paramters from ros node handle. The nhTarget might have ns
 * @param nhTarget
 */
LibraryParam::LibraryParam(const ros::NodeHandle &nhTarget) {

    nhTarget.getParam("is_load_mode",isLoadMode);
    nhTarget.getParam("verbose",verbose);
    nhTarget.getParam("horizon",horizon);
    string method;
    nhTarget.getParam("method",method);
    if (method == "discrete_integrator" )
        strategy = Strategy::DISCRETE_ACCEL_INTEGRATOR;
    else if (method == "skeleton_permutation")
        strategy = Strategy::SKELETON_PERMUTATION;
    else{
        ROS_ERROR("Wrong method! choose btw {discrete_integrator,skeleton_permutation }");
        abort();
    }

    string associationType;
    nhTarget.getParam("association",associationType);
    if ( associationType == "collision")
        association = AssociationMethod::COLLISION;
    else if (associationType == "occlusion")
        association  = AssociationMethod::OCCLUSION;
    else{
        ROS_ERROR("Wrong association! choose btw {collision,occlusion}");
        abort();
    }

    nhTarget.getParam("lib_folder",libFolder);

    nhTarget.getParam("traverse_grid/resolution",TG_resolution);
    nhTarget.getParam("traverse_grid/padding",TG_padding);

    nhTarget.getParam("offline/save",saveLib);
    nhTarget.getParam("offline/dx",TG_inspectionDx);
    nhTarget.getParam("offline/dy",TG_inspectionDy);

    nhTarget.getParam("discrete_integrator/speed/n_disc",DI_speedDisc);
    nhTarget.getParam("discrete_integrator/speed/min",DI_speedMin);
    nhTarget.getParam("discrete_integrator/speed/max",DI_speedMax);

    nhTarget.getParam("discrete_integrator/accel/x/n_disc",DI_accelXDisc);
    nhTarget.getParam("discrete_integrator/accel/x/min",DI_accelXMin);
    nhTarget.getParam("discrete_integrator/accel/x/max",DI_accelXMax);

    nhTarget.getParam("discrete_integrator/accel/y/n_disc",DI_accelYDisc);
    nhTarget.getParam("discrete_integrator/accel/y/min",DI_accelYMin);
    nhTarget.getParam("discrete_integrator/accel/y/max",DI_accelYMax);

    nhTarget.getParam("skeleton_permutation/weight_skeleton",SP_weightSampleDeviation);
    nhTarget.getParam("skeleton_permutation/dt",SP_timeInterval);
    nhTarget.getParam("skeleton_permutation/t0",SP_oscGenStartTime);
    int polyorder;
    nhTarget.getParam("skeleton_permutation/poly_order",polyorder); SP_polyOrder = polyorder;
    nhTarget.getParam("skeleton_permutation/max_step",SP_maxTimeStep);
    nhTarget.getParam("skeleton_permutation/azimuth/step",SP_azimStep);
    nhTarget.getParam("skeleton_permutation/elevation/step",SP_elevStep);
    float angDeg;
    nhTarget.getParam("skeleton_permutation/elevation/min",angDeg); SP_elevMin = angDeg * M_PI / 180.0f;
    nhTarget.getParam("skeleton_permutation/elevation/max",angDeg); SP_elevMax = angDeg * M_PI / 180.0f;
    nhTarget.getParam("skeleton_permutation/radius/step",SP_radStep);
    nhTarget.getParam("skeleton_permutation/radius/min",SP_radMin);
    nhTarget.getParam("skeleton_permutation/radius/max",SP_radMax);

    nhTarget.getParam("visualization/draw_start_time",VIS_drawStartTime);
    nhTarget.getParam("visualization/ref_traj/r",VIS_colorRefTraj.r);
    nhTarget.getParam("visualization/ref_traj/g",VIS_colorRefTraj.g);
    nhTarget.getParam("visualization/ref_traj/b",VIS_colorRefTraj.b);
    nhTarget.getParam("visualization/ref_traj/a",VIS_colorRefTraj.a);
    nhTarget.getParam("visualization/ref_traj/w",VIS_widthRefTraj);

    nhTarget.getParam("visualization/candid_traj/r",VIS_colorCandidTraj.r);
    nhTarget.getParam("visualization/candid_traj/g",VIS_colorCandidTraj.g);
    nhTarget.getParam("visualization/candid_traj/b",VIS_colorCandidTraj.b);
    nhTarget.getParam("visualization/candid_traj/a",VIS_colorCandidTraj.a);
    nhTarget.getParam("visualization/candid_traj/w",VIS_widthCandidTraj);
    nhTarget.getParam("visualization/candid_traj/dN",VIS_strideCandidTraj);

    nhTarget.getParam("visualization/feasible_traj/r",VIS_colorFeasibleTraj.r);
    nhTarget.getParam("visualization/feasible_traj/g",VIS_colorFeasibleTraj.g);
    nhTarget.getParam("visualization/feasible_traj/b",VIS_colorFeasibleTraj.b);
    nhTarget.getParam("visualization/feasible_traj/a",VIS_colorFeasibleTraj.a);
    nhTarget.getParam("visualization/feasible_traj/w",VIS_widthFeasibleTraj);
    nhTarget.getParam("visualization/feasible_traj/dN",VIS_strideFeasibleTraj);

    nhTarget.getParam("visualization/traverse_grid/association/r",VIS_travAssociation.r);
    nhTarget.getParam("visualization/traverse_grid/association/g",VIS_travAssociation.g);
    nhTarget.getParam("visualization/traverse_grid/association/b",VIS_travAssociation.b);
    nhTarget.getParam("visualization/traverse_grid/association/a",VIS_travAssociation.a);

    nhTarget.getParam("visualization/traverse_grid/non_association/r",VIS_travNonAssociation.r);
    nhTarget.getParam("visualization/traverse_grid/non_association/g",VIS_travNonAssociation.g);
    nhTarget.getParam("visualization/traverse_grid/non_association/b",VIS_travNonAssociation.b);
    nhTarget.getParam("visualization/traverse_grid/non_association/a",VIS_travNonAssociation.a);
}


/**
 * return all polynomial primitives tracking the pointchain
 * @return (polyOrder + 1) x nPermutation Matrix whose column is polynomial coefficients
 * + array of tag chain associated with each polynomial primitive
 */

vector<observer::ObserverTagChain> Library::getPermutationPolynomial(MatrixXd &coeffX, MatrixXd &coeffY,
                                                                     MatrixXd &coeffZ) {

    double *pointData[DIM];
    vector<observer::ObserverTagChain> vecTagArray = SP_oscPtr->getPermutationPointChain(pointData[0], pointData[1], pointData[2]);

    MatrixXd kktRHS[DIM];
    int polyOrder = param.SP_polyOrder;
    PolynomialHelper helper(polyOrder);
    int camSizePerStep = SP_oscPtr->getCurCamSizePerStep();
    int curChainSize = SP_oscPtr->getCurChainSize();
    Traj baseChain = SP_oscPtr->getBaseChain();

    int nTraj = pow(camSizePerStep,curChainSize);

    for (auto & d : kktRHS){
        d = MatrixXd(polyOrder+1,nTraj);
    }

    // ! Computing KKT condition to obtain polynomial family
    MatrixXd kktMat = 2*helper.intDerSquared(2,param.horizon) / pow(param.horizon, 2*2-1);
    for (uint m = 0 ; m < curChainSize ; m++ ){
        double tn = baseChain.ts[m];
        for(uint i = 0 ; i <polyOrder+1 ; i++)
            for (uint j = 0; j <polyOrder+1 ; j++)
                kktMat(i,j) += 2*param.SP_weightSampleDeviation*pow(tn,i)*pow(tn,j); // todo variation in weight
    }


    MatrixXd Tterm(polyOrder + 1,curChainSize);
    for (int m = 0;m< curChainSize;m++)
        Tterm.col(m) = helper.tVec(baseChain.ts[m], 0);

    for (uint d = 0 ; d<DIM ;d++) {
        MatrixXd pntMat =  Map<MatrixXd>(pointData[d],curChainSize,nTraj);
        kktRHS[d] = 2 * param.SP_weightSampleDeviation * Tterm * pntMat;

    }

    // Update canddiate polynomials
    coeffX = kktMat.llt().solve(kktRHS[0]);
    coeffY = kktMat.llt().solve(kktRHS[1]);
    coeffZ = kktMat.llt().solve(kktRHS[2]);

//    MatrixXd kktMatInv = kktMat.inverse();
//    coeffX = kktMatInv * kktRHS[0];
//    coeffY = kktMatInv * kktRHS[1];
//    coeffZ = kktMatInv * kktRHS[2];

    return vecTagArray;

}

/**
 * Generate polyArr given strategy and horizon
 * @return points to be included in traverse grid. This is used for sizing in the initialization
 */
PointSet Library::generateTrajectory() {
    PointSet includePts; float tSample = 0.2;
    VectorXd ts; ts.setLinSpaced(int(param.horizon/tSample),0,param.horizon);
    /**
     * MODE 1 : DI method: start from origin (x-forwarding),
     * generate polynomials using constant acceleration model
     */
    if (param.strategy == Strategy::DISCRETE_ACCEL_INTEGRATOR) {
        nTraj = param.DI_speedDisc * param.DI_accelXDisc * param.DI_accelYDisc;
        polyArr = new PolynomialXYZ[nTraj];

        int nDiscSpeed = param.DI_speedDisc;
        float speedMin = param.DI_speedMin;
        float speedMax = param.DI_speedMax;
        VectorXf speedSet(nDiscSpeed);
        speedSet.setLinSpaced(nDiscSpeed, speedMin, speedMax);

        int nDiscAccelX = param.DI_accelXDisc;
        float accelXmin = param.DI_accelXMin;
        float accelXmax = param.DI_accelXMax;
        VectorXf accelSetX(nDiscAccelX);
        accelSetX.setLinSpaced(nDiscAccelX, accelXmin, accelXmax);

        int nDiscAccelY = param.DI_accelYDisc;
        float accelYmin = param.DI_accelYMin;
        float accelYmax = param.DI_accelYMax;
        VectorXf accelSetY(nDiscAccelY);
        accelSetY.setLinSpaced(nDiscAccelY, accelYmin, accelYmax);

        int polyOrder = 2;
        Point generatorOrig; // set to zero
        Point generatorDir(1.0, 0, 0); // x-forward
        VectorXd coeffX(polyOrder + 1);
        VectorXd coeffY(polyOrder + 1);
        VectorXd coeffZ(1); //! z-axis no acceleration

        int trajIdx= 0;
        for (int s = 0; s < nDiscSpeed; s++)
            for (int ax = 0; ax < nDiscAccelX; ax++)
                for (int ay = 0; ay < nDiscAccelY; ay++) {
                    // decide target polynomial
                    coeffX << generatorOrig.x, generatorDir.x * speedSet(s), 0.5 * accelSetX(ax);
                    coeffY << generatorOrig.y, generatorDir.y * speedSet(s), 0.5 * accelSetY(ay);
                    coeffZ << 0;

                    polyArr[trajIdx].px = Polynomial(coeffX);
                    polyArr[trajIdx].py = Polynomial(coeffY);
                    polyArr[trajIdx].pz = Polynomial(coeffZ);

                    //! Append include points from sparse sampling
                    includePts.push_back(polyArr[trajIdx].evalPntSet(ts));
                    trajIdx++;
                }

    /**
     *  MODE 2 : SP method
     *  1) Spawn OSC around polyRefPtr and 2) generate polynomials the OS per step
     */
    }else if (param.strategy == Strategy::SKELETON_PERMUTATION){
        if (param.SP_polyRefPtr == NULL){
            ROS_ERROR("SP method was selected but no ref trajectory.");
            abort();
        }

        Traj refTraj = param.SP_polyRefPtr->toTraj(0,param.horizon,15); // # of sample is just arbitrary
        SP_oscPtr = new observer::OSC(observer::spawnOscAroundTraj(param.getObserverParam(),refTraj,
                                     param.SP_oscGenStartTime, param.horizon,
                                     param.SP_timeInterval,param.SP_maxTimeStep));

        MatrixXd coeffPoly[DIM];
        vector<observer::ObserverTagChain> tagChainArr =
                getPermutationPolynomial(coeffPoly[0],coeffPoly[1],coeffPoly[2]);
        nTraj = tagChainArr.size();
        polyArr = new PolynomialXYZ[nTraj];

        for (int trajIdx = 0; trajIdx < nTraj; trajIdx ++){
            polyArr[trajIdx].px = Polynomial(coeffPoly[0].col(trajIdx));
            polyArr[trajIdx].py = Polynomial(coeffPoly[1].col(trajIdx));
            polyArr[trajIdx].pz = Polynomial(coeffPoly[2].col(trajIdx));
            includePts.push_back(polyArr[trajIdx].evalPntSet(ts));
        }
    }else{
        ROS_ERROR("Library Unknown method %d for the generation of library",param.strategy);
    }

    return includePts;
}

Library::Library(LibraryParam param, int index): index(index),param(param) {

    if (param.SP_polyRefPtr == NULL and param.strategy == Strategy::SKELETON_PERMUTATION){
        ROS_ERROR_STREAM( getMyName() << " skeleton permutation selected without ref trajectory");
        return;
    }

    /**
     * Generate polyArr from parameter and strategy
     */
    PointSet includePnts = generateTrajectory(); // points to be must-included in traverseGrid

    /**
     *  Construct traverseGrid
     */
    bool isValidField = false;
    ROS_INFO("Generating TG: nTraj=%d / include-points=%d",nTraj,includePnts.size());
    BooleanGridParam gridParam(nTraj, param.TG_resolution, includePnts,
                               param.TG_padding, param.TG_padding,
                               numeric_limits<float>::lowest(),numeric_limits<float>::max(),
                               isValidField);

    traverseGridPtr = new TraverseGrid(gridParam);
    ind3 sizeGrid = traverseGridPtr->getMaxAxis();

    if (param.verbose){
        printf("Generated %s with (%d, %d, %d) grid and %d trajectory.\n",
               getMyName().c_str(),sizeGrid.x,sizeGrid.y,sizeGrid.z,nTraj);

    }

    string file_name = getMyName();
    file_name =param.libFolder+ "/"  + file_name;
    cout << "reading " << file_name << endl;
    Timer timer;

    /**
     * Determine association (traj,inspection points)
     */
    //! MODE 1 : load traverse grid if isLoadMode
    if (param.isLoadMode){
        cout << getMyName() << " is called in load mode.." << endl;
        FILE* pFile = fopen(file_name.c_str(),"r");
        if (pFile  == NULL) {
            cerr << "file " <<file_name << " not opened. Not be loaded and aborted." << endl;
            return;
        }
        int* nAssociation = new int[1];
        fread(nAssociation,sizeof(int),1,pFile); // head size of true
        cout << "reading header found:  " << *nAssociation << " associations."<< endl;
        traverseGridPtr->setNumAssociation(*nAssociation);
        associationLinearInd = vector<int>(*nAssociation);
        fread(associationLinearInd.data(),sizeof(int),*nAssociation,pFile);

        //! The binary read indicates true cell in linear index
        for (int i = 0 ; i < *nAssociation; i++){
            int linIdx = associationLinearInd[i];
            ind4 subIdx = traverseGridPtr->ind2sub(linIdx);
            traverseGridPtr->set(subIdx,true);
        }
        
        // Find cells having non-zero association with trajectory
        traverseGridPtr->computeGridTrajPairSet();
        traverseGridPtr->computeTrajGridPairSet();

        gridTrajPairSet = traverseGridPtr->getGridTrajPairSet();
        trajGridPairSet = traverseGridPtr->getTrajGridPairSet();

        cout << "read complete in "<<  timer.stop() << " ms" << endl;

    //! MODE 2: construct association by inspection points and save binary
    }else {
        cout << getMyName() << " is called in write mode.." << endl;
        for (int i = 0; i < nTraj; i++) {
            PointSet inspectionPnts = getInspectionPoints(polyArr[i]);
            bool isCorrect;
            traverseGridPtr->markTraversed(inspectionPnts, i, isCorrect);
            if (not isCorrect)
                cout << "some of inspection points are out of grid" << endl;
        }
        //! record association index
        int nTravCell = 1;
        int nx = sizeGrid.x, ny = sizeGrid.y, nz = sizeGrid.z;
        int* travIdxBuf = new int[nx*ny*nz*nTraj+1]; //! assumed that nTraj could be different per library in the future
        travIdxBuf[0] = traverseGridPtr->getNumAssociation(); // used as read hint (or header)
        for (int trajIdx =0 ; trajIdx < nTraj ; trajIdx ++){
            for (int z = 0 ; z < nz ;  z++ )
                for (int y = 0 ; y < ny  ; y++)
                    for (int x = 0 ; x < nx ; x++ ){
                        ind3 ind (x,y,z);
                        if (traverseGridPtr->get(ind,trajIdx))
                            travIdxBuf[nTravCell++] = traverseGridPtr->sub2ind(ind4(ind,trajIdx));
                    }
        }
        cout << "number of association: " << traverseGridPtr->getNumAssociation()<< endl;
        associationLinearInd = vector<int> (travIdxBuf,travIdxBuf+nTravCell);

        //! save to directory
        if (param.saveLib) {
            cout << getMyName() << " save requested at  " << file_name.c_str()  << endl;
            FILE* pFile = fopen(file_name.c_str(),"w");
            if (pFile  == NULL) {
                cerr << file_name << " not opened. Not be saved" << endl;
                return;
            }
            fwrite(travIdxBuf,sizeof(int),nTravCell,pFile);
            fclose(pFile);
            delete[] travIdxBuf;
        }
    }

    libInitialized = true;
}

/**
 * For a polynomial, generate inspection points based on association method
 * @param poly for this polynomial, we determine the association points on the traverse grid
 * @return
 * @bug the last point might not be captured to inspection points
 */
PointSet Library::getInspectionPoints(const PolynomialXYZ &poly) const {
    float dt = 0.05; // should be small enough
    float t = 0.0;
    PointSet inspectionPoints;
    Point lastKnot = poly.evalPnt(t);
    inspectionPoints.push_back(lastKnot);

    while(t <= param.horizon){
        t += dt;
        Point pnt =  poly.evalPnt(t); // point on the trajectory

        if (pnt.distTo(lastKnot) >= param.TG_inspectionDx) {
            // self collision association
            if (param.association == AssociationMethod::COLLISION) {
                inspectionPoints.push_back(pnt);
                lastKnot = pnt;
                inspectionPoints.push_back(lastKnot);

            // bearing occlusion association
            } else if (param.association == AssociationMethod::OCCLUSION) {
                Point pntOnRefTraj = param.SP_polyRefPtr->evalPnt(t);
                LineSegment line(pnt,pntOnRefTraj);
                inspectionPoints.push_back(line.samplePoints(param.TG_inspectionDy,false));
            } else{
                ROS_ERROR("Library: unknown association. choose either collision or occlusion ");
            }
        }
    }
    return inspectionPoints;
}

visualization_msgs::MarkerArray Library::getMarkerCandidTraj(string frameId) const {
    vector<int> visIdx;
    for (int n = 0 ; n < nTraj; n+= param.VIS_strideCandidTraj)
        visIdx.push_back(n);

    return getMarkerCandidTraj(frameId,visIdx);
}

/**
 * Visualize the candid traj (polyArray) of trajIdx set. The other index <= nTraj will be used for clearing
 * @param frameId library ref tf
 * @param trajIdx
 * @return
 */
visualization_msgs::MarkerArray Library::getMarkerCandidTraj(string frameId, const vector<int>& trajIdx) const {
    vector<std_msgs::ColorRGBA> colors(trajIdx.size());
    for (int i = 0; i < trajIdx.size() ; i++){
        colors[i] = param.VIS_colorCandidTraj;
    }
    return getMarkerCandidTraj(frameId,trajIdx,colors);
}


/**
 * Visualize the candid traj (polyArray) of trajIdx set. The other index <= nTraj will be used for clearing
 * @param frameId library ref tf
 * @param trajIdx
 * @return
 */
visualization_msgs::MarkerArray Library::getMarkerCandidTraj(string frameId, const vector<int>& trajIdx,
                                                             const vector<std_msgs::ColorRGBA>& colors) const {
    visualization_msgs::MarkerArray markerArray;
    string ns = getMyName() + "/candid_traj";
    auto it = trajIdx.begin();
    auto itColor = colors.begin();

    int nKnot = *it; // index to be visualized.

    for(int n = 0 ; n < nTraj ; n++){
        if (n==nKnot){ // visualized
            Traj traj = polyArr[n].toTraj(param.VIS_drawStartTime,param.horizon,0.1f);
            auto lineStrip = traj.getLineStrip(*itColor++,param.VIS_widthCandidTraj,
                                               frameId,ns,n); // candidate starts from 1

            nKnot = *(++it); markerArray.markers.push_back(lineStrip);

        }else{ // clearing marker
//            markerArray.markers.push_back(getClearingMarker(frameId,ns,n));
        }
    }
    return markerArray;
}


visualization_msgs::Marker Library::getMarkerRefTraj(string frameId) const {


    if ( not param.strategy ==  Strategy::SKELETON_PERMUTATION){
        cerr << "ref marker requested not under permutation sekelton" << endl;
        return visualization_msgs::Marker();
    }
    string ns = getMyName() + "/ref_traj";
    auto traj = param.SP_polyRefPtr->toTraj(param.VIS_drawStartTime,param.horizon,0.1f);
    return traj.getLineStrip(param.VIS_colorRefTraj,param.VIS_widthRefTraj,
                             frameId,ns,0); // candidate starts from 1

}
visualization_msgs::Marker Library::getMarkerTraverseGrid(string frameId)  const {


    string ns = getMyName() + "/traverse_grid";
    return traverseGridPtr->visualizeAllMarker(frameId,param.VIS_travAssociation,
                                        param.VIS_travNonAssociation,ns);

}

visualization_msgs::MarkerArray Library::getMarkerTotal(string frameId) const  {
    visualization_msgs::MarkerArray markerArray  = getMarkerCandidTraj(frameId);
    if (param.strategy == Strategy::SKELETON_PERMUTATION) {
        // osc points


        // ref traj
        markerArray.markers.push_back(getMarkerRefTraj(frameId));
    }
    markerArray.markers.push_back(getMarkerTraverseGrid(frameId));
    return markerArray;
}

/**
 * evaluate location in the trajectory library
 * @param trajIdx
 * @param evalTime
 * @return point in world coordinate
 */
Point LibraryOnline::eval(int trajIdx, ros::Time evalTime) const {
    assert (trajIdx >= 0 and trajIdx < getNumTraj());
    double t = (evalTime - libRefTime).toSec();
    return libRefPose.poseMat * getPolyPtrAt(trajIdx)->evalPnt(t).toEigen();
}

LibraryOnline::LibraryOnline(LibraryParam param, int index) : Library(param,index){

    costs.resize(getNumTraj());
    feasibility.resize(getNumTraj());
    for (int n = 0 ; n < getNumTraj() ; n++){
        costs[n] = numeric_limits<float>::quiet_NaN();
        feasibility[n] = true;
        feasibleTraj.push_back(n);
    }
}

void LibraryOnline::registerCost(int trajIdx, float cost) {
    costs[trajIdx] = cost;
}


/**
 * Renew the feasibility by iterate over traverse grid
 * @param feasibilityParam used to determine feasibility of trajectory using associated cell
 * @param feasibleIndex returns feasible trajectory index
 * @return
 * @details using gridTrajPairSet which is list of <cell, vector of its associated trajectories>
 */
bool LibraryOnline::registerFeasibility(FeasibilityParam feasibilityParam, vector<int> &feasibleIndex) {

    if (feasibilityParam.edtServerPtr == NULL) {
        ROS_ERROR("LibraryOnline: checking feasibility not possible as no edt given.  ");
        return false;
    }

    // initialize
    feasibleIndex.clear();
    cellMarker.clear();
    for (int n = 0 ; n < getNumTraj() ; n++)
        feasibility[n] = true;

    // First check cell occupancy (for fast lock and unlock)
    // occupied = true / free = false
    int nCellWithAssociations = gridTrajPairSet.size();
    vector<bool> cellOccupancy (nCellWithAssociations);
    feasibilityParam.edtServerPtr->getLocker().lock();
    int m = 0;
    for (auto elem: gridTrajPairSet){
        ind3 cell = elem.first;
        Point pnt_l = traverseGridPtr->ind2pnt(cell);
        Vector3f pnt_w = libRefPose.poseMat * pnt_l.toEigen();
        octomap::point3d pnt(pnt_w.x(),pnt_w.y(),pnt_w.z());
        double dist = feasibilityParam.edtServerPtr->getDistance(pnt);
        if (dist < feasibilityParam.margin)
            cellOccupancy[m] = true;
        else{
            // update marker for only non-colliding cells
            pcl::PointXYZI pclPnt;
            pclPnt.x = pnt_w.x();
            pclPnt.y = pnt_w.y();
            pclPnt.z = pnt_w.z();
            pclPnt.intensity = dist;
            cellMarker.push_back(pclPnt);
            cellOccupancy[m] = false;
        }
        m++;
    }
    feasibilityParam.edtServerPtr->getLocker().unlock();

    // Update feasibility of trajectories using occupancy of traverseGrid
    for (int m = 0; m < nCellWithAssociations ; m++){
        if (cellOccupancy[m]) // if true, all traj associated with this cell -> infeasible
            for (int n : gridTrajPairSet[m].second)
                feasibility[n] = false;
    }
    for (int n = 0 ; n < getNumTraj() ; n++)
        if (feasibility[n])
            feasibleIndex.push_back(n);
    feasibleTraj = feasibleIndex;
    return true;
}

/**
 * Check feasibility of the single trajectory in the library traversing associated cells
 * @param feasibilityParam
 * @param trajIdx inspection index
 * @return true if feasible
 */
bool LibraryOnline::checkTrajectoryFeasibility(FeasibilityParam feasibilityParam, int trajIdx) const {

    if (trajIdx >= getNumTraj()){
        ROS_ERROR("LibraryOnline: trajectory index %d out of index < %d.",trajIdx,getNumTraj());
        return false;
    }

    for (auto cell: trajGridPairSet[trajIdx].second){
        Point pnt_l = traverseGridPtr->ind2pnt(cell);
        Vector3f pnt_w = libRefPose.poseMat * pnt_l.toEigen();
        octomap::point3d pnt(pnt_w.x(),pnt_w.y(),pnt_w.z());
        double dist = feasibilityParam.edtServerPtr->getDistance(pnt);
        if (dist < feasibilityParam.margin)
            return false;
    }
    return true;
}



/**
 * Get individual trajectory and its global time w.r.t worldFrame using libRefPose and ref Time
 * @param trajIdx
 * @param traj
 * @param refTime
 */
void LibraryOnline::getTraj(int trajIdx, Traj &traj, ros::Time& refTime) const {
    refTime = libRefTime;
    PolynomialXYZ* polyPtr = getPolyPtrAt(trajIdx);

    string polyCoeffString;
    for (int n = 0; n<=polyPtr->px.getOrder() ; n++)
        polyCoeffString += to_string(polyPtr->px.coeff(n)) + ", ";
    ROS_DEBUG("px: %s ",polyCoeffString.c_str());
    Traj traj_l = getPolyTrajAt(trajIdx);
    traj_l.applyTransform(libRefPose);
    traj = traj_l;
}

/**
 * get visualization of feasible trajectory in library
 * @param
 * @return
 */
visualization_msgs::MarkerArray LibraryOnline::getMarkerFeasibleTraj(string frameId) const {
    vector<std_msgs::ColorRGBA> colors;
    std_msgs::ColorRGBA feasibleColor = param.VIS_colorFeasibleTraj;
    vector<int> visIdx;
    for(int n = 0; n < feasibleTraj.size() ; n+=param.VIS_strideFeasibleTraj){
        visIdx.push_back(feasibleTraj[n]);
        colors.push_back(feasibleColor);
    }
    return getMarkerCandidTraj(frameId,visIdx,colors);
}


/**
 * visualize EDT on traverse grid
 * @param worldFrameId
 * @return
 * @bug this should be called after register feasibility
 */
pcl::PointCloud<pcl::PointXYZI> LibraryOnline::getMarkerCells(string worldFrameId) const {
    auto markerCell = cellMarker;
    markerCell.header.frame_id = worldFrameId;
    markerCell.header.stamp = pcl_conversions::toPCL(ros::Time::now());
    return markerCell;
}






