//
// Created by jbs on 21. 3. 20..
//

#include <chasing_utils/Utils.h>

namespace chasing_utils {

    /**
     * Return jet color map
     * @param xNorm [0,1]
     * @param r [0,1]
     * @param g
     * @param b
     */
    void getColor(float xNorm,  float & r, float & g, float & b) {

        // Only important if the number of colors is small. In which case the rest is
        // still wrong anyway
        // x = linspace(0,1,jj)' * (1-1/jj) + 1/jj;
        //
        const double rone = 0.8;
        const double gone = 1.0;
        const double bone = 1.0;
        float x = xNorm;
        x = (x<0 ? 0 : (x>1 ? 1 : x));

        if (x<1. / 8.)
        {
            r = 0;
            g = 0;
            b = bone*(0.5 + (x) / (1. / 8.)*0.5);
        } else if (x<3. / 8.)
        {
            r = 0;
            g = gone*(x - 1. / 8.) / (3. / 8. - 1. / 8.);
            b = bone;
        } else if (x<5. / 8.)
        {
            r = rone*(x - 3. / 8.) / (5. / 8. - 3. / 8.);
            g = gone;
            b = (bone - (x - 3. / 8.) / (5. / 8. - 3. / 8.));
        } else if (x<7. / 8.)
        {
            r = rone;
            g = (gone - (x - 5. / 8.) / (7. / 8. - 5. / 8.));
            b = 0;
        } else
        {
            r = (rone - (x - 7. / 8.) / (1. - 7. / 8.)*0.5);
            g = 0;
            b = 0;
        }




    }


    visualization_msgs::Marker Path::get_point_marker(std_msgs::ColorRGBA color, string frame_id,int id) {
        visualization_msgs::Marker marker;
        marker.id = id;
        marker.type =visualization_msgs::Marker::SPHERE_LIST;
        marker.header.frame_id = frame_id;
        marker.pose.orientation.w =1.0;
        marker.scale.x = 0.6;
        marker.scale.y = 0.6;
        marker.scale.z = 0.6;
        marker.color = color;

        for (auto pnt : points){
            points.push_back(pnt);
        }
        return marker;
    }


    /**
     *
     * @param color
     * @param width
     * @param frame_id
     * @param ns namespace
     * @param id
     * @return
     */
    visualization_msgs::Marker Path::getLineStrip(std_msgs::ColorRGBA color, float width,
                                                  string frame_id, string ns, int id) {

        visualization_msgs::Marker lineStrip;
        lineStrip.type = visualization_msgs::Marker::LINE_STRIP;
        lineStrip.header.frame_id = frame_id;
        lineStrip.color = color;
        lineStrip.scale.x = width; // width
        lineStrip.action = visualization_msgs::Marker::MODIFY;
        lineStrip.pose.orientation.w = 1.0;
        lineStrip.ns = ns;
        lineStrip.id = id;

        for (auto pnt : points){
            lineStrip.points.push_back(pnt.toGeometry());
        }

        return lineStrip;
    }

    visualization_msgs::MarkerArray Path::get_bearing(visualization_msgs::Marker arrowBase, Path targetPath,int id) {
        visualization_msgs::MarkerArray arrowArray;
        int arrowStep = min(targetPath.points.size(),points.size());
        for(int n = 0 ; n < arrowStep ; n++){
            visualization_msgs::Marker arrow = arrowBase;
            arrow.points.push_back(points[n].toGeometry());
            arrow.points.push_back(targetPath.points[n].toGeometry());
            arrow.id = id + n ;
            arrowArray.markers.push_back(arrow);
        }
        return arrowArray;
    }


    float Traj::get_length(float t0, float tf) const {
        int nInnerStart,nInnerEnd;
        for (int n = 0 ; n < ts.size() ; n ++){
            if (t0 >= ts[n]){
                nInnerStart = n;
                break;
            }
        }
        for (int n = nInnerStart ; n < ts.size() ; n++){
            if (tf >= ts[n]){
                nInnerEnd = n;
                break;
            }
        }

        float length = 0; Point p0 = eval(t0);
        for (int n = nInnerStart ; n < nInnerEnd ; n++){
            length+=p0.distTo(points[n]);
            p0 = points[n];
        }
        length+= p0.distTo(eval(tf));
        return length;
    }

    pcl::PointCloud<pcl::PointXYZ> Path::get_point_pcl(string frame_id) {
        pcl::PointCloud<pcl::PointXYZ> pcl;
        for (auto pnt: points)
            pcl.points.push_back(pnt.toPCL());
        pcl.header.frame_id = frame_id;
        return pcl;
    }

    pcl::PointCloud<pcl::PointXYZI> Path::get_point_pcl_timecolor(string frame_id,int nStartColor ) {

        pcl::PointCloud<pcl::PointXYZI> pcl;
        for (int n = 0 ; n < points.size(); n++){
            pcl::PointXYZI pnt;
            pnt.x = points[n].x;
            pnt.y = points[n].y;
            pnt.z = points[n].z;
            pnt.intensity = n+nStartColor;
            pcl.push_back(pnt);
        }
        pcl.header.frame_id = frame_id;
        return pcl;
    }


    nav_msgs::Path Path::toNavPath(string frame_id, int strideStep){

        nav_msgs::Path path;
        path.header.frame_id = frame_id;

        for (int n = 0 ; n < points.size() ; n += strideStep) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = points[n].x;
            pose.pose.position.y = points[n].y;
            pose.pose.position.z = points[n].z;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        return path;
    }
    /**
     * Transform all points w.r.t frome {1}
     * @param T01
     */
    void Path::applyTransform(Pose T01) {
        for (auto& pnt: points){
            auto pntTransformed = T01.poseMat * pnt.toEigen();
            pnt = Point(pntTransformed);
        }
    }



    float interpolate(const vector<float>& xData,const vector<float>& yData, const float& x,bool extrapolate){


        int size = xData.size();
        int i = 0;                                                                  // find left end of interval for interpolation
        if ( x >= xData[size - 2] )                                                 // special case: beyond right end
        {
            i = size - 2;
        }
        else
        {
            while ( x > xData[i+1] ) i++;
        }
        double xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];      // points on either side (unless beyond ends)
        if ( !extrapolate )                                                         // if beyond ends of array and not extrapolating
        {
            if ( x < xL ) {yR = yL; }
            if ( x > xR ) yL = yR;
        }

        double dydx = ( yR - yL ) / ( xR - xL );                                    // gradient
        return yL + dydx * ( x - xL );                                              // linear interpolation
    }
    /**
     * extrapolation turned off. (will be clamped)
     * @param t
     * @return
     */
    Point Traj::eval(float t)const  {
        int Npnt = points.size();
        vector<float> xs(Npnt);
        vector<float> ys(Npnt);
        vector<float> zs(Npnt);

        for(int n = 0 ; n < Npnt; n++){
            xs[n] = points[n].x;
            ys[n] = points[n].y;
            zs[n] = points[n].z;
        }

        float x = interpolate(ts,xs,t,false);
        float y = interpolate(ts,ys,t,false);
        float z = interpolate(ts,zs,t,false);

        return Point(x,y,z);
    }


    /**
     * Euclidean difference sum of all points. time evaluation points = this
     * @param traj
     * @return
     */
    float Traj::diff(const Traj &traj, float weightMinMaxRatio) {

        float diffSum = 0 ;
        int N = ts.size();
        float R = weightMinMaxRatio;
        for (int n = 0 ; n < N ; n++){
            float weight = 2.0f/(N*(R+1)) *( float(R-1)/(N-1) * N + 1  );
            diffSum += weight * points[n].distTo(traj.eval(ts[n]));
        }
        return diffSum;
    }


    float Traj::diff(const PolynomialXYZ &poly, float weightMinMaxRatio) {
        float diffSum = 0 ;
        int N = ts.size();
        for (int n = 0 ; n < ts.size() ; n++){
            diffSum +=  pow(weightMinMaxRatio,n) * points[n].distTo(poly.evalPnt(ts[n]));
        }
        return diffSum;
    }

    pcl::PointCloud<pcl::PointXYZ> PointSet::toPCL(string frame_id)    {
        pcl::PointCloud<pcl::PointXYZ> pcl;
        pcl::PointXYZ pntPCL;
        for(auto pnt:points){
            pntPCL.x = pnt.x;
            pntPCL.y = pnt.y;
            pntPCL.z = pnt.z;
            pcl.points.push_back(pntPCL) ;
        }
        pcl.header.frame_id = frame_id;
        return pcl;
    }


    Point PointSet::center() {
        int M = points.size();
        if (M == 0 ){
            cout << "PointSet: number of points is zero. center has nan value." << endl;
        }
        float xc = 0,yc = 0, zc = 0;

        for(int m = 0 ; m < M ; m++){
            xc+=points[m].x;
            yc+=points[m].y;
            zc+=points[m].z;
        }

        xc/= M; yc/=M ; zc /=M;
        return Point (xc,yc,zc);
    }



    void copyPntAt(sensor_msgs::PointCloud2& pcl, int n , float x, float y, float z){

        int arrayPosX = n*pcl.point_step + pcl.fields[0].offset;
        int arrayPosY = n*pcl.point_step + pcl.fields[1].offset;
        int arrayPosZ = n*pcl.point_step + pcl.fields[2].offset;

        memcpy(&pcl.data[arrayPosX],&x,sizeof(float));
        memcpy(&pcl.data[arrayPosY],&y,sizeof(float));
        memcpy(&pcl.data[arrayPosZ],&z,sizeof(float));

    }
    /**
     * Cartesian product implementation
     * @param lists
     * @return
     */
    vector<vector<int>> product(const std::vector<std::vector<int>>& lists) {
        std::vector<std::vector<int>> result;
        if (std::find_if(std::begin(lists), std::end(lists),
                         [](auto e) -> bool { return e.size() == 0; }) != std::end(lists)) {
            return result;
        }
        for (auto& e : lists[0]) {
            result.push_back({ e });
        }
        for (size_t i = 1; i < lists.size(); ++i) {
            std::vector<std::vector<int>> temp;
            for (auto& e : result) {
                for (auto f : lists[i]) {
                    auto e_tmp = e;
                    e_tmp.push_back(f);
                    temp.push_back(e_tmp);
                }
            }
            result = temp;
        }
        return result;
    }

    /**
     * closet distance between two line segment.
     * @param l
     * @return
     */
    float LineSegment::distTo(LineSegment l) const {
        float SMALL_NUM = 1e-4;
        Point u = p2 - p1;
        Point v = l.p2 - l.p1;
        Point w = p1 - l.p1;
        float    a = u.dot(u);         // always >= 0
        float    b = u.dot(v);
        float    c = v.dot(v);         // always >= 0
        float    d = u.dot(w);
        float    e = v.dot(w);
        float    D = a*c - b*b;        // always >= 0
        float    sc, sN, sD = D;       // sc = sN / sD, default sD = D >= 0
        float    tc, tN, tD = D;       // tc = tN / tD, default tD = D >= 0

        // compute the line parameters of the two closest points
        if (D < SMALL_NUM) { // the lines are almost parallel
            sN = 0.0;         // force using point P0 on segment S1
            sD = 1.0;         // to prevent possible division by 0.0 later
            tN = e;
            tD = c;
        }
        else {                 // get the closest points on the infinite lines
            sN = (b*e - c*d);
            tN = (a*e - b*d);
            if (sN < 0.0) {        // sc < 0 => the s=0 edge is visible
                sN = 0.0;
                tN = e;
                tD = c;
            }
            else if (sN > sD) {  // sc > 1  => the s=1 edge is visible
                sN = sD;
                tN = e + b;
                tD = c;
            }
        }

        if (tN < 0.0) {            // tc < 0 => the t=0 edge is visible
            tN = 0.0;
            // recompute sc for this edge
            if (-d < 0.0)
                sN = 0.0;
            else if (-d > a)
                sN = sD;
            else {
                sN = -d;
                sD = a;
            }
        }
        else if (tN > tD) {      // tc > 1  => the t=1 edge is visible
            tN = tD;
            // recompute sc for this edge
            if ((-d + b) < 0.0)
                sN = 0;
            else if ((-d + b) > a)
                sN = sD;
            else {
                sN = (-d +  b);
                sD = a;
            }
        }
        // finally do the division to get sc and tc
        sc = (abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
        tc = (abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

        // get the difference of the two closest points
        Point dP = w + (u*sc) - (v*tc);  // =  S1(sc) - S2(tc)

        float distance = (dP).norm();   // return the closest distance
        return distance;
    }

    Point operator*( float scalar,const Point& pnt){
        return  pnt.operator*(scalar);
    }

    /**
     * linearly sample uniform points spaced with ds
     * @param ds stride of sampling
     * @param includeEnds include p1,p2 for sampling
     * @return
     */
    PointSet LineSegment::samplePoints(float ds, bool includeEnds) const {
        Point dir = p2 - p1; dir.normalize();
        PointSet sampledPnts;

        Point p = p1 + ds * dir;
        float traverseLength = ds;

        if (includeEnds)
            sampledPnts.points.push_back(p1);

        while(traverseLength < this->length()){
            sampledPnts.points.push_back(p);
            p = p + dir * ds; // stride
            traverseLength += ds;
        }

        if (includeEnds)
            sampledPnts.points.push_back(p2);
        return sampledPnts;
    }

    /**
     * Sample points along line
     * @param nPoints 1 = p1 / 2 = (p1,p2) / ....
     * @return
     */
    PointSet LineSegment::samplePoints(unsigned int nPoints) const {
        float ds = (p2 - p1).norm() / (nPoints + 1);
        return samplePoints(ds,true);
    }





    void DirectedGraph::Report::init() {
        nFeasibleNodes.clear();
        nRejectEdgesTargetCollision.clear();
        nRejectEdgesDistanceAllowable.clear();
        nRejectEdgesTraverseObstacleCollision.clear();
        nEdgesFromPrevious.clear();

        elapseConstruction = -1;
        elapseSolve = -1;
    }


    bool DirectedGraph::edge_relax() {
        if (N_node){
            optimal_cost_to_nodes = Eigen::MatrixXf(2,N_node);
            // row 0 = optimal cost
            optimal_cost_to_nodes.block(0,0,1,N_node).setConstant(numeric_limits<float>::max());
            optimal_cost_to_nodes(0,0) = 0;
            // row 1 = optimal parent
            optimal_cost_to_nodes.block(1,0,1,N_node).setConstant(-1);
            optimal_cost_to_nodes(1,0) = -1;
            // relaxation
            for(int e_idx = 0 ; e_idx < N_edge ; e_idx++){
                int u = edges(1,e_idx), v = edges(2,e_idx); float w = edges(3,e_idx);
                if (optimal_cost_to_nodes(0,u) + w < optimal_cost_to_nodes(0,v)){
                    optimal_cost_to_nodes(0,v) = optimal_cost_to_nodes(0,u) + w;
                    optimal_cost_to_nodes(1,v) = u;
                }
            }

            return true;
        }
        else
            cout << "[Warning] Edge relaxation was performed before the graph created" << endl;
        return false;
    }

    bool DirectedGraph::solve() {
        optimal_node_idx_seq = Eigen::VectorXi(N+1); // optimal node sequence. This include 0th node also
        optimal_node_idx_seq.setConstant(-1);
        isSolved = false;

        // 1. edge_relax
        if (edge_relax()){
            // find the detector at every sequence from backtracing
            int n1 = node_div_location[N];
            int n2 = N_node-1;
            int min_idx_sub_node; // index of optimal final node in the final sub set of Node matrix
            int opt_goal_idx;

            // First, let's find optimal goal among the last time step
            Eigen::VectorXf cost_last_layer(n2-n1+1);
            for( int i = 0; i < cost_last_layer.size() ; i++)
                cost_last_layer(i) = optimal_cost_to_nodes(0,n1+i);

            cost_last_layer.minCoeff(&min_idx_sub_node);
            opt_goal_idx = n1 + min_idx_sub_node; optimal_node_idx_seq(N) = opt_goal_idx;
            int parent_idx, child_idx = opt_goal_idx;

            for ( int t = N-1 ; t >= 0 ; t--){
                parent_idx = optimal_cost_to_nodes(1,child_idx);
                optimal_node_idx_seq(t) = parent_idx;
                child_idx = parent_idx;
            }
            isSolved = true;
        }
        return isSolved;
    }

    void DirectedGraph::report() {

        // header
        int nEffectiveCol = 5;
        TOP_RULE_STAR(nEffectiveCol)
        TITLE(nEffectiveCol,"DAG statistics (n=0: current / n=1... planning)")

        if (N != log.nFeasibleNodes.size()-1)
            cout << "\n (E: no layer exists from time step " << N <<")";

        TOP_RULE_STAR(nEffectiveCol)
        FILL_TAG("[u->v]")
        FILL_CELL_RIGHT("[CV]/[FV]")
        FILL_CELL_RIGHT("[R-TC]")
        FILL_CELL_RIGHT(" > [R-DV]")
        FILL_CELL_RIGHT(" > [R-OC]")
        FILL_CELL_RIGHT("[E/UxV]")
        MID_RULE_DASH(nEffectiveCol)


        for (int n = 1 ; n <= N  ; n++){
            FILL_TAG(to_string(n-1) + "->" + to_string(n));
            FILL_CELL_RIGHT( to_string(log.nFeasibleAndConnectedNodes[n])+ "/" + to_string(log.nFeasibleNodes[n]))

            int uv =  log.nFeasibleAndConnectedNodes[n - 1] * log.nFeasibleNodes[n];
            int e = log.nEdgesFromPrevious[n-1];

            FILL_CELL_RIGHT(-log.nRejectEdgesTargetCollision[n - 1]);
            FILL_CELL_RIGHT(-log.nRejectEdgesDistanceAllowable[n - 1]);
            FILL_CELL_RIGHT(-log.nRejectEdgesTraverseObstacleCollision[n - 1]);
            FILL_CELL_RIGHT(to_string(e) + "/" + to_string(uv));
            if (n != N)
                NEW_LINE
        }
        MID_RULE_DASH(5)
        cout << "CV: connected vertex /FV: feasible vertex" << endl;
        cout << "TC: target colli. /DV: distance violation /OC: obst. colli.";
        MID_RULE_DASH(5)
    }

    /**
     * draw the registered node layers along time step
     * @param frame_id
     * @return
     * @details intensity coloring along time
     */
    pcl::PointCloud<pcl::PointXYZI> DirectedGraph::getPCLPath(string frame_id) {

        pcl::PointCloud<pcl::PointXYZI> pclPath;
        pclPath.header.frame_id = frame_id;

        for (int n = 0 ; n <= N ; n++){
            int startIdx = node_div_location[n];
            int endIdx;

            if (n < N)
                endIdx = node_div_location[n+1];
            else
                endIdx = N_node;
            for (int i  =  startIdx; i < endIdx ; i++ ){
                pcl::PointXYZI pnt;

                pnt.x = nodes(2,i);
                pnt.y = nodes(3,i);
                pnt.z = nodes(4,i);
                pnt.intensity = n;

                pclPath.points.push_back(pnt);
            }
        }
        return pclPath;
    }


    /**
     *
     * @param data
     * @param edges binning
     * @param var
     * @return histogram (not probability)
     * @todo boolean property meta data
     */
    vector<int> hist(const vector<float>& data,Eigen::VectorXf edges,float * var ){

        vector<float> edge_vec(edges.data(),edges.data()+edges.size());
        float dx = edge_vec[2] - edge_vec[1]; // uniform interval assumed
        Eigen::VectorXf bin_centers(edges.size() - 1);
        bin_centers = edges.block(0,0,edges.size() - 1,1); bin_centers.array() += dx/2;

        auto it_begin = data.begin();
        auto it_end = data.end();
        Eigen::VectorXf hist_prob(edges.size()-1); hist_prob.setZero();

        for (auto it_data = data.begin() ; it_data < data.end() ; it_data++){
            auto it_lower = lower_bound(edge_vec.begin(),edge_vec.end(),*it_data);
            int pos = it_lower - edge_vec.begin();
            if (pos == 0)
                printf("[Histogram warning]: data %f out of lower bound of edges(%f). not counted. \n",*it_data,edges(0));
            else if (pos == edges.size())
                printf("[Histogram warning]: data %f out of upper bound of edges(%f). not counted. \n",*it_data,edges(edges.size()-1));
            else
                hist_prob(pos-1) += 1;
        }

//        hist_prob /= hist_prob.sum(); // I want NUMBER
        if (var != NULL)
            *var = (hist_prob.array()*bin_centers.array()*bin_centers.array()).sum() - pow((hist_prob.array()*bin_centers.array()).sum(),2);

        return vector<int>(hist_prob.data(),hist_prob.data() + hist_prob.size());
    }

    std::string to_string_with_precision(const float a_value, const int n ){
        std::ostringstream out;
        out.precision(n);
        out << std::fixed << a_value;
        return out.str();
    }


    double polyEval(const double* polyCoeff,int order,double x){
        double result = polyCoeff[order];
        for (int i = order - 1 ; i >=0 ; i--)
            result = (x*result + polyCoeff[i]);
        return result;
    }


    double B(uint n, uint d) {
        if (d==0)
            return 1;
        else
        if (n<d)
            return 0;
        else{
            Eigen::VectorXi prodTarget = Eigen::VectorXi::LinSpaced(d,n-(d-1),n);
            return accumulate(prodTarget.data(),prodTarget.data()+prodTarget.size(),
                              1,std::multiplies<int>());
        }
    }


    Polynomial::Polynomial(const VectorXd &coeffVec)  {
        order = coeffVec.size()-1;
        while (abs(coeffVec(order)) < 1e-10 and order > 0) {
//        printf("[Polynomial consturction] the leading coeffi = 0\n");
            order--;
        }
        polyCoeff = new double[order+1];
        for (auto i = 0 ; i <= order ; i++ ) {
            polyCoeff[i] = coeffVec.coeffRef(i);
        }
    }

    Polynomial::Polynomial(double *polyCoeffIn,int polyOrderIn,bool isShallow) {
        while (abs(polyCoeffIn[polyOrderIn]) < 1e-10 and polyOrderIn > 0) {
//        printf("[Polynomial consturction] the leading coeffi = 0\n");
            polyOrderIn--;
        }
        isShallowCopied = isShallow;
        order = polyOrderIn;

        unsigned int size = polyOrderIn+1;
        if (not isShallow){
            polyCoeff = new double[size];
            for (auto i = 0 ; i <= order ; i++ )
                polyCoeff[i] = polyCoeffIn[i];
        }
        else
            polyCoeff = polyCoeffIn;
    }

    Polynomial::Polynomial(const Polynomial &rhs) {

        size_t order_ = rhs.getOrder();
        while (abs(rhs.getData()[order_]) < 1e-10 and order_ > 0) {
//        printf("[Polynomial consturction] the leading coeffi = 0\n");
            order_--;
        }
        polyCoeff = new double[order_+1];
        order = order_;
        for (auto i = 0 ; i <= order ; i++ )
            polyCoeff[i] = rhs.getData()[i];
    }

    Polynomial::Polynomial(const size_t &order_):order(order_) {
        polyCoeff = new double[order_+1];
        for (uint i = 0 ; i <= order ; i++) // set zero
            polyCoeff[i] = 0 ;
    }

    void Polynomial::setData(double *newPolyCoeff,size_t order_) {
        order = order_;
        if (polyCoeff != NULL)
            delete[] polyCoeff;
        polyCoeff = newPolyCoeff;
        isShallowCopied  = true;
    }

    void Polynomial::copyData(double *newPolyCoeff, size_t order_) {
        for (uint i = 0 ; i <= order_ ; i++)
            polyCoeff[i] = newPolyCoeff[i];
    }

    void Polynomial::setZero() {
        for (uint i = 0 ; i <= order ; i++)
            polyCoeff[i] = 0;
    }

    Polynomial& Polynomial::operator=(const Polynomial& rhs){

        if (this != &rhs){
            if (polyCoeff == nullptr){
                polyCoeff = new double[rhs.getOrder()+1];
            }else{
                if (order < rhs.getOrder()){ // should increase the size.dont know how to do that in pure pointer
                    delete[] polyCoeff;
                    polyCoeff = new double[rhs.getOrder()+1];
                }
            }
            order = rhs.getOrder();
            for (auto i = 0 ; i <= order ; i++ ) {
                polyCoeff[i] = rhs.getData()[i];
            }
        }
        return *this;
    }

    double& Polynomial::coeffRef(const unsigned int index) {
        assert(index <= order && "Querying index > polynomial order");
        return this->polyCoeff[index];
    };

    double Polynomial::coeff(unsigned int index) const{
        assert(index <= order && "Querying index > polynomial order");
        return this->polyCoeff[index];
    };


    Polynomial Polynomial::operator*(const double &rhs) const {
        return Polynomial(rhs*getVector());
    }


    Polynomial operator*(double scalar, const Polynomial &rhs) {
        return rhs*scalar;
    }


    Polynomial Polynomial::operator+(const Polynomial &rhs) const{

        if (order > rhs.getOrder()){
            Polynomial pOut(*this);
            for (auto i = 0 ; i<= rhs.getOrder(); i++)
                pOut.coeffRef(i) += rhs.coeff(i);
            return pOut;

        }else{
            Polynomial pOut(rhs);
            for (auto i = 0 ; i<= getOrder(); i++)
                pOut.coeffRef(i) += coeff(i);
            return pOut;
        }
    }

    Polynomial Polynomial::operator-(const Polynomial &rhs) const {
        return Polynomial(*this + rhs * double(-1.0));
    }

    Polynomial Polynomial::operator*(const Polynomial& rhs) const{
        unsigned int n1 = order;
        unsigned int n2 = rhs.getOrder();
        auto dataPtr = new double[n1+n2+1];
        for (unsigned int k = 0; k <= n1+n2 ; k++) {
            dataPtr[k] = 0;
            for (auto j = max(0, int(k - n2)); j <= min(n1, k); j++)
                dataPtr[k] += polyCoeff[j] * rhs.getData()[k - j];
        }
        Polynomial pOut(dataPtr,n1+n2);
        return pOut;
    }

    Polynomial& Polynomial::operator *= (double c){
        for(uint i = 0 ; i <= order ; i++)
            polyCoeff[i]*=c;
        return *this;
    }

    Polynomial& Polynomial::operator+=(const Polynomial& rhs){
        assert(order >= rhs.getOrder() && "Do not support this case.");
        for (uint i = 0 ;  i<= rhs.getOrder() ; i++)
            polyCoeff[i] += rhs.getData()[i];
        return *this;
    }

    Polynomial& Polynomial::operator-=(const Polynomial& rhs){
        assert(order >= rhs.getOrder() && "Do not support this case.");
        for (uint i = 0 ;  i<= rhs.getOrder() ; i++)
            polyCoeff[i] -= rhs.getData()[i];
        return *this;
    }

    Polynomial& Polynomial::operator+=(double c) {
        polyCoeff[0] += c;
        return *this;
    }


    Polynomial& Polynomial::operator-=(double c) {
        polyCoeff[0] -= c;
        return *this;
    }


    PolynomialDivOutput Polynomial::div(const Polynomial &rhs) const {
        uint m = getOrder();
        uint n = rhs.getOrder();
        assert(m>=n && "deg(N) >= deg(D) failed");

        Polynomial q(m-n);

        if (n == 0){
            q = (*this)*(1/rhs.coeff(0));
            Polynomial r(0); r.coeffRef(0) = 0;
            return PolynomialDivOutput(q,r);
        }
        else{
            Polynomial r(n-1);
            double u[m+1];
            for (auto i = 0; i <= m ; i++)
                u[i] = coeff(i);

            for (int k = m-n ; k >=0 ; k--){
                q.coeffRef(k) = u[n+k]/rhs.coeff(n);
                for (int j = n+k-1 ; j>=k; j--)
                    u[j] -= q.coeff(k)*rhs.coeff(j-k);
            }
            for (uint k = 0 ; k <= n-1 ;k++)
                r.coeffRef(k) = u[k];
            return PolynomialDivOutput(q,r);
        }
    }

    /**
     * @brief Number of real root in [start,end]. Inaccurate when multiple order drop of sturm sequence
     * @param start
     * @param end
     * @return
     */
    size_t Polynomial::nRoot2(double start, double end) const {
        uint n = getOrder();

        // Check the non zero of leading coeffi
        while (abs(polyCoeff[n]) < 1e-8 ) {
//        printf("[Polynomial consturction] the leading coeffi = 0\n");
            n--;
        }

        if ( n == 0){ // just constant polynomial
            if (abs(polyCoeff[0]) < 1e-8){ // zero constant
                cerr << "zero constant polynomial. Number of zeros is inf" << endl;
                return 100;
            }
            else // non zero constant
                return 0;
        }
        else if (n == 1){
            double root = -polyCoeff[0]/polyCoeff[1];
            if (root >= start and root <= end)
                return 1;
            else
                return 0;
        }else{
            // do nothing
        }

        double f0[n+1],f1[n],f2[n-1]; // maximum size

        // Read
        for (int i = 0 ; i<=n-1 ; i++) {
            f0[i] = coeff(i);
            f1[i] = coeff(i+1)*(i+1);
        }
        f0[n] = coeff(n);

        uint sDiff1 = ((polyEval(f0,n,start)*polyEval(f1,n-1,start))<0) ;
        uint sDiff2 = ((polyEval(f0,n,end)*polyEval(f1,n-1,end))<0);

        // Calculating Sturm sequence
        double q1,q0,q2;
        for (int i = 2 ; i<=n;i++){
            // deg of each fun
            uint n0 = n-i+2,n1 = n0-1,n2 = n0-2;

            // quotient
            q1 = f0[n0]/f1[n1],q0 = 1/f1[n1]*(f0[n0-1]-f1[n1-1]*f0[n0]/f1[n1]);
            q2 = f0[n0-2]-f1[n1-1]*q0;
            if (abs(q2) <  1e-10){
//            cout << "Multiple decrease of order is not handled. "<<endl;
                return 1;
            }


            // 1. calculate f2 and sign change
            f2[0] = f1[0]*q0-f0[0];
            for (int j = 1 ; j<= n2 ; j++)
                f2[j] = f1[j-1]*q1+f1[j]*q0-f0[j];

            sDiff1 += (polyEval(f2,n2,start)*polyEval(f1,n1,start)<0);
            sDiff2 += (polyEval(f2,n2,end)*polyEval(f1,n1,end)<0);

            // 2. update
            for (int j = 0; j<=n2;j++) {
                f0[j] = f1[j];
                f1[j] = f2[j];
            }
            f0[n1] = f1[n1];
        }
        return abs(int(sDiff1 - sDiff2));
    }



    Polynomial Polynomial::derviative() const {
        if(getOrder()>=0){
            Polynomial der(getOrder()-1);
            for (int i = 0 ; i <= getOrder()-1 ; i++)
                der.coeffRef(i) = coeff(i+1)*(i+1);
            return der;
        }else{
            Polynomial der(0);
            der.coeffRef(0) = 0;
            return der;
        }
    }

    double Polynomial::eval(double x) const  {
        double result = polyCoeff[order];
        for (int i = order - 1 ; i >=0 ; i--)
            result = (x*result + polyCoeff[i]);
        return result;
    }

    double Polynomial::eval(double x,uint dOrder) const {
        // Constructing tVec
        VectorXd vec(order+1); vec.setZero();
        for (int n = dOrder ; n < order +1 ; n ++)
            vec(n) = B(n,dOrder)*pow(x,n-dOrder);

        return vec.transpose()*getVector();
    }

    double Polynomial::integration(double x1, double x2) const {
        double result = 0 ;
        for (int i=0 ; i <= order ; i ++){
            result+= polyCoeff[i]/(i+1) * (pow(x2,i+1) - pow(x1,i+1));
        }
        return result;
    }

    Polynomial::~Polynomial(){
        // We assume that the origin of the pointer shallow copied will be deleted in the origin class
        if ((not isShallowCopied) and polyCoeff != NULL) {

            delete[] polyCoeff;
        }
    }

    Polynomial PolynomialXYZ::operator [] (int i) const {
        assert( i == 0 or i ==1 or i==2&& "[PolynomialXYZ] invalid indexing with bracket" );
        if ( i == 0)
            return px;
        else if (i == 1 )
            return py;
        else
            return pz;
    }
    Polynomial& PolynomialXYZ::coeffRef (int i) {
        assert( i == 0 or i ==1 or i==2&& "[PolynomialXYZ] invalid indexing with bracket" );
        if ( i == 0)
            return px;
        else if (i == 1 )
            return py;
        else
            return pz;
    }

    double PolynomialXYZ::coeff (int i,int j ) const {
        assert( i == 0 or i ==1 or i==2&& "[PolynomialXYZ] invalid indexing with bracket" );
        if ( i == 0)
            return px.coeff(j);
        else if (i == 1 )
            return py.coeff(j);
        else
            return pz.coeff(j);
    }

    uint PolynomialXYZ::getOrder(int i ) const {
        assert( i == 0 or i ==1 or i==2&& "[PolynomialXYZ] invalid indexing with bracket" );
        if ( i == 0)
            return px.getOrder();
        else if (i == 1 )
            return py.getOrder();
        else
            return pz.getOrder();

    }

    void PolynomialXYZ::setX(const Polynomial& px_){
        px = Polynomial(px_.getData(),px_.getOrder());
    };
    void PolynomialXYZ::setY(const Polynomial& py_){
        py = Polynomial(py_.getData(),py_.getOrder());
    };
    void PolynomialXYZ::setZ(const Polynomial& pz_){
        pz = Polynomial(pz_.getData(),pz_.getOrder());
    };


    PolynomialXYZ::PolynomialXYZ(Polynomial* pxPtr,Polynomial* pyPtr,Polynomial* pzPtr){
        px = Polynomial(pxPtr->getData(),pxPtr->getOrder(),true);
        py = Polynomial(pyPtr->getData(),pyPtr->getOrder(),true);
        pz = Polynomial(pzPtr->getData(),pzPtr->getOrder(),true);
    }

    Vector3f PolynomialXYZ::evalVec(double t) const {
        return Vector3f(px.eval(t),py.eval(t),pz.eval(t));
    }

    Vector3f PolynomialXYZ::evalVec(double t,uint dOrder) const {
        return Vector3f(px.eval(t,dOrder),py.eval(t,dOrder),pz.eval(t,dOrder));
    }

    Point PolynomialXYZ::evalPnt (double t) const {
        return Point (evalVec(t));
    }

    Point PolynomialXYZ::evalPnt (double t,uint dOrder) const {
        return Point (evalVec(t,dOrder));
    }

    PointSet PolynomialXYZ::evalPntSet(VectorXd ts) const {
        PointSet pntSet;
        for (int n = 0; n < ts.size() ; n++)
            pntSet.push_back(evalPnt(ts(n)));
        return pntSet;
    }

    void PolynomialXYZ::print() const{
        cout << "px/py/pz " << endl;
        cout << px.getVector().transpose() << endl;
        cout << py.getVector().transpose() << endl;
        cout << pz.getVector().transpose() << endl;
    }



    /**
     *
     * @param t0
     * @param tf
     * @param dt
     * @return
     */
    Traj PolynomialXYZ::toTraj(float t0, float tf, float dt) const {
        float t = t0;
        vector<Point> pnts;
        vector<float> ts;
        int N = int((tf-t0)/dt);
        return this->toTraj(t0,tf,N+1);
    }



    /**
     *
     * @param t0
     * @param tf
     * @param N number of points included in the traj
     * @return
     */
    Traj PolynomialXYZ::toTraj(float t0, float tf, int N ) const {
        VectorXd tVec (N); tVec.setLinSpaced(N,t0,tf);
        vector<Point> pnts;
        vector<float> ts(tVec.data() , tVec.data()+N);
        for (int n = 0; n < N ; n++){
            pnts.push_back( evalPnt(ts[n]));
        }
        return Traj(pnts,ts);
    }



    /**
     * Initialize parameter to develop a gridfield
     * @param nTraj number of instance to be inspected per grid field
     * @param resolution grid resolution
     * @param targets points to be included in the field with the margin
     * @param margin_xy
     * @param margin_z
     * @param minZ clamping
     * @param maxZ
     * @param isValidField report whether this initialization is valid
     */
    BooleanGridParam::BooleanGridParam(int nTraj, double resolution_, const PointSet &targets, double margin_xy, double margin_z,
                                       double minZ, double maxZ, bool &isValidField): nTraj(nTraj) {

        assert(targets.points.size() != 0);

        float xmin = numeric_limits<float>::max();
        float ymin = numeric_limits<float>::max();
        float zmin = numeric_limits<float>::max();

        float xmax = -numeric_limits<float>::max();
        float ymax = -numeric_limits<float>::max();
        float zmax = -numeric_limits<float>::max();
        for(auto pnt :targets.points){
            xmin = min(pnt.x,xmin);
            ymin = min(pnt.y,ymin);
            zmin = min(pnt.z,zmin);
            xmax = max(pnt.x,xmax);
            ymax = max(pnt.y,ymax);
            zmax = max(pnt.z,zmax);
        }

        origin.x = xmin - margin_xy;
        origin.y = ymin - margin_xy;
        origin.z = max(zmin - margin_z,minZ);
        lx = xmax - xmin + 2*margin_xy;
        ly = ymax - ymin + 2*margin_xy;
        lz = min(zmax + margin_z - origin.z,maxZ -origin.z );

        if (lx <= 0 or ly <= 0 or lz <= 0 ){
            isValidField = false;
        }else
            isValidField = true;

        resolution = resolution_;
        min_z = minZ;
    }


    TraverseGrid::TraverseGrid(BooleanGridParam param): param(param) {

        isFieldInit = true;
        Nx = floor(param.lx/param.resolution);
        Ny = floor(param.ly/param.resolution);
        Nz = floor(param.lz/param.resolution);
        Nt = param.nTraj;

        data = new bool***[Nx];
        for (int x=0;x<Nx;x++){
            data[x] = new bool**[Ny];
            for (int y=0;y<Ny;y++) {
                data[x][y] = new bool *[Nz];
                for (int z = 0 ; z < Nz ; z++) {
                    data[x][y][z] = new bool[Nt];
                    for (int n = 0 ; n < Nt ; n++)
                        data[x][y][z][n] = false; //! init nth traj with non-traversed
                }
            }
        }
    }

    /**
     * Conversion between linear index and subscript index (x,y,z,w)
     * @param linIdx
     * @return
     */
    ind4 TraverseGrid::ind2sub(int linIdx) const {
        const int ndims = 4;
        int siz[4]  = {Nx,Ny,Nz,Nt};
        ind4 sub;
        std::vector<int> cumProd(ndims);
        int nextProd = 1;
        for(int k=0; k<ndims; k++)
        {
            cumProd[k] = nextProd;
            nextProd *= siz[k];
        }

        int maxInd = cumProd[ndims-1] * siz[ndims-1] - 1;
        if( (linIdx<0) || (linIdx > maxInd) )
        {
            cout << "[TraverseGrid] ind2sub out of range" << endl;
            return sub;
        }

        int remainder = linIdx;
        for(int k=ndims-1; k>=0; k--)
        {
            sub(k) = remainder / cumProd[k];
            remainder = remainder % cumProd[k];
        }
        return sub;
    }

    int TraverseGrid::sub2ind(ind4 subIdx) const {
        return subIdx.x + subIdx.y * Nx + subIdx.z * Nx*Ny + subIdx.w * Nx*Ny*Nz;
    }


    ind3 TraverseGrid::pnt2ind(Point pnt) const {
        return ind3(min(int(floor((pnt.x-param.origin.x)/param.resolution)),this->Nx-1),
                    min(int(floor((pnt.y-param.origin.y)/param.resolution)),this->Ny-1) ,
                    min(int(floor((pnt.z-param.origin.z)/param.resolution)),this->Nz-1));
    }

    Point TraverseGrid::ind2pnt(ind3 ind ) const {


        assert(isInRange(ind.x,0) and isInRange(ind.y,1) and isInRange(ind.z,2) and "ind2pnt index out of range");
        Point pnt;
        pnt.x = param.origin.x + param.resolution * (ind.x+0.5);
        pnt.y = param.origin.y + param.resolution * (ind.y+0.5);
        pnt.z = param.origin.z + param.resolution * (ind.z+0.5);
        return pnt;
    }

    bool TraverseGrid::isInRange(int ind_, int dim) const  {
        switch (dim) {
            case 0:
                return (ind_ >= 0 and ind_ < Nx);
            case 1:
                return (ind_ >= 0 and ind_ < Ny);
            case 2:
                return (ind_ >= 0 and ind_ < Nz);
            default:
                return -1;
        }
    }

    bool TraverseGrid::isInRange(ind3 ind ) const {
        return (ind.x >= 0 and ind.x < Nx and ind.y >= 0 and ind.y < Ny and ind.z >= 0 and ind.z < Nz );
    }

    bool TraverseGrid::isInRange(Point pnt) const {
        return pnt.x >= param.origin.x  and pnt.x <= param.origin.x + param.lx and
               pnt.y >= param.origin.y and pnt.y <= param.origin.y + param.ly and
               pnt.z >=  param.origin.z and pnt.z <= param.origin.z + param.lz;

    }


    /**
     * mark the (pnt,trajIdx) is associated (: the pnt is traversed by trajIdx trajectory)
     * @param pnt
     * @param trajIdx
     * @bug we assume the point boundedness check aleardy performed
     */
    void TraverseGrid::setTraverse(const Point &pnt, int trajIdx, bool isTraverse) {
        ind3 locIdx = pnt2ind(pnt);
        if ((! data[locIdx.x][locIdx.y][locIdx.z][trajIdx]) and isTraverse)
            nTrue++;
        data[locIdx.x][locIdx.y][locIdx.z][trajIdx] = isTraverse;
    }


    /**
     * identify the traversed cells w.r.t pointSet and save it to trajIdx th library
     * @param pointSet
     * @param trajIdx
     * @param isAllPointsInGrid if false, cells out of grid can affect the feasibility of the pair
     * @return number of pnts on traverse in the grid
     */
    int TraverseGrid::markTraversed(const PointSet &pointSet, int trajIdx, bool & isAllPointsInGrid) {
        isAllPointsInGrid = true;
        int nTraverseCell = 0;
        for (const auto& pnt : pointSet.points){
            if (isInRange(pnt)){
                setTraverse(pnt, trajIdx); //! this cell is traversed w.r.t trajIdx
                nTraverseCell++;
            }else{
                isAllPointsInGrid = false;
            }
        }

        return nTraverseCell;
    }

    /**
     * visualize the cells with intensity encoding the traverse of a trajectory
     * @param trajIdx index of trajectory in library
     * @param worldFrameId
     * @param onlyTraverse
     * @return
     */
    pcl::PointCloud<pcl::PointXYZI> TraverseGrid::visualizeAt(int trajIdx, string worldFrameId,bool onlyTraverse) const {
        pcl::PointCloud<pcl::PointXYZI> pntSet;
        int nTraverseCell = 0;
        for(int x = 0 ; x< Nx ; x++)
            for(int y = 0 ; y< Ny ; y++)
                for(int z = 0 ; z< Nz ; z++){
                    Point pnt = ind2pnt(ind3(x,y,z));
                    pcl::PointXYZI pclPnt;
                    pclPnt.x = pnt.x;
                    pclPnt.y = pnt.y;
                    pclPnt.z = pnt.z;
                    pclPnt.intensity = (float) data[x][y][z][trajIdx];
                    if (data[x][y][z][trajIdx])
                        nTraverseCell++;
                    if (onlyTraverse) {
                        if (data[x][y][z][trajIdx])
                            pntSet.points.push_back(pclPnt);
                    }else
                        pntSet.points.push_back(pclPnt);
                }
        printStr("# of traverse cells by " +  to_string(trajIdx)  + " th trajectory: " + to_string(nTraverseCell));
        pntSet.header.frame_id = worldFrameId;
        return pntSet;
    }

    /**
     * Point (cell) has non-zero intensity if any of trajectories pass it
     * @param frameId
     * @return
     */
    pcl::PointCloud<pcl::PointXYZI> TraverseGrid::visualizeAll(string frameId) const {
        pcl::PointCloud<pcl::PointXYZI> pntSet;
        pntSet.header.frame_id = frameId;

        for (auto elem: gridTrajPairSet){
            Point pnt  = ind2pnt(elem.first);
            pcl::PointXYZI pclPnt;
            pclPnt.x = pnt.x;
            pclPnt.y = pnt.y;
            pclPnt.z = pnt.z;
            pclPnt.intensity = elem.second.size();
            pntSet.points.push_back(pclPnt);
        }

        return pntSet;
    }

    visualization_msgs::Marker TraverseGrid::visualizeAllMarker(string frameId,std_msgs::ColorRGBA colorAssociation,
                                                                                std_msgs::ColorRGBA colorNonAssociation,
                                                                                string ns) const {
        visualization_msgs::Marker marker;
        marker.ns = ns ;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = param.resolution;
        marker.scale.y = param.resolution;
        marker.scale.z = param.resolution;
        marker.header.frame_id = frameId;

        for(int x = 0 ; x< Nx ; x++)
            for(int y = 0 ; y< Ny ; y++)
                for(int z = 0 ; z< Nz ; z++){
                    Point pnt = ind2pnt(ind3(x,y,z));
                    // todo make function?
                    bool isThisTraverseByAny = false;
                    for(int i = 0 ; i <param.nTraj ; i++)
                        if (data[x][y][z][i]){
                            isThisTraverseByAny = true;
                            break;
                        }
                    marker.points.push_back(pnt.toGeometry());
                    if (isThisTraverseByAny)
                        marker.colors.push_back(colorAssociation);
                    else
                        marker.colors.push_back(colorNonAssociation);
                }
       return marker;
    }

    /**
     * identify list of trajectories traversed by each cell
     * This function should be called after full initialization of data
     */
    void TraverseGrid::computeGridTrajPairSet() {
        gridTrajPairSet.clear();
        for(int x = 0 ; x< Nx ; x++)
            for(int y = 0 ; y< Ny ; y++)
                for(int z = 0 ; z< Nz ; z++) {
                    ind3 cell(x,y,z) ;
                    vector<int> associatedTrajIndexSet;
                    for (int n = 0; n < Nt ; n++)
                       if (get(cell,n))
                           associatedTrajIndexSet.push_back(n);

                   if (not associatedTrajIndexSet.empty())
                       gridTrajPairSet.emplace_back(make_pair(cell, associatedTrajIndexSet));
                }
    }

    /**
     * identify list of cells traversed by each traj
     * This function should be called after full initialization of data
     */
    void TraverseGrid::computeTrajGridPairSet() {
        trajGridPairSet.clear();
        for (int n = 0; n < Nt ; n++) {
            vector<ind3> associatedCellIndexSet;
            for (int x = 0; x < Nx; x++)
                for (int y = 0; y < Ny; y++)
                    for (int z = 0; z < Nz; z++) {
                        ind3 cell(x, y, z);
                        if (get(cell, n))
                            associatedCellIndexSet.push_back(cell);
                    }
                trajGridPairSet.emplace_back(make_pair(n,associatedCellIndexSet));
        }
    }

    void swap(int *a, int *b ){
        int tmp;
        tmp = *a;
        *a = *b;
        *b = tmp;
    }

    void permutation(int * arr, int n, int r, int depth){
        if( r == depth){
//        print_arr(arr,depth);
            return;
        }

        for(int i=depth; i<n; i++){
            swap(&arr[i], &arr[depth]);
            permutation(arr,n, r, depth+1);
            swap(&arr[i], &arr[depth]);
        }
    }

    Permutator::Permutator(int s, int v): cSlots(s), cValues(v){
        a = new int[s];
        for (int i = 0; i < cSlots - 1; i++) {
            a[i] = 1;
        }
        a[cSlots - 1] = 0;
        nextInd = cSlots;
        outputArr = new int[int(s*pow(v,s))];
    }

    bool Permutator::doNext(){
        for (;;){
            if (a[nextInd - 1] == cValues) {
                nextInd--;
                if (nextInd == 0)
                    return false;
            }
            else {
                a[nextInd - 1]++;
                while (nextInd < cSlots) {
                    nextInd++;
                    a[nextInd - 1] = 1;
                }
                return true;
            }
        }
    }

    int* Permutator::getOutputArr(){
        int slotN = 0;
        while (doNext()) {
            for (int i = 0; i < cSlots; i++)
                outputArr[slotN*cSlots+i] =  a[i]-1;
            slotN ++;
        }
        return outputArr;
    }



    double PolynomialHelper::B(uint n, uint d) {
        if (d==0)
            return 1;
        else
        if (n<d)
            return 0;
        else{
            VectorXi prodTarget = VectorXi::LinSpaced(d,n-(d-1),n);
            return accumulate(prodTarget.data(),prodTarget.data()+prodTarget.size(),
                              1,std::multiplies<int>());
        }
    }

    MatrixXd PolynomialHelper::scaleMat(double dt){

        uint N = order;
        MatrixXd mat(N+1,N+1);
        mat.setZero();
        for (int i = 0 ; i < N+1 ; i++)
            mat(i,i) = pow(dt,i);
        return mat;
    }

    VectorXd PolynomialHelper::tVec(double t,uint d){

        uint N = order;
        VectorXd vec(N+1); vec.setZero();
        for (int n = d ; n < N+1 ; n ++)
            vec(n) = B(n,d)*pow(t,n-d);
        return vec;

    }

    MatrixXd PolynomialHelper::intDerSquared(uint d, double T){

        uint N = order;
        MatrixXd Q(N+1,N+1);
        Q.setZero();
        if (d > N) {
            printf("[Type 1] Order of derivative exceeds the polynomial order. Returning zero mat.\n");
        }else {
            for (uint i = d; i < N + 1; i++)
                for (uint j = d; j < N + 1; j++)
                    Q.coeffRef(i, j) = B(i, d) * B(j, d) / (i + j - 2 * d + 1);
        }
        return scaleMat(T)*Q*scaleMat(T)/pow(T,2*d-1);

    }

    MatrixXd PolynomialHelper::intDerSquared(uint d , double T, int dOrder){

        uint N = dOrder;
        MatrixXd Q(N+1,N+1);
        Q.setZero();

        if (d > N) {
//        printf("[Type 2] Order of derivative exceeds the polynomial order. Returning zero mat.\n");
        }else {
            for (uint i = d; i < N + 1; i++)
                for (uint j = d; j < N + 1; j++)
                    Q.coeffRef(i, j) = B(i, d) * B(j, d) / (i + j - 2 * d + 1);
        }
        return scaleMat(T).block(0,0,order+1,order+1)*Q*scaleMat(T).block(0,0,order+1,order+1)/pow(T,2*d-1);
    }
}



progressbar::progressbar() :
        progress(0),
        n_cycles(0),
        last_perc(0),
        do_show_bar(true),
        update_is_called(false),
        done_char("#"),
        todo_char(" "),
        opening_bracket_char("["),
        closing_bracket_char("]") {}

progressbar::progressbar(int n, bool showbar) :
        progress(0),
        n_cycles(n),
        last_perc(0),
        do_show_bar(showbar),
        update_is_called(false),
        done_char("#"),
        todo_char(" "),
        opening_bracket_char("["),
        closing_bracket_char("]") {}

void progressbar::reset() {
    progress = 0,
            update_is_called = false;
    last_perc = 0;
    return;
}

void progressbar::set_niter(int niter) {
    if (niter <= 0) throw std::invalid_argument(
                "progressbar::set_niter: number of iterations null or negative");
    n_cycles = niter;
    return;
}

void progressbar::update() {

    if (n_cycles == 0) throw std::runtime_error(
                "progressbar::update: number of cycles not set");

    if (!update_is_called) {
        if (do_show_bar == true) {
            std::cout << opening_bracket_char;
            for (int _ = 0; _ < 50; _++) std::cout << todo_char;
            std::cout << closing_bracket_char << " 0%";
        }
        else std::cout << "0%";
    }
    update_is_called = true;

    int perc = 0;

    // compute percentage, if did not change, do nothing and return
    perc = progress*100./(n_cycles-1);
    if (perc < last_perc) return;

    // update percentage each unit
    if (perc == last_perc + 1) {
        // erase the correct  number of characters
        if      (perc <= 10)                std::cout << "\b\b"   << perc << '%';
        else if (perc  > 10 and perc < 100) std::cout << "\b\b\b" << perc << '%';
        else if (perc == 100)               std::cout << "\b\b\b" << perc << '%';
    }
    if (do_show_bar == true) {
        // update bar every ten units
        if (perc % 2 == 0) {
            // erase closing bracket
            std::cout << std::string(closing_bracket_char.size(), '\b');
            // erase trailing percentage characters
            if      (perc  < 10)               std::cout << "\b\b\b";
            else if (perc >= 10 && perc < 100) std::cout << "\b\b\b\b";
            else if (perc == 100)              std::cout << "\b\b\b\b\b";

            // erase 'todo_char'
            for (int j = 0; j < 50-(perc-1)/2; ++j) {
                std::cout << std::string(todo_char.size(), '\b');
            }

            // add one additional 'done_char'
            if (perc == 0) std::cout << todo_char;
            else           std::cout << done_char;

            // refill with 'todo_char'
            for (int j = 0; j < 50-(perc-1)/2-1; ++j) std::cout << todo_char;

            // readd trailing percentage characters
            std::cout << closing_bracket_char << ' ' << perc << '%';
        }
    }
    last_perc = perc;
    ++progress;
    std::cout << std::flush;

    return;
}


nav_msgs::Odometry chasing_utils::poseToOdom(const geometry_msgs::PoseStamped& poseStamped){
    nav_msgs::Odometry odom;
    odom.header = poseStamped.header;
    odom.pose.pose = poseStamped.pose;
    return odom;
}

visualization_msgs::Marker chasing_utils::getClearingMarker(string worldFrame, string ns, int id){
    visualization_msgs::Marker clearingMarker;
    clearingMarker.ns = ns;
    clearingMarker.id = id;
    clearingMarker.header.frame_id = worldFrame;
    clearingMarker.action = visualization_msgs::Marker::DELETE;
    return clearingMarker;
}


/**
 * @brief get the marker array from t0 to tf
 * @param t0
 * @param tf
 * @param frame_id
 * @return
 */
visualization_msgs::MarkerArray chasing_utils::EllipsoidObstacle::getMarkerTrace(double t0, double tf, string frame_id,int& nsId,
                                                         double rValue,double gValue,double bValue,double aValue,double scale) {
    visualization_msgs::MarkerArray markerArray;

    visualization_msgs::Marker markerBase;
//    markerBase.ns = to_string(nsId);

    markerBase.action = visualization_msgs::Marker::ADD;
    markerBase.header.stamp = ros::Time::now();
    markerBase.header.frame_id = frame_id;
    markerBase.type = visualization_msgs::Marker::SPHERE;
    markerBase.scale.x = 2*scale*u(0);
    markerBase.scale.y = 2*scale*u(1);
    markerBase.scale.z = 2*scale*u(2);
    markerBase.color.r = rValue;
    markerBase.color.g = gValue;
    markerBase.color.b = bValue;
    markerBase.color.a = aValue;

    Quaternionf q(R.poseMat.rotation().matrix());
    markerBase.pose.orientation.x = q.x();
    markerBase.pose.orientation.y = q.y();
    markerBase.pose.orientation.z = q.z();
    markerBase.pose.orientation.w = q.w();
    int  N  = 10 ;
    VectorXf ts= VectorXf::LinSpaced(N,t0,tf);
    for (int n = 0 ; n < N ; n++){
        if (n > 0 )
            markerBase.color.a = aValue*0.5;
//        markerBase.id = n;
        double t = ts(n)-tLastUpdate;
        markerBase.pose.position.x = r.px.eval(t);
        markerBase.pose.position.y = r.py.eval(t);
        markerBase.pose.position.z = r.pz.eval(t);
        markerBase.id = nsId++;
        markerArray.markers.push_back(markerBase);
    }

    return markerArray;
}



int chasing_utils::sign(float x){
    return int(x > 0) - int(x < 0);
}

float chasing_utils::bearingAngle(PointSet targets,Point observer){
    // simple
    int M = targets.size();
    float angMax = -std::numeric_limits<float>::max();
    for (int m1 = 0 ; m1 < M ; m1++){
        for (int m2 = m1 + 1 ; m2 < M ; m2 ++ ){
            float l1 = targets.points[m1].distTo(observer);
            float l2 = targets.points[m2].distTo(observer);
            float l = targets.points[m1].distTo(targets.points[m2]);
            float ang = abs(acos((pow(l1,2)+pow(l2,2)-pow(l,2))/(2*l1*l2)));
            angMax = max (angMax, ang);
        }
    }
    return angMax;
}




bool chasing_utils::collisionRay(octomap_server::EdtOctomapServer *edf_ptr,
                                 Point pnt1, Point pnt2, float stride, float eps) {

    float length = pnt1.distTo(pnt2);
    Point dir = (pnt2 - pnt1);
    Point pnt;
    int N = ceil(length / stride);
    if (N != 0) {
        for (int n = 0; n <= N; n++) {
            pnt = pnt1 + dir * (n / float(N)) ;
            float dist = edf_ptr->getDistance(octomap::point3d(pnt.x, pnt.y, pnt.z));
            if (dist == -1) {
                printf("unknown point [%f, %f, %f] ? \n ", pnt.x, pnt.y, pnt.z);
            }
            if ((dist != -1) and dist <= eps) {

//            printf("point [%f, %f, %f] : collision with distance = %f \n ",pnt(0),pnt(1),pnt(2),dist);
                return true;
            }
        }
        return false;
    } else {
        pnt = pnt1;
        float dist = edf_ptr->getDistance(octomap::point3d(pnt.x, pnt.y, pnt.z));
        if (dist == -1) {
            printf("unknown point [%f, %f, %f] ? \n ", pnt.x, pnt.y, pnt.z);
        }
        if ((dist != -1) and dist <= eps) {
//            printf("point [%f, %f, %f] : collision with distance = %f \n ",pnt(0),pnt(1),pnt(2),dist);
            return true;
        }
        return false;
    }
}
