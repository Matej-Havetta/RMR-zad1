#include "robot.h"
#include <deque>

//here i am fighting for PI
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
const double pi = 3.14159265358979323846;


robot::robot(QObject *parent) : QObject(parent)
{
    qRegisterMetaType<LaserMeasurement>("LaserMeasurement");
    #ifndef DISABLE_OPENCV
    qRegisterMetaType<cv::Mat>("cv::Mat");
    #endif
    #ifndef DISABLE_SKELETON
    qRegisterMetaType<skeleton>("skeleton");
    #endif
}

void robot::initAndStartRobot(std::string ipaddress)
{
    // Defining PID gains
    const double kp_rotation = 3; //real
    const double ki_rotation = 0.01;
    const double kd_rotation = 0.05;

    // const double kp_distance = 10; //for faster simulation
    const double kp_distance = 20; //real
    const double ki_distance = 0.01;
    const double kd_distance = 0.02;

    xko=0.00;
    y=0.00;
    fi=0.00;
    prevFi=0.00;
    gyroStart=0.00;
    prevGyro=0.00;

    forwardspeed=0.0;
    rotationspeed=0.0;

    encodersSetFlag=false;
    previousEncoderLeft=0;
    previousEncoderRight=0;


    // simulation
    // waypointQueue.emplace_back(-5.0,0.0);
    // waypointQueue.emplace_back(40, 5);
    // real
    // waypointQueue.emplace_back(275, 0);
    // waypointQueue.emplace_back(0, 0);

    //goals
    goalsQueue.emplace_back(Cella{-5,10});
    goalsQueue.emplace_back(Cella{-50,100});


    rotationPID = new PIDController(kp_rotation, ki_rotation, kd_rotation, 0.03, 0.1);
    distancePID = new PIDController(kp_distance, ki_distance, kd_distance, 10, 1);

    map = std::vector<std::vector<int>>(gridSize, std::vector<int>(gridSize, 0));
    costMap = std::vector<std::vector<int>>(gridSize, std::vector<int>(gridSize, 0));

    ///setovanie veci na komunikaciu s robotom/lidarom/kamerou.. su tam adresa porty a callback.. laser ma ze sa da dat callback aj ako lambda.
    /// lambdy su super, setria miesto a ak su rozumnej dlzky,tak aj prehladnost... ak ste o nich nic nepoculi poradte sa s vasim doktorom alebo lekarnikom...
    robotCom.setLaserParameters(ipaddress,52999,5299,/*[](LaserMeasurement dat)->int{std::cout<<"som z lambdy callback"<<std::endl;return 0;}*/std::bind(&robot::processThisLidar,this,std::placeholders::_1));
    robotCom.setRobotParameters(ipaddress,53000,5300,std::bind(&robot::processThisRobot,this,std::placeholders::_1));
  #ifndef DISABLE_OPENCV
    robotCom.setCameraParameters("http://"+ipaddress+":8000/stream.mjpg",std::bind(&robot::processThisCamera,this,std::placeholders::_1));
#endif
   #ifndef DISABLE_SKELETON
      robotCom.setSkeletonParameters("127.0.0.1",23432,23432,std::bind(&robot::processThisSkeleton,this,std::placeholders::_1));
#endif
    ///ked je vsetko nasetovane tak to tento prikaz spusti (ak nieco nieje setnute,tak to normalne nenastavi.cize ak napr nechcete kameru,vklude vsetky info o nej vymazte)
    robotCom.robotStart();


}

void robot::calculateXY(TKobukiData robotdata) { // double& xko, double& y
    //should run only after init (I didnt want to modify the inistAndStartRobot method,it would need robotdata as an input
    if(encodersSetFlag==false){
        previousEncoderRight = robotdata.EncoderRight;
        previousEncoderLeft = robotdata.EncoderLeft;
        prevGyro=robotdata.GyroAngle/100.00;
        encodersSetFlag=true;
    }
    //distance
    short deltaEncoderRight = (robotdata.EncoderRight) - (previousEncoderRight);
    short deltaEncoderLeft = (robotdata.EncoderLeft) - (previousEncoderLeft);
    // handle unsigned short overflow
    int uShortLimit=65535;
    int half = uShortLimit / 2;
    if (deltaEncoderLeft > half)
        deltaEncoderLeft -= uShortLimit + 1;
    if (deltaEncoderLeft < -half)
        deltaEncoderLeft += uShortLimit + 1;
    if (deltaEncoderRight > half)
        deltaEncoderRight -= uShortLimit + 1;
    if (deltaEncoderRight < -half)
        deltaEncoderRight += uShortLimit + 1;
    // update encoders
    previousEncoderRight = robotdata.EncoderRight;
    previousEncoderLeft = robotdata.EncoderLeft;
    // encoder to distance in metres
    double rightWheelDist = ((double) deltaEncoderRight) * robotCom.tickToMeter;
    double leftWheelDist = ((double) deltaEncoderLeft) * robotCom.tickToMeter;
    double deltaDistance = (rightWheelDist + leftWheelDist)/2;

    // uhol
    //double prevGyro;
    double gyro = robotdata.GyroAngle/100.00 - prevGyro;
    double gyroRad = (((gyro)*pi)/180.0);
    //double deltaFi = (rigtWheelDist - leftWheelDist) / wheelBase;
    // double deltaFi = (rightWheelDist - leftWheelDist);
    // double deltaFi = (deltaEncoderRight - deltaEncoderLeft);
    //fi += deltaFi;
    //double fiInRad = fi * (pi / 180.0);
    //double prevFiInRad= prevFi * (pi/180.0);
    //fi = atan2(sin(fi), cos(fi));

    //x,y
   // if (prevGyro == gyroRad)
    {
        xko += deltaDistance * (double) cos(gyroRad);
        y += deltaDistance * (double) sin(gyroRad);
    }
   /* else
    {
        // xko += deltaDistance * (double)(sin(gyroRad) - sin(prevGyro*pi/180.00));
        // y -= deltaDistance * (double)(cos(gyroRad) - cos(prevGyro*pi/180.00));
        xko += deltaDistance * (double)(sin(gyroRad)- sin(prevGyro));
        y -= deltaDistance * (double)(cos(gyroRad) -  cos(prevGyro));
    }*/
    //prevFi = fi;
    //prevGyro=gyroRad;
    fi=gyro;
    poseHistory.emplace_back(robotdata.synctimestamp, xko, y, gyroRad);
}

std::vector<std::vector<int>> robot::updateMap(LaserMeasurement laserMeasurement, double xko, double yko, double fi){
    const double scale = 0.1; //cm
    const double offsetX = gridSize / 2.0;
    const double offsetY = gridSize / 2.0;
    static int prevIt=1;
    double robotRads = ((fi*pi)/180.0); // converting it to rads (((gyro)*pi)/180.0);
    for (int i = 0; i < laserMeasurement.numberOfScans; i++) {
        // clockwise float angle = (360 - laserMeasurement.Data[i].scanAngle) * pi / 180.0;
        float angle = -laserMeasurement.Data[i].scanAngle * pi / 180.0;
        float distance = laserMeasurement.Data[i].scanDistance / 10.0; // [cm]
        unsigned int pointTimestamp = laserMeasurement.Data[i].timestamp;
        RobotPose pose = interpolatePose(pointTimestamp,prevIt);
        if (distance > 15 && distance <=300 && !(distance >= 64 && distance <= 70)) {

            // Transform to global coordinates
            double globalX = pose.x + distance * cos(angle + pose.angle);
            double globalY = pose.y + distance * sin(angle + pose.angle);

            // Map to grid
            int gridX = static_cast<int>(globalX * scale + offsetX);
            int gridY = static_cast<int>(globalY * scale + offsetY);

            if (gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize) {
                map[gridX][gridY] = -2;
            }
        }
    }
    return map;
 }

std::vector<std::vector<int>> robot::updateCostMapFloodFill(Cella goal, Cella start)
{
    if (map.empty() || map[0].empty()) {
        std::cerr << "Map is empty!\n";
        return {};
    }
    const double scale = 0.1; //cm
    const double offsetX = gridSize / 2.0;
    const double offsetY = gridSize / 2.0;
    int rows = map.size();
    int cols = map[0].size();

    std::vector<std::vector<int>> newCostMap = map;
    int gridGoalX = static_cast<int>(goal.x * scale + offsetX);
    int gridGoalY = static_cast<int>(goal.y * scale + offsetY);
    Cella gridGoal = {gridGoalX, gridGoalY};
    if (gridGoalX < 0 || gridGoalY < 0 || gridGoalX >= cols || gridGoalY >= rows || map[gridGoalY][gridGoalX] == -1) {
        std::cerr << "Invalid goal for flood fill\n";
        return costMap ;
    }
    std::vector<Cella> checkpointVect;
    std::deque<Cella> queue;
    queue.push_back(gridGoal); //    queue.push_back({gridGoalX,gridGoalY});
    newCostMap[gridGoalY][gridGoalX] = 2;  // Start cost

    // Directions: 4-connected neighbors (you can expand to 8 if you want diagonals)
    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    while (!queue.empty()) {
        Cella currentCell = queue.front();
        //int x = static_cast<int>(currentCell.x * scale + offsetX);        int y = static_cast<int>(currentCell.y * scale + offsetY);
        int x = currentCell.x;
        int y = currentCell.y;
        queue.pop_front();
        if(y>149 || x>149){
            std::cout << y;
            std::cout << x;
        }
        int currentCost = newCostMap[y][x];
        for (const std::pair<int, int>& dir : directions) {
            int dx = dir.first;
            int dy = dir.second;
            int nx = x + dx;
            int ny = y + dy;

            // Check bounds
            // if (nx >= 0 && ny >= 0 && nx < cols
            if (ny >= 0 && ny < map.size()) {
                if (nx >= 0 && nx < map[ny].size()) {
                // Only spread to free cells that haven't been visited
                if (map[ny][nx] == 0 && newCostMap[ny][nx] == 0) { //obstacle is -2
                    newCostMap[ny][nx] = currentCost + 1;
                    queue.push_back({nx, ny});
                } else if (map[ny][nx] == -1) {
                    return newCostMap;
                }
                // if(map[ny][nx] == -1){
                //     //checkpointVect.push_back(goal);
                //     return newCostMap;
                // }

            }
            else {
                std::cerr << "Invalid access at (" << nx << ", " << ny << ")\n";
            }
        }
        //checkpointVect.push_back(queue.back());
        currentCost++;
    }
    std::cout << "Cost map updated from goal (" << goal.x << ", " << goal.y << ")\n";
    return newCostMap;
}

void robot::drawMap(std::vector<std::vector<int>> map){
    std::ofstream outfile("C:\\Users\\szdor\\Desktop\\I-RK\\SEM8\\RMR\\mapa2.txt");
    for (int i = gridSize-1; i >= 0; i--) {
        for (int j = 0; j < gridSize; j++) {
            if(map[i][j] == 0){
                outfile << "0";
            }
            else if( map[i][j]==-1){
                outfile << "-1";
            }
            else if(map[i][j]==-2){
                outfile << "-2";
            }
            else{
                cout << "WHAT THE JSS FCK";
            }
        }
        cout << endl;
        outfile << std::endl;
    }
    outfile.close();
    cout<<"zapisal som"<<endl;
}

void robot::drawCostMap(std::vector<std::vector<int>> map){
    std::ofstream outfile("C:\\Users\\szdor\\Desktop\\I-RK\\SEM8\\RMR\\costMap.txt");
    for (int i = gridSize-1; i >= 0; i--) {
        for (int j = 0; j < gridSize; j++) {
                outfile << map[i][j];
        }
        cout << endl;
        outfile << std::endl;
    }
    outfile.close();
    cout<<"zapisal som costmap"<<endl;
}

void robot::setSpeedVal(double forw, double rots)
{
    forwardspeed=forw;
    rotationspeed=rots;
    useDirectCommands=0;
}

void robot::setSpeed(double forw, double rots)
{
    if(forw==0 && rots!=0)
    {
        robotCom.setRotationSpeed(rots);
        currentRotationSpeed=rots;
        currentForwardSpeed=forw;

    }
    else if(forw!=0 && rots==0)
    {
        robotCom.setTranslationSpeed(forw);
        currentRotationSpeed=rots;
        currentForwardSpeed=forw;
    }
    else if((forw!=0 && rots!=0))
    {
        robotCom.setArcSpeed(forw,forw/rots);
        currentRotationSpeed=rots;
        currentForwardSpeed=forw;
    }
    else
    {
        robotCom.setTranslationSpeed(0);
        currentRotationSpeed=rots;
        currentForwardSpeed=forw;
    }
    useDirectCommands=1;
}

///toto je calback na data z robota, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa vzdy ked dojdu nove data z robota. nemusite nic riesit, proste sa to stane
int robot::processThisRobot(TKobukiData robotdata)
{
    ///tu mozete robit s datami z robota
    calculateXY(robotdata);
    if (!goalIsSet && !goalsQueue.empty()) {
        currentGoal = goalsQueue.front();
        goalsQueue.pop_front();
        goalIsSet = true;
        replanNeeded = true;
    }
///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX
    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(xko*100,y*100,fi);
        ///toto neodporucam na nejake komplikovane struktury. signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow. ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
    }
    if(datacounter%20==0){
        if (goalIsSet && replanNeeded) {
            int robotCellX = xko;
            int robotCellY = y;
            Cella start = Cella({robotCellX,robotCellY});
            costMap = updateCostMapFloodFill(currentGoal, start);
            // backtrackPath(robotCellX, robotCellY);
            //replanNeeded = false;
        }
    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if (useDirectCommands == 0) {
        if (!waypointQueue.empty()) {
            double angleDeviationThreshold = 0.1; //5.7 degrees
            double distanceDeviationThreshold = 0.1;

            std::pair<double, double> targetWaypoint = waypointQueue.front();
            double targetX = targetWaypoint.first;
            double targetY = targetWaypoint.second;

            // Calculate desired angle
            double deltaX = targetX - xko*100;
            double deltaY = targetY - y*100;
            double desiredAngle = atan2(deltaY, deltaX);
            // Calculate angle error
            double angleRN= fi*3.14159265358979323846/180;
            double angleError = desiredAngle - angleRN ;
            angleError = atan2(sin(angleError), cos(angleError));
            // Calculate distance to waypoint
            double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

            // angle and distance derivation
            if((distance > distanceDeviationThreshold)){
                // Distance PID
                rotationspeed=0.0;
                forwardspeed = distancePID->update(0, distance);

                if(forwardspeed<15){
                    forwardspeed=25;
                }
                 if(forwardspeed>200){
                     forwardspeed=200;
                 }

            }
            if(abs(angleError) > angleDeviationThreshold*angleRefiner){
                // Rotation PID
                forwardspeed = 0.0;
                rotationspeed = rotationPID->update(0, angleError);
                angleRefiner =1;
                distancePID->setPrevOut(0.0);
            }
            else{
                angleRefiner =2;
                rotationPID->setPrevOut(0.0);

            }
            if (distance < distanceDeviationThreshold) {
                waypointQueue.pop_front();
                std::cout << "POINT REACHED" ;
            }

            // Apply speeds to robot
            if (forwardspeed == 0 && rotationspeed != 0) {
                robotCom.setRotationSpeed(rotationspeed);
            } else if (forwardspeed != 0 && rotationspeed == 0) {
                robotCom.setTranslationSpeed(forwardspeed);
            } else if (forwardspeed != 0 && rotationspeed != 0) {
                robotCom.setArcSpeed(forwardspeed, forwardspeed / rotationspeed);
            } else {
                robotCom.setTranslationSpeed(0);
            }
        }
        else {
            robotCom.setTranslationSpeed(0);
            robotCom.setRotationSpeed(0);
        }
    }
    datacounter++;
    return 0;
}

///toto je calback na data z lidaru, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z lidaru
int robot::processThisLidar(LaserMeasurement laserData)
{
    static int init=0;
    if(init==1){
        map = updateMap(copyOfLaserData, xko, y, fi);
    }
    init=1;
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    drawMap(map);
    //drawCostMap(costMap);
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
    //update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia
    return 0;

}

robot::RobotPose robot::interpolatePose(unsigned int timestamp,int &prevIndex) {
    if (poseHistory.empty()) {
        prevIndex=1;
        return RobotPose(0, 0.0, 0.0, 0.0);
    }
    if (poseHistory.size() < 2){
        prevIndex=1;
        return poseHistory.back();
    }
    for (size_t i = prevIndex; i < poseHistory.size(); ++i) {
        RobotPose& before = poseHistory[i - 1];
        RobotPose& after = poseHistory[i];
        if (before.timestamp <= timestamp && timestamp <= after.timestamp) {
            prevIndex=i;
            double ratio = (timestamp - before.timestamp) / double(after.timestamp - before.timestamp);

            double x = before.x*100 + ratio * (after.x - before.x);
            double y = before.y*100 + ratio * (after.y - before.y);

            double angleDiff = atan2(sin(after.angle - before.angle), cos(after.angle - before.angle));
            double angle = before.angle + ratio * angleDiff;

            return {timestamp, x, y, angle};
        }
    }
    prevIndex=1;
    return poseHistory.back();
}


  #ifndef DISABLE_OPENCV
///toto je calback na data z kamery, ktory ste podhodili robotu vo funkcii initAndStartRobot
/// vola sa ked dojdu nove data z kamery
int robot::processThisCamera(cv::Mat cameraData)
{

    cameraData.copyTo(frame[(actIndex+1)%3]);//kopirujem do nasej strukury
    actIndex=(actIndex+1)%3;//aktualizujem kde je nova fotka

    emit publishCamera(frame[actIndex]);
    return 0;
}
#endif

  #ifndef DISABLE_SKELETON
/// vola sa ked dojdu nove data z trackera
int robot::processThisSkeleton(skeleton skeledata)
{

    memcpy(&skeleJoints,&skeledata,sizeof(skeleton));

    emit publishSkeleton(skeleJoints);
    return 0;
}
#endif
