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
    goalsQueue.emplace_back(Cella{140,220});


    rotationPID = new PIDController(kp_rotation, ki_rotation, kd_rotation, 0.03, 0.1);
    distancePID = new PIDController(kp_distance, ki_distance, kd_distance, 10, 1);

    map = std::vector<std::vector<int>>(gridSize, std::vector<int>(gridSize, 0));
    //costMap = std::vector<std::vector<int>>(gridSize, std::vector<int>(gridSize, 0));
    costMap = readMapFromFile("C://Users//szdor//Desktop//I-RK//SEM8//RMR//mapa1_DONE.txt");

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
    if (costMap.empty() || costMap[0].empty())
    {
        std::cerr << "Map is empty!\n";
        return {};
    }
    const double scale = 0.1; //cm
    const double offsetX = gridSize / 2.0;
    const double offsetY = gridSize / 2.0;
    int rows = costMap.size();
    int cols = costMap[0].size();

    std::vector<std::vector<int>> newCostMap = costMap;
    int gridGoalX = static_cast<int>(goal.x * scale + offsetX);
    int gridGoalY = static_cast<int>(goal.y * scale + offsetY);
    Cella gridGoal = {gridGoalX, gridGoalY};
    if(newCostMap.size()!=rows)
        std::cout<<"pruser"<<std::endl;
    if (gridGoalX < 0 || gridGoalY < 0 || gridGoalX >= cols || gridGoalY >= rows || map[gridGoalY][gridGoalX] == -1) {
        std::cerr << "Invalid goal for flood fill\n";
        return costMap ;
    }
    std::vector<Cella> checkpointVect;
    std::deque<Cella> queue;
    queue.push_back(gridGoal); //    queue.push_back({gridGoalX,gridGoalY});
    newCostMap[gridGoalY][gridGoalX] = 2;  // Start cost

    const std::vector<std::pair<int, int>> directions = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1}
    };

    while (!queue.empty()) {
        Cella currentCell = queue.front();
        //int x = static_cast<int>(currentCell.x * scale + offsetX);        int y = static_cast<int>(currentCell.y * scale + offsetY);
        int x = currentCell.x;
        int y = currentCell.y;
        queue.pop_front();
        int currentCost = newCostMap[y][x];
        for (const std::pair<int, int>& dir : directions) {
            int dx = dir.first;
            int dy = dir.second;
            int nx = x + dx;
            int ny = y + dy;

            // Check bounds
            if (nx >= 0 && ny >= 0 && nx < cols && ny < rows) {
                if (newCostMap[ny][nx] == 0) { //obstacle is -2
                    newCostMap[ny][nx] = currentCost + 1;
                    queue.push_back({nx, ny});
                }
                else if (map[ny][nx] == -1) {
                    checkpointVect.push_back(goal);
                    return newCostMap;
                }
            }
        }
        //checkpointVect.push_back(queue.back());
        currentCost++;
    }
    std::cout << "Cost map updated from goal (" << goal.x << ", " << goal.y << ")\n";
    return newCostMap;
}

std::vector<robot::Cella> robot::extractPathFromCostMap(Cella start) { // const std::vector<std::vector<int>>& costMap
    std::vector<Cella> path;
    const double scale = 0.1; //cm
    const double offsetX = gridSize / 2.0;
    const double offsetY = gridSize / 2.0;
    // Convert to grid coordinates if needed
    int gridStartX = static_cast<int>(start.x * 0.1 + gridSize / 2.0);
    int gridStartY = static_cast<int>(start.y * 0.1 + gridSize / 2.0);
    Cella current;
    current.x = gridStartX;
    current.y = gridStartY;

    int rows = static_cast<int>(costMap.size());
    int cols = static_cast<int>(costMap[0].size());

    if (current.x < 0 || current.y < 0 || current.x >= cols || current.y >= rows)
        return path;

    //path.push_back(current);
    int currentCost = costMap[current.y][current.x];

    std::vector<std::pair<int, int>> directions;
    directions.push_back(std::make_pair(1, 0));
    directions.push_back(std::make_pair(-1, 0));
    directions.push_back(std::make_pair(0, 1));
    directions.push_back(std::make_pair(0, -1));

    while (currentCost > 2) {
        int minCost = INT_MAX;
        Cella nextCell = current;

        for (int i = 0; i < static_cast<int>(directions.size()); i++) {
            int dx = directions[i].first;
            int dy = directions[i].second;

            int nx = current.x + dx;
            int ny = current.y + dy;

            if (nx >= 0 && ny >= 0 && nx < cols && ny < rows) {
                int neighborCost = costMap[ny][nx];
                if (neighborCost >= 2 && neighborCost < minCost) {
                    minCost = neighborCost;
                    nextCell = {nx, ny};
                }
            }
        }

        if (minCost < currentCost) {
            current = nextCell;
            currentCost = minCost;
            int globX = static_cast<int>((current.x - offsetX) / scale);
            int globY = static_cast<int>((current.y - offsetY) / scale);
            Cella globalCurrent = {globX, globY};
            path.push_back(globalCurrent);
        } else {
            std::cerr << "Backtracking failed. Could not find a neighbor with lower cost.\n";
            return std::vector<Cella>();  // return empty path
        }
    }
    // while (currentCost > 2) {
    //     bool found = false;
    //     for (int i = 0; i < static_cast<int>(directions.size()); i++) {
    //         int dx = directions[i].first;
    //         int dy = directions[i].second;

    //         int nx = current.x + dx;
    //         int ny = current.y + dy;

    //         if (nx >= 0 && ny >= 0 && nx < cols && ny < rows) {
    //             if (costMap[ny][nx] == currentCost - 1) {
    //                 current.x = nx;
    //                 current.y = ny;
    //                 currentCost--;
    //                 int globX = (current.x / scale - offsetX);
    //                 int globY = (current.y / scale - offsetY);
    //                 Cella globalCurrent=Cella({globX,globY});
    //                 //path.push_back(current);
    //                 path.push_back(globalCurrent);
    //                 found = true;
    //                 break;
    //             }
    //         }
    //     }

    //     if (!found) {
    //         std::cerr << "Fuck, backtracking failed. Could not find a neighbor with less cost.\n";
    //         return std::vector<Cella>();  // return empty path
    //     }
    // }

    //std::reverse(path.begin(), path.end());
    return path;
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
                cout << "no more swear words, I swear";
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
            drawCostMap(costMap);
            // backtrackPath(robotCellX, robotCellY);
            replanNeeded = false;
            std::vector<Cella> checkpoints = extractPathFromCostMap(start);
            std::vector<Cella> zlomoveBody = findZlomoveBody(checkpoints);
            waypointQueue=checkpoints;
            std::cout << "idk";
        }
    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    if (useDirectCommands == 0) {
        //waypointQueue = path;
        if (!waypointQueue.empty()) {
            double angleDeviationThreshold = 0.1; //5.7 degrees
            double distanceDeviationThreshold = 0.1;

            Cella targetWaypoint = waypointQueue.front();
            double targetX = targetWaypoint.x;
            double targetY = targetWaypoint.y;

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
                //waypointQueue.pop_front();
                waypointQueue.erase(waypointQueue.begin());
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

std::vector<std::vector<int>> robot::readMapFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<std::vector<int>> map;

    if (!file.is_open()) {
        std::cerr << "Failed to open map file: " << filename << std::endl;
        return map;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.size() < 150) continue; // Skip malformed lines if necessary

        std::vector<int> row;
        for (char c : line) {
            switch (c) {
            case ' ': row.push_back(0); break;     // Free space
            case '1': row.push_back(-2); break;     // Wall or obstacle
            //case '-': row.push_back(-1); break;    // If you see this in the map, you have a reason to be unhappy.s
            default: row.push_back(0); break;
            }
        }
        map.push_back(row);
    }

    file.close();
    return map;
}

std::vector<robot::Cella> robot::findZlomoveBody(const std::vector<Cella>& path) {
    std::vector<Cella> zlomoveBody;

    if (path.size() < 2) {
        return path; // No turning points possible
    }
    zlomoveBody.push_back(path.front());

    // Previous direction
    int dxPrev = path[1].x - path[0].x;
    int dyPrev = path[1].y - path[0].y;

    for (size_t i = 2; i < path.size(); ++i) {
        int dx = path[i].x - path[i - 1].x;
        int dy = path[i].y - path[i - 1].y;

        // It's a turning point
        if (dx != dxPrev || dy != dyPrev) {
            zlomoveBody.push_back(path[i - 1]);
        }

        dxPrev = dx;
        dyPrev = dy;
    }

    // Add goal
    zlomoveBody.push_back(path.back());

    return zlomoveBody;
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
