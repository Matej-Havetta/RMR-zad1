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
    // waypointQueue.emplace_back(40.0, 5.0);
    // real
    // waypointQueue.emplace_back(275, 0.0);
    // waypointQueue.emplace_back(0.0, 0.0);

    rotationPID = new PIDController(kp_rotation, ki_rotation, kd_rotation, 0.03, 0.1);
    distancePID = new PIDController(kp_distance, ki_distance, kd_distance, 10, 1);

    map = std::vector<std::vector<int>>(gridSize, std::vector<int>(gridSize, -1));

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

//this works for forward backwards movements
// std::vector<std::vector<int>> robot::updateMap(LaserMeasurement laserMeasurement, double xko, double yko, double fi){
//     const double scale = 0.1; //0.01; // what????
//     const double offsetX = gridSize / 2.0;
//     const double offsetY = gridSize / 2.0;

//     double robotRads = ((fi*pi)/180.0); // converting it to rads (((gyro)*pi)/180.0);

//     for (int i = 0; i < (laserMeasurement.numberOfScans); ++i) {
//         // float angle = laserMeasurement.Data[i].scanAngle;
//         float angle = (360-laserMeasurement.Data[i].scanAngle)*pi/180.0; // [rads]
//         float distance = laserMeasurement.Data[i].scanDistance/10.0; // [cm]
//                     // [cm]   [cm]
//         double x = xko*100 + distance * cos(angle + robotRads);
//         double y = yko*100 + distance * sin(angle + robotRads);

//         int gridX = static_cast<int>(x * scale + offsetX);
//         int gridY = static_cast<int>(y * scale + offsetY);

//         if (distance > 15 && distance <=300 && !(distance >= 64 && distance <= 70)) {
//             if (gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize) {
//                 map[gridY][gridX] = 1;
//             }
//             else{
//                 map[gridY][gridX] = 0;
//             }
//             // std::cout << map[gridY][gridX];
//         }
//         else{
//             map[gridY][gridX]=0;
//         }
//     }
//     return map;
// }
std::vector<std::vector<int>> robot::updateMap(LaserMeasurement laserMeasurement, double xko, double yko, double fi){
    const double scale = 0.1; //0.01; // what????
    const double offsetX = gridSize / 2.0;
    const double offsetY = gridSize / 2.0;
    double robotRads = ((fi*pi)/180.0); // converting it to rads (((gyro)*pi)/180.0);
    for (int i = 0; i < laserMeasurement.numberOfScans; i++) {
        // clockwise float angle = (360 - laserMeasurement.Data[i].scanAngle) * pi / 180.0;
        float angle = laserMeasurement.Data[i].scanAngle * pi / 180.0;
        float distance = laserMeasurement.Data[i].scanDistance / 10.0; // [cm]
        unsigned int pointTimestamp = laserMeasurement.Data[i].timestamp;
        RobotPose pose = interpolatePose(pointTimestamp);
        if (distance > 15 && distance <=300 && !(distance >= 64 && distance <= 70)) {
            // Convert polar to Cartesian in robot frame
            double localX = distance * cos(angle);
            double localY = distance * sin(angle);
            double robotX = xko * 100;
            double robotY = yko * 100;

            // Transform to global coordinates
            double globalX = pose.x * 100 + localX * cos(pose.angle) - localY * sin(pose.angle);
            double globalY = pose.y * 100 + localX * sin(pose.angle) + localY * cos(pose.angle);

            // Map to grid
            int gridX = static_cast<int>(globalX * scale + offsetX);
            int gridY = static_cast<int>(globalY * scale + offsetY);

            if (gridX >= 0 && gridX < gridSize && gridY >= 0 && gridY < gridSize) {
                map[gridX][gridY] = 1;
            }
        }
    }
    return map;
 }

void robot::drawMap(std::vector<std::vector<int>> map){
    std::ofstream outfile("C:\\Users\\szdor\\Desktop\\I-RK\\SEM8\\RMR\\mapa2.txt");
    for (int i = gridSize-1; i >= 0; i--) {
        for (int j = 0; j < gridSize; j++) {
            if(map[i][j] == 0){
                outfile << " ";
            }
            else if( map[i][j]==-1){
                outfile << ' ';
            }
            else if(map[i][j]==1){
                outfile << 1;
            }
            else{
                cout << "WHAT THE JSS FCK";
                cout << endl;
            }
        }
        cout << endl;
        outfile << std::endl;
    }
    outfile.close();
    cout<<"zapisal som"<<endl;
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
        // std::cout << x;
        ///toto neodporucam na nejake komplikovane struktury. signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow. ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde
    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    /*if(useDirectCommands==0)
    {
        if(forwardspeed==0 && rotationspeed!=0)
            robotCom.setRotationSpeed(rotationspeed);
        else if(forwardspeed!=0 && rotationspeed==0)
            robotCom.setTranslationSpeed(forwardspeed);
        else if((forwardspeed!=0 && rotationspeed!=0))
            robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
        else
            robotCom.setTranslationSpeed(0);
    }
    datacounter++;*/

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
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    map = updateMap(copyOfLaserData, xko, y, fi);
    drawMap(map);

    //robotdata.synctimestamp;

    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    // std::cout << "\n";
    // std::cout << laserData.Data[0].scanAngle;
    // std::cout << "\n";
    // std::cout << laserData.Data[1].scanAngle;
    // std::cout << "\n";
    // std::cout << laserData.Data[2].scanAngle;
    // std::cout << "\n";
    // std::cout << laserData.Data[200].scanAngle;

    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

}

robot::RobotPose robot::interpolatePose(unsigned int timestamp) {
    if (poseHistory.empty()) {
        return RobotPose(0, 0.0, 0.0, 0.0);
    }
    if (poseHistory.size() < 2) return poseHistory.back();
    for (size_t i = 1; i < poseHistory.size(); ++i) {
        RobotPose& before = poseHistory[i - 1];
        RobotPose& after = poseHistory[i];
        if (before.timestamp <= timestamp && timestamp <= after.timestamp) {
            double ratio = (timestamp - before.timestamp) / double(after.timestamp - before.timestamp);

            double x = before.x + ratio * (after.x - before.x);
            double y = before.y + ratio * (after.y - before.y);

            // Interpolating angle properly (handles wrap-around)
            double angleDiff = atan2(sin(after.angle - before.angle), cos(after.angle - before.angle));
            double angle = before.angle + ratio * angleDiff;

            return {timestamp, x, y, angle};
        }
    }
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
