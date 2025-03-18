#include "robot.h"
#include <deque>

//here i am fighting for PI
#define _USE_MATH_DEFINES
#include <cmath>
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
    const double kp_rotation = 0.3;
    const double ki_rotation = 0.01;
    const double kd_rotation = 0.05;
    const double dt_rotation = 0.01; // where the fuck do i get this from

    const double kp_distance = 0.10;
    const double ki_distance = 0.01;
    const double kd_distance = 0.02;
    const double dt_distance = 0.01; // where the fuck do i get this from

    xko=0.00;
    y=0.00;
    fi=0.00;
    prevFi=0.00;
    gyroStart=0.00;
    prevGyro=0.00;

    forwardspeed=0.0;
    rotationspeed=0.0;

    previousEncoderLeft=0;
    previousEncoderRight=0;

    //std::deque<std::pair<double, double>>
    waypointQueue.emplace_back(80.0, -20.0);
    waypointQueue.emplace_back(4.0, 70.0);

    rotationPID = new PIDController(kp_rotation, ki_rotation, kd_rotation, dt_rotation, 0.0, 0.0);
    distancePID = new PIDController(kp_distance, ki_distance, kd_distance, dt_distance, 0.0, 0.0);

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

    //distance
    short deltaEncoderRight = (robotdata.EncoderRight) - (previousEncoderRight);
    short deltaEncoderLeft = (robotdata.EncoderLeft) - (previousEncoderLeft);
    // update encoders
    previousEncoderRight = robotdata.EncoderRight;
    previousEncoderLeft = robotdata.EncoderLeft;
    // encoder to distance in metres
    double rightWheelDist = ((double) deltaEncoderRight) * robotCom.tickToMeter;
    double leftWheelDist = ((double) deltaEncoderLeft) * robotCom.tickToMeter;
    double deltaDistance = (rightWheelDist + leftWheelDist)/2;

    // uhol
    //double prevGyro;
    double gyro = robotdata.GyroAngle/100.00;
    double gyroRad = (((gyro)*pi)/180.0);
    //double deltaFi = (rightWheelDist - leftWheelDist) / wheelBase;
    // double deltaFi = (rightWheelDist - leftWheelDist);
    // double deltaFi = (deltaEncoderRight - deltaEncoderLeft);
    //fi += deltaFi;
    //double fiInRad = fi * (pi / 180.0);
    //double prevFiInRad= prevFi * (pi/180.0);
    //fi = atan2(sin(fi), cos(fi));

    //x,y
    if (prevGyro == gyroRad)
    {
        xko += deltaDistance * (double) cos(gyroRad);
        y += deltaDistance * (double) sin(gyroRad);
    }
    else
    {
        // xko += deltaDistance * (double)(sin(gyroRad) - sin(prevGyro*pi/180.00));
        // y -= deltaDistance * (double)(cos(gyroRad) - cos(prevGyro*pi/180.00));
        xko += deltaDistance * (double)(sin(gyroRad) - sin(prevGyro));
        y -= deltaDistance * (double)(cos(gyroRad) -  cos(prevGyro));
    }
    //prevFi = fi;
    prevGyro=gyroRad;
    fi=gyro;
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



///TU PISTE KOD... TOTO JE TO MIESTO KED NEVIETE KDE ZACAT,TAK JE TO NAOZAJ TU. AK AJ TAK NEVIETE, SPYTAJTE SA CVICIACEHO MA TU NATO STRING KTORY DA DO HLADANIA XXX

    ///kazdy piaty krat, aby to ui moc nepreblikavalo..
    if(datacounter%5==0)
    {
        calculateXY(robotdata);
        ///ak nastavite hodnoty priamo do prvkov okna,ako je to na tychto zakomentovanych riadkoch tak sa moze stat ze vam program padne
        // ui->lineEdit_2->setText(QString::number(robotdata.EncoderRight));
        //ui->lineEdit_3->setText(QString::number(robotdata.EncoderLeft));
        //ui->lineEdit_4->setText(QString::number(robotdata.GyroAngle));
        /// lepsi pristup je nastavit len nejaku premennu, a poslat signal oknu na prekreslenie
        /// okno pocuva vo svojom slote a vasu premennu nastavi tak ako chcete. prikaz emit to presne takto spravi
        /// viac o signal slotoch tu: https://doc.qt.io/qt-5/signalsandslots.html
        ///posielame sem nezmysli.. pohrajte sa nech sem idu zmysluplne veci
        emit publishPosition(xko*100,y*100,fi);
        //std::cout << x;
        ///toto neodporucam na nejake komplikovane struktury. signal slot robi kopiu dat. radsej vtedy posielajte
        /// prazdny signal a slot bude vykreslovat strukturu (vtedy ju musite mat samozrejme ako member premmennu v mainwindow. ak u niekoho najdem globalnu premennu,tak bude cistit bludisko zubnou kefkou.. kefku dodam)
        /// vtedy ale odporucam pouzit mutex, aby sa vam nestalo ze budete pocas vypisovania prepisovat niekde inde

    }
    ///---tu sa posielaju rychlosti do robota... vklude zakomentujte ak si chcete spravit svoje
    // if(useDirectCommands==0)
    // {
    //     if(forwardspeed==0 && rotationspeed!=0)
    //         robotCom.setRotationSpeed(rotationspeed);
    //     else if(forwardspeed!=0 && rotationspeed==0)
    //         robotCom.setTranslationSpeed(forwardspeed);
    //     else if((forwardspeed!=0 && rotationspeed!=0))
    //         robotCom.setArcSpeed(forwardspeed,forwardspeed/rotationspeed);
    //     else
    //         robotCom.setTranslationSpeed(0);
    // }
    // datacounter++;

    if (useDirectCommands == 0) {
        if (!waypointQueue.empty()) {
            double angleDeviationThreshold = 0.2;
            double distanceDeviationThreshold = 0.5;

            std::pair<double, double> targetWaypoint = waypointQueue.front();
            double targetX = targetWaypoint.first;
            double targetY = targetWaypoint.second;

            // Calculate desired angle
            double deltaX = targetX - xko*100;
            double deltaY = targetY - y*100;
            double desiredAngle = atan2(deltaY, deltaX);
            // Calculate angle error
            double angleError = desiredAngle - fi;
            angleError = atan2(sin(angleError), cos(angleError));
            // Calculate distance to waypoint
            double distance = sqrt(pow(deltaX, 2) + pow(deltaY, 2));

            // angle and distance derivation
            if(abs(angleError) > angleDeviationThreshold){
                // Rotation PID
                forwardspeed = 0.0;
                rotationspeed = rotationPID->update(0, angleError);
                std::cout << rotationspeed;
                std::cout << "\n";
            }
            else if((distance > distanceDeviationThreshold)){
                // Distance PID
                rotationspeed=0.0;
                forwardspeed = distancePID->update(0, distance);
            }
            if (distance < distanceDeviationThreshold) {
                waypointQueue.pop_front();
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
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
   // updateLaserPicture=1;
    emit publishLidar(copyOfLaserData);
   // update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia


    return 0;

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
