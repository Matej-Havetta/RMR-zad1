#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QWidget>
#include "librobot.h"
#include <deque>
#include "PID.h"
#ifndef DISABLE_OPENCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include <cmath>
#include <stdio>

Q_DECLARE_METATYPE(cv::Mat)
#endif
#ifndef DISABLE_SKELETON
Q_DECLARE_METATYPE(skeleton)
#endif
Q_DECLARE_METATYPE(LaserMeasurement)
class robot : public QObject
{
    Q_OBJECT
    struct RobotPose {
        unsigned int timestamp;  // microseconds
        double x;
        double y;
        double angle; // radians
        RobotPose(unsigned int ts, double x_, double y_, double angle_)
            : timestamp(ts), x(x_), y(y_), angle(angle_) {}
    };
    struct Cella {
        int x;
        int y;
    };

public:
    explicit robot(QObject *parent = nullptr);

    void initAndStartRobot(std::string ipaddress);
    void calculateXY(TKobukiData robotdata);
    std::vector<std::vector<int>> updateMap(LaserMeasurement laserMeasurement, double xko, double yko, double fi);
    std::vector<std::vector<int>> updateCostMap();
    std::vector<std::vector<int>> updateCostMapFloodFill(Cella goal, Cella start);
    void drawMap(std::vector<std::vector<int>> map);
    void drawCostMap(std::vector<std::vector<int>> map);
    RobotPose interpolatePose(unsigned int timestamp,int &prevIndex);
    //tato funkcia len nastavuje hodnoty.. posielaju sa v callbacku(dobre, kvoli asynchronnosti a zabezpeceniu,ze sa poslu len raz pri viacero prepisoch vramci callu)
    void setSpeedVal(double forw,double rots);
    //tato funkcia fyzicky posiela hodnoty do robota
    void setSpeed(double forw,double rots);
signals:
    void publishPosition(double x, double y, double z);
    void publishLidar(const LaserMeasurement &lidata);
    #ifndef DISABLE_OPENCV
    void publishCamera(const cv::Mat &camframe);
#endif
#ifndef DISABLE_SKELETON
void publishSkeleton(const skeleton &skeledata);
#endif
private:
    /// toto su vase premenne na vasu odometriu
    double xko;
    double y;
    double fi;

///-----------------------------
/// toto su rychlosti ktore sa nastavuju setSpeedVal a posielaju v processThisRobot
    double forwardspeed;//mm/s
    double rotationspeed;//omega/s
    const float wheelDia=0.05;
    const float wheelBase=0.32;
    double currentForwardSpeed;
    double currentRotationSpeed;
    double prevFi;
    double gyroStart;
    double prevGyro;
    unsigned short previousEncoderRight;
    unsigned short previousEncoderLeft;

    ///waypoints
    std::deque<std::pair<double, double>> waypointQueue;
    std::deque<Cella> goalsQueue;
    std::deque<RobotPose> poseHistory; // store last N seconds

    ///controllers
    PIDController *rotationPID;
    PIDController *distancePID;
    bool encodersSetFlag;
    int angleRefiner=2;

    ///map
    // the size of the map is 120x120 which means, eachunit represents 10cmx10cm, valid values are: -1 (UNKNOWN), 0 (FREE), -1 (OCCUPIED), map is initialized with values -1
    bool goalIsSet = false;
    bool replanNeeded = false;
    Cella currentGoal;
    const int gridSize = 150;
    std::vector<std::vector<int>> map;
    std::vector<std::vector<int>> costMap;

    ///toto su callbacky co sa sa volaju s novymi datami
    int processThisLidar(LaserMeasurement laserData);
    int processThisRobot(TKobukiData robotdata);
    #ifndef DISABLE_OPENCV
    int processThisCamera(cv::Mat cameraData);
#endif


    ///pomocne strukutry aby ste si trosku nerobili race conditions
    LaserMeasurement copyOfLaserData;
    #ifndef DISABLE_OPENCV
    cv::Mat frame[3];
#endif
    ///classa ktora riesi komunikaciu s robotom
    libRobot robotCom;


    ///pomocne premenne... moc nerieste naco su
    int datacounter;
    #ifndef DISABLE_OPENCV
    bool useCamera1;
    int actIndex;
#endif


#ifndef DISABLE_SKELETON
int processThisSkeleton(skeleton skeledata);
int updateSkeletonPicture;
     skeleton skeleJoints;
#endif
    int useDirectCommands;


};

#endif // ROBOT_H
