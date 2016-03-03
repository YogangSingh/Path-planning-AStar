#include "Aria.h"
#include "astar/astar.h"
#include "header.h"

#include <algorithm>
#include <math.h>
#include <cstring>
int mymap[1000*1000];
myRect * rect;

bool laserflag = true;

class ActionBug2 : public ArAction
{
public:
    ActionBug2(double maxSpeed,double stopDistance,ArPose goal);
    virtual ~ActionBug2(void) {};
    virtual ArActionDesired *fire(ArActionDesired currentDesired);
    virtual void setRobot(ArRobot *robot);
protected:
    ArRangeDevice *mySonar;
    std::map<int,ArLaser*>  *lasers;
    ArActionDesired myDesired;
    ArPose mygoal;
    int end;
    double myMaxSpeed;
    double myStopDistance;
    bool seekgoal;
    int anglefollowwall();
    map_vis mmmap;
};

ActionBug2::ActionBug2(double maxSpeed,double stopDistance,ArPose goal):
    ArAction("Bug2")
{
    mySonar = NULL;
    myMaxSpeed = maxSpeed;
    mygoal = goal;
    seekgoal = true;
    myStopDistance = stopDistance;
    setNextArgument(ArArg("maximum speed",&myMaxSpeed,"Maximum speed to go."));
    setNextArgument(ArArg("stop distance",&myStopDistance, "Distance at which to stop."));
    end = (floor((goal.getY()+250)/500) + 500)*1000+(floor((goal.getX()+250)/500)+500);
    mymap[end] = 9;
    mmmap.init(40000,40000,20000,20000);
    mmmap.update_image((int)goal.getY(),(int)goal.getX(),2);
}
void ActionBug2::setRobot(ArRobot *robot)
{
    ArAction::setRobot(robot);
    mySonar = robot->findRangeDevice("sonar");
    
    laserflag && (lasers = robot->getLaserMap());
    
    if(robot == NULL)
    {
        ArLog::log(ArLog::Terse, "test: ActionBug2: Warning: I found no sonar,deactivating.");
        deactivate();
    }
}
ArActionDesired *ActionBug2::fire(ArActionDesired currentDesired)
{
    myDesired.reset();
    // 绘制机器人当前位置
    mmmap.update_image((int)myRobot->getY(),(int)myRobot->getX(),-1);
    
    if(mySonar == NULL)
    {
        deactivate();
        return NULL;
    }
    double vel,headto;
    // get goal
    if(300 >= myRobot->findDistanceTo(mygoal))
    {
        printf("  ____      _     _   _             ____             _   _ \n");
        printf(" / ___| ___| |_  | |_| |__   ___   / ___| ___   __ _| | | |\n");
        printf("| |  _ / _ \\ __| | __| '_ \\ / _ \\ | |  _ / _ \\ / _` | | | |\n");
        printf("| |_| |  __/ |_  | |_| | | |  __/ | |_| | (_) | (_| | | |_|\n");
        printf(" \\____|\\___|\\__|  \\__|_| |_|\\___|  \\____|\\___/ \\__,_|_| (_)\n");
        
        myRobot->keyHandlerExit();
        
        myDesired.setVel(0);
        myDesired.setRotVel(0);
        return &myDesired;

    }
    const std::list<ArPoseWithTime *> *readings;
   
    if(laserflag)
    {
        // use laser
        for (std::map<int,ArLaser*>::const_iterator i = lasers->begin();i!=lasers->end(); i++) {
            ArLaser* myLaser = (*i).second;
            if(!myLaser)
                continue;
            readings = myLaser->getCurrentBuffer();
            for(std::list<ArPoseWithTime *>::const_iterator it = readings->begin();it != readings->end();it++)
            {
                //绘制探测到到障碍物
                mmmap.update_image((int)(*it)->getY(),(int)(*it)->getX());
                //更新算法中使用的地图矩阵
                int obs = (floor(((*it)->getY() + 250)/500+500)*1000+floor(((*it)->getX() + 250)/500+500));
                mymap[obs] = 1;
            }
        }
    } else {
        //use sonar
        readings = mySonar->getCurrentBuffer();
        for(std::list<ArPoseWithTime *>::const_iterator it = readings->begin();it != readings->end();it++)
        {
            int obs = (floor(((*it)->getY() + 250)/500+500)*1000+floor(((*it)->getX() + 250)/500+500));
            mymap[obs] = 1;
            mmmap.update_image((int)(*it)->getY(),(int)(*it)->getX());
        }
    }
    
    
    double angle;
    double range = mySonar -> currentReadingPolar(5,-5, &angle) - 270; //四周障碍物
    double frange = mySonar -> currentReadingPolar(-30,30);            //前方障碍物
    
    ArPose ro = myRobot->getPose();
    int cur = (int((ro.getY()+250)/500+500))*1000+int((ro.getX()+250)/500+500); //当前位置
    
    AStar *tmp = new AStar(1000,1000,cur,end); //路径规划
    if(tmp->FindPath())
    {
        tmp->getResultPath();
        
        for(std::list<_Rect>::iterator iter = tmp->result.begin();
            iter != tmp->result.end();iter++)
            //绘制预测路径
            mmmap.update_image((iter->y-500)*500,(iter->x-500)*500,1);
        myRect nextstep;
        if(tmp->result.size() > 2)
        {
            nextstep = *(--(--(tmp->result.end())));
            headto = myRobot->findDeltaHeadingTo(ArPose((nextstep.x-500)*500,(nextstep.y-500)*500));
            myDesired.setDeltaHeading(headto);
            if(abs(headto)> 60)
                vel= range * 0.05;
            else if(range < 100 && abs(headto) > 30)
                vel = 10;
            else
                vel = frange * 0.15;
        }
        else
        {
            headto = myRobot->findDeltaHeadingTo(mygoal);
            myDesired.setDeltaHeading(headto);
            if(abs(headto)> 60)
                vel  = range * 0.05;
            else if (range < 100 && abs(headto) > 30)
                vel = 10;
            else
                vel = frange * 0.15;
        }
        if(abs(angle -headto) < 20 && range < 50)
        {
            myDesired.setRotVel(0);
            vel = -80;
        }

        cvWaitKey(1);
        for(std::list<_Rect>::iterator iter = tmp->result.begin();
            iter != tmp->result.end();iter++)
            mmmap.update_image((iter->y-500)*500,(iter->x-500)*500,3);
    }
    else
    {
        //  迷之输出
        printf(" _   _        __        __            _ \n");
        printf("| \\ | | ___   \\ \\      / /_ _ _   _  | |\n");
        printf("|  \\| |/ _ \\   \\ \\ /\\ / / _` | | | | | |\n");
        printf("| |\\  | (_) |   \\ V  V / (_| | |_| | |_|\n");
        printf("|_| \\_|\\___/     \\_/\\_/ \\__,_|\\__, | (_)\n");
        printf("                              |___/   \n");
        // output end
        delete tmp;
        myRobot->keyHandlerExit();
    }
    delete tmp;
    if(vel > myMaxSpeed)
        vel = myMaxSpeed;
    
    myDesired.setVel(vel);
    return &myDesired;
}

int main(int argc, char** argv)
{
    memset(mymap,0,sizeof(int)*1000*1000);
    int goalx,goaly;
    std::cout<<":::::::::::::::::::::::::::::::::::"<<std::endl;
    std::cout<<"Please set a goal(x,y):"<<std::endl;
    std::cin>> goalx >> goaly;
    std::cout<<":::::::::::::::::::::::::::::::::::"<<std::endl;
    
    Aria::init();
    ArArgumentParser argParser(&argc,argv);
    argParser.loadDefaultArguments();
    ArRobot robot;
    ArRobotConnector robotConnector(&argParser,&robot);
    ArAnalogGyro gyro(&robot);
    if (!robotConnector.connectRobot())
    {
        ArLog::log(ArLog::Terse, "Could not connect to the robot.");
        if (argParser.checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
            return 1;
        }
    }
    if(!robot.isConnected())
    {
        ArLog::log(ArLog::Terse, "Internal error: robot connector succeeded but ArRobot::isConnected() is false!");
    }
    
    ArLaserConnector laserConnector(&argParser,&robot, &robotConnector);
    
    ArCompassConnector compassConnector(&argParser);
    
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);
    
    ArSonarDevice sonar;
    robot.addRangeDevice(&sonar);

    robot.runAsync(true);
    
    if(!laserConnector.connectLasers(false,false,true))
    {
        ArLog::log(ArLog::Terse,"Counld not connect to configured lasers. Use sonar only.");
        laserflag = false;
    }
    ArUtil::sleep(500);
    
    robot.lock();

    if(robot.getOrigRobotConfig()->getHasGripper())
        new ArModeGripper(&robot,"gripper",'g','G');

    ArActionStallRecover recover;
    ArActionBumpers bumpers;
    
    robot.comInt(ArCommands::ENABLE, 1);
    robot.unlock();
    ActionBug2 bug(300,200,ArPose(goalx,goaly));
  
    robot.addAction(&bug,60);
    robot.waitForRunExit();
       
    Aria::exit(0);
    return 0;
}

