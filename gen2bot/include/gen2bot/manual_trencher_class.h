// This class is the base class for all essential motor declarations, functions, and variables
#include <gen2bot/base_trencher_class.h>

// This class controls all manual motor functions besides wheels
class manual_trencher_class : public base_trencher_class
{

public:

    // Initiation
    manual_trencher_class(ros::NodeHandle nh);

    // manual motor functions
    void rightLinActBack(int& p_cmd, ros::NodeHandle  nh);
    void rightLinActForward(int& p_cmd, ros::NodeHandle  nh);
    void rightBucketForward(int& p_cmd, ros::NodeHandle  nh);
    void rightBucketBack(int& p_cmd, ros::NodeHandle  nh);
    void bucketsBack(int& p_cmd, ros::NodeHandle  nh);
    void bucketsForward(int& p_cmd, ros::NodeHandle  nh);

    void leftLinActBack(int& p_cmd, ros::NodeHandle  nh);
    void leftLinActForward(int& p_cmd, ros::NodeHandle  nh);
    void leftBucketForward(int& p_cmd, ros::NodeHandle  nh);
    void leftBucketBack(int& p_cmd, ros::NodeHandle  nh);
    void ballScrewIn(int &p_cmd, ros::NodeHandle nh);
    void ballScrewOut(int &p_cmd, ros::NodeHandle nh);

    void spinScoops(int &p_cmd, ros::NodeHandle nh);
    void scoopsBScrew(int &p_cmd, ros::NodeHandle nh);
    void linActsForward(int &p_cmd, ros::NodeHandle nh);
    void linActsBack(int &p_cmd, ros::NodeHandle nh);
    void bucketsTrencherBack(int& p_cmd, ros::NodeHandle nh);
    void turnTrencher(int& p_cmd, ros::NodeHandle  nh);

	void scoopsForward(int& p_cmd, ros::NodeHandle nh);
	void scoopsBack(int& p_cmd, ros::NodeHandle nh);
	void bucketsTrencher(int& p_cmd, ros::NodeHandle nh);

    // code clean w/ general functions
    void keepSpinningMotors(int& p_cmd, ros::NodeHandle  nh);
    void motorsStoppedSpinning(int& p_cmd, ros::NodeHandle  nh);

    void speedUpdate(ros::NodeHandle  nh);

    double linActSpeed;
    double bucketSpeed;
    double bScrewSpeed;
    double scoopsSpeed;
};
