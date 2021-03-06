/*!
 ******************************************************************************
 **  CarMaker - Version 7.1.2
 **  Vehicle Dynamics Simulation Toolkit
 **
 **  Copyright (C)   IPG Automotive GmbH
 **                  Bannwaldallee 60             Phone  +49.721.98520.0
 **                  76185 Karlsruhe              Fax    +49.721.98520.99
 **                  Germany                      WWW    www.ipg-automotive.com
 ******************************************************************************
 *
 * Description:
 * - Prototype/Proof of Concept
 * - Unsupported ROS Example with CarMaker
 * - Structure may change in future!
 * - Change general parameters in Infofile for CMRosIF ("Data/Config/CMRosIFParameters")
 * - Basic communication with or without parameterizable synchronization
 *
 *
 * ToDo:
 * - C++!!!
 * - ROS naming/way/namespaces
 * - parameter: CarMaker read, ROS set by service?
 *   -> ROS parameter mechanism seems better solution!
 * - node/topic/... destruction to allow dynamic load/unload
 *   when TestRun starts instead of initialization at CarMaker startup
 * - New Param_Get() function to read parameters from Infofile
 * - ...
 *
 */


/* CarMaker
 * - include other headers e.g. to access to vehicle data
 *   - e.g. "Vehicle.h" or "Vehicle/Sensor_*.h".
 * - additional headers can be found in "<CMInstallDir>/include/"
 * - see Reference Manual, chapter "User Accessible Quantities" to find some variables
 *   that are already defined in DataDictionary and their corresponding C-Code Name
 */
#include <boost/math/special_functions/sign.hpp>
#include "Log.h"
#include "DataDict.h"
#include "SimCore.h"
#include "InfoUtils.h"

#include "apo.h"
#include "GuiCmd.h"
#include "Traffic.h"
#include "Vehicle.h"
#include "VehicleControl.h"
#include "DrivMan.h"
#include "Vehicle/Sensor_Road.h"
#include "Vehicle/Sensor_GNav.h"
// #include "Vehicle/Sensor_Object.h"


/* ROS */
#include "cmrosutils/CMRosUtils.h"    /* Node independent templates, ...*/
#include "cmrosutils/CMRosIF_Utils.h" /* Only for CarMaker ROS Node!!! Functions are located in library for CarMaker ROS Interface */
#include "cmrosutils/CMRemote.h"      /* Basic service for CarMaker remote from ROS */

/* Following header from external ROS node can be used to get topic/service/... names
 * Other mechanism:
 * 1. Put names manually independently for each node
 * 2. Using command line arguments or launch files and ROS remapping
 * - Doing so, only general message headers are necessary
 */
#if 1
#  include "hellocm/ROS1_HelloCM.h"  /* External ROS Node. Topic name, ... */
#else
#  include <hellocm_msgs/Ext2CM.h>
#  include <hellocm_msgs/CM2Ext.h>
#  include <hellocm_msgs/Init.h>
#endif

/*! String and numerical version of this Node
 *  - String:    e.g. <Major>.<Minor>.<Patch>
 *  - Numerical: e.g. <nDigitsMajor><2DigitsMinor><2DigitsPatch>
 */
#define CMNODE_VERSION "0.8.0"
#define CMNODE_NUMVER  800


/* NDEBUG is set in CarMaker Makefile/MakeDefs in OPT_CFLAGS */
#if !defined NDEBUG
#  warning "Debug options are enabled!"
#  define DBLOG LOG
#else
#  define DBLOG(...)
#endif

/* Not beautiful but consistent with external ROS Node
 * where ROS_INFO is used (implicit newline)*/
# define LOG(frmt, ...)  Log(frmt "\n", ##__VA_ARGS__)


/* General switches for CarMaker ROS Node */
typedef enum tCMNode_Mode {
    CMNode_Mode_Disabled  = 0,  /*!< Node is disabled. e.g. don't publish. */
    CMNode_Mode_Default   = 1,  /*!< Node is enabled, spinOnce is used  */
    CMNode_Mode_Threaded  = 2   /*!< Node is enabled, spin in parallel thread
                                     - Messages are received all the time
                                     - Data is updated at defined position, e.g. *_In()
                                     - Currently not implemented! */
} tCMNode_Mode;



/* Managing synchronization between CarMaker Node and other ROS nodes */
typedef enum tCMNode_SyncMode {
    CMNode_SyncMode_Disabled  = 0, /*!< No synchronization on CarMaker side */
    CMNode_SyncMode_Tpc       = 1  /*!< Buffer messages or Spin until external Topics are received */
} tCMNode_SyncMode;



/* Global struct for this Node */
static struct {
    unsigned long  CycleNoRel;  /*!< CarMaker relative cycle number, e.g. since start of TestRun */

    struct {
        double         Duration;      /*!< Time spent for synchronization task */
        int            nCycles;       /*!< Number of cycles in synchronization loop */
        int            CyclePrepDone; /*!< Last cycle when preparation was done */
        int            CycleJobDone;  /*!< Last cycle when job was done */
        double         SynthDelay;    /*!< Synthetic delay in seconds provided to external node to check sync */
    } Sync; /*!< Synchronization related information */

    struct {
        int CycleNo;      /*!< Cycle number of external ROS Node (only for information) */

        /* For debugging */
        int            CycleLastOut;   /*!< Cycle number when Topic was published */
        int            CycleLastIn;    /*!< Cycle number when Topic from external ROS Node was received */
        int            CycleLastFlush; /*!< Cycle number when data from external ROS Node was provided to model */
    } Model; /*!< Model related information. ROS side! */

    struct {
        struct {
            tRosIF_TpcSub<hellocm_msgs::Ext2CM> Ext2CM; /* For this example also used for Synchronization */
        } Sub; /*!< Topics to be subscribed */

        struct {
            tRosIF_TpcPub<hellocm_msgs::CM2Ext> CM2Ext;

            /*!< CarMaker can be working as ROS Time Server providing simulation time
             *   starting at 0 for each TestRun */
            tRosIF_TpcPub<rosgraph_msgs::Clock> Clock;
        } Pub; /*!< Topics to be published */
    } Topics; /*!< ROS Topics used by this Node */

    struct {
        /*!< Initialization/Preparation of external ROS Node e.g. when simulation starts */
        tRosIF_Srv<hellocm_msgs::Init>    Init;
        tRosIF_Srv<cmrosutils::CMRemote>  CMRemote; // Trial
    } Services; /*!< ROS Services used by this Node (client and server)*/

    struct {
        int               QueuePub;     /*!< Queue size for Publishers */
        int               QueueSub;     /*!< Queue size for Subscribers */
        int               nCyclesClock; /*!< Number of cycles publishing /clock topic.
                                             CycleTime should be multiple of this value */
        tCMNode_Mode      Mode;
        tCMNode_SyncMode  SyncMode;
        double            SyncTimeMax;  /* Maximum Synchronization time */

        tRosIF_Cfg        Ros;
    } Cfg; /*!< General configuration for this Node */

} CMNode;

static double x;
static double y;
static double x_last;
static double y_last;
static double RouteVector_x;
static double RouteVector_y;
static double x_v;
static double y_v;
static double x_v_last;
static double y_v_last;
static double VehicleVector_x;
static double VehicleVector_y;



/*!
 * Description:
 * - Callback for ROS Topic published by external ROS Node
 *
 */
static void
cmnode_HelloCM_CB_TpcIn (const hellocm_msgs::Ext2CM::ConstPtr &msg)
{
    /* Process message only if receive is expected */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
	return;
    
    int rv;
    auto in = &CMNode.Topics.Sub.Ext2CM;

    /* Update receive buffer
     * - No lock for spinOnce necessary?
     */
    in->Msg.header  = msg->header;
    in->Msg.time    = msg->time;
    in->Msg.cycleno = msg->cycleno;
    in->Msg.sRoad = msg->sRoad;
    in->Msg.tRoad = msg->tRoad;
    in->Msg.SteeringAng = msg->SteeringAng;
    
    tTrafficObj *pObj = Traffic_GetByTrfId(0);
    pObj->sRoad = in->Msg.sRoad;
    pObj->tRoad = in->Msg.tRoad;
    
    
    /* Stopping simulation is only necessary when synchronization is activated */
    if (CMNode.Cfg.SyncMode == CMNode_SyncMode_Tpc && (rv = CMCRJob_DoPrep_SetDone(in->Job, CMNode.CycleNoRel)) != CMCRJob_RV_Success) {
	LogErrF(EC_Sim, "CMNode: Error on DoPrep_SetDone for Job '%s'! rv=%s", CMCRJob_GetName(in->Job), CMCRJob_RVStr(rv));
    }

    /* Remember cycle for debugging */
    CMNode.Model.CycleLastIn = CMNode.CycleNoRel;


    LOG("%s (CMSimTime=%.3fs): External Node is in cycle %lu, Time=%.3fs, Position=%.3fm Stamp=%.3fs, SeqID=%d",
	    ros::this_node::getName().c_str(), SimCore.Time,
	    in->Msg.cycleno, msg->time.toSec(), in->Msg.sRoad, in->Msg.header.stamp.toSec(), in->Msg.header.seq);

}



/*!
 * Description:
 * - Exemplary Service Callback for CarMaker Remote using ROS
 * - e.g. via rqt Service Caller or terminal "rosservice call ..."
 *
 *
 */
static bool
cmnode_HelloCM_CB_SrvCMRemote(cmrosutils::CMRemote::Request &req,
	cmrosutils::CMRemote::Response &resp)
{

    int rv = -2;
    char sbuf[512];

    LOG("%s: Service '%s' was triggered with type='%s', msg='%s', data='%s'",
	    ros::this_node::getName().c_str(),
	    CMNode.Services.CMRemote.Srv.getService().c_str(),
	    req.type.c_str(), req.msg.c_str(), req.data.c_str());


    /* Commands to CarMaker GUI
     * - Tcl commands!
     * - More information see "ProgrammersGuide"
     */
    if (strcasecmp("guicmd", req.type.c_str()) == 0) {
	/* Default: Evaluate command sent with message */
	if (strcasecmp("eval", req.msg.c_str()) == 0) {
	    /* e.g. data = 'LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim' */
	    rv = GuiCmd_Eval(req.data.c_str());
	} else {
	    if (strcasecmp("start", req.msg.c_str()) == 0) {
		if (req.data.length() == 0)
		    rv = GuiCmd_Eval("LoadTestRun CMRosIF/AdaptiveCruiseControl; StartSim");
		else {
		    sprintf(sbuf, "%s; StartSim", req.data.c_str());
		    rv = GuiCmd_Eval(sbuf);
		}
	    }
	    if (strcasecmp("stop", req.msg.c_str()) == 0)
		rv = GuiCmd_Eval("StopSim");
	}


	/* Commands directly to CarMaker Executable
	 * - Warning:
	 *   - Information normally provided by CarMaker GUI might be missing
	 */
    } else if (strcasecmp("cmd", req.type.c_str()) == 0) {
	if (strcasecmp("start", req.msg.c_str()) == 0) {
	    if (req.data.length() == 0) {
		/* Most strings are normally provided by CarMaker GUI */
		SimStart(NULL, ros::this_node::getName().c_str(),
			"CMRosIF/AdaptiveCruiseControl", NULL, NULL);
	    } else {
		/* Most strings are normally provided by CarMaker GUI */
		SimStart(NULL, ros::this_node::getName().c_str(),
			req.data.c_str(), NULL, NULL);
	    }
	}
	if (strcasecmp("stop", req.msg.c_str()) == 0)
	    SimStop2(0);
	rv = 0;
    }

    resp.res = rv;
    return true;
}



/*****************************************************************************/
/**********          C-Code for interfacing with CarMaker!          **********/
/*****************************************************************************/


#ifdef __cplusplus
extern "C" {
#endif



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Get versions from shared library
 * - Set the returned Version to 0 if there is no dependency!
 * - Compatibility check should be done by calling procedure
 *   as early as possible(e.g. before CMRosIF_CMNode_Init()
 *
 * Arguments:
 * - CMRosIFVer = CMRosIF shared library version (User defined)
 *                - Initially filled with version of CMRosIF management library
 * - CMNumVer   = CarMaker version used for shared library at compile time (normally CM_NUMVER)
 *                - Initially filled with version of CMRosIF management library
 * - RosVersion = ROS version used for shared library at compile time (normally ROS_VERSION)
 *                - Initially filled with version requested by CMRosIF management library (0 if no request)
 *
 */
int
CMRosIF_CMNode_GetVersion (unsigned long *CMRosIFCMNodeNumVer,
                           unsigned long *CMNumVer,
			   unsigned long *RosNumVer)
{

    *CMRosIFCMNodeNumVer = CMNODE_NUMVER;
    *CMNumVer            = CM_NUMVER;
    *RosNumVer           = ROS_VERSION;

    return 0;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Basic Initialization
 * - e.g. create ROS Node, subscriptions, ...
 * - Return values
 *   - "rv <  0" = Error at initialization, CarMaker executable will stop
 *   - "rv >= 0" = Everything OK, CarMaker executable will continue
 *
 * Arguments:
 * - Argc/Argv  = Arguments normally provided to ROS executable are not be provided
 *                to CM executable directly, but can be set in Infofile for CMRosIF
 *                with key "Node.Args" in "Data/Config/CMRosIFParameters"
 *
 * - CMNodeName = Default CarMaker Node name
 *                - Can be parameterized in Infofile for CMRosIF
 *                - Final node name might be different (argument remapping, ...)
 *
 * - Inf        = Handle to CarMaker Infofile with parameters for this interface
 *                - Please note that pointer may change, e.g. before TestRun begins
 *
 * ToDo:
 * - Possible to create/initialize node/... before each TestRun start instead of CM startup?
 * - New Param_Get() function to read parameters from Infofile
 */
int
CMRosIF_CMNode_Init (int Argc, char **Argv, char *CMNodeName, struct tInfos *Inf)
{

    int rv;
    bool rvb                = false;
    char sbuf[512]          = "";
    char keybuf[256]        = "";
    ros::NodeHandlePtr node = NULL;
    ros::V_string names;
    x = (*RoadSensor).Route.P_0[0];
    y = (*RoadSensor).Route.P_0[1];
    x_last = (*RoadSensor).Route.P_0[0];
    y_last = (*RoadSensor).Route.P_0[1];
    
    x_v = Vehicle.FrX.t_0[0];
    y_v = Vehicle.FrX.t_0[1];
 
    


    LOG("Initialize CarMaker ROS Node");
    LOG("  -> Node Version = %05d", CMNODE_NUMVER);
    LOG("  -> ROS Version  = %05d", ROS_VERSION);
    LOG("  -> CM Version   = %05d", CM_NUMVER);

    /* ROS initialization. Name of Node might be different after remapping! */
    if (ros::isInitialized() == false) {
	/* "Remapping arguments" functionality (launchfiles, ...)? */
	ros::init(Argc, Argv, CMNodeName);
    } else {
	//node.reset(); ToDo!
    }

    if (ros::master::check() == false) {
	LogErrF(EC_Init, "Can't contact ROS Master!\n Start roscore or run launch file e.g. via Extras->CMRosIF\n");
	ros::shutdown();
	return -1;
    }

    /* Node specific */
    CMNode.Cfg.Ros.Node = ros::NodeHandlePtr(boost::make_shared<ros::NodeHandle>());
    node                = CMNode.Cfg.Ros.Node;

    /* Publish specific */
    CMNode.Cfg.QueuePub  = iGetIntOpt(Inf, "Node.QueuePub", 1000); /* ToDo: Influence of queue length relevant? */

    /* Prepare the node to provide simulation time. CarMaker will be /clock server */
    strcpy(sbuf, "/use_sim_time");

    if ((rv = node->hasParam(sbuf)) == true) {
	node->getParam(sbuf, rvb);
	LOG("  -> Has param '%s' with value '%d'", sbuf, rvb);
    }

    /* Additional switch to provide simulation Time */
    strcpy(keybuf, "Node.UseSimTime");

    if ((rv = iGetIntOpt(Inf, keybuf, 1)) > 0) {
	/* Parameter must be set before other nodes start
	 * - set parameter outside to be independent from execution order?
	 */
	LOG("  -> Provide simulation time!");
	node->setParam("/use_sim_time", true); /* enable parameter if not already done */

	CMNode.Cfg.nCyclesClock  = iGetIntOpt(Inf, "Node.nCyclesClock", 1000);

	strcpy(sbuf, "/clock");
	LOG("    -> Publish '%s' every %dms", sbuf, CMNode.Cfg.nCyclesClock);
	CMNode.Topics.Pub.Clock.Pub  = node->advertise<rosgraph_msgs::Clock>(sbuf, CMNode.Cfg.QueuePub);


	/* ToDo: Necessary/Possible to ensure /clock is zeroed? */
	CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
	CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    } else {
	LOG("  -> Don't provide simulation time!");
	CMNode.Cfg.nCyclesClock  = 0;
    }

    strcpy(sbuf, hellocm::tpc_in_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Publish '%s'", sbuf);
    CMNode.Topics.Pub.CM2Ext.Pub         = node->advertise<hellocm_msgs::CM2Ext>(sbuf, CMNode.Cfg.QueuePub);
    CMNode.Topics.Pub.CM2Ext.Job         = CMCRJob_Create("CM2Ext");
    CMNode.Topics.Pub.CM2Ext.CycleTime   = 100; //time 5000
    CMNode.Topics.Pub.CM2Ext.CycleOffset = 0;

    /* Subscribe specific */
    CMNode.Cfg.QueueSub  = iGetIntOpt(Inf, "Node.QueueSub", 1); /* ToDo: Effect of queue length for subscriber? */


    strcpy(sbuf, hellocm::tpc_out_name.c_str() /*! Opposite in/out compared to external node */);
    LOG("  -> Subscribe '%s'", sbuf);
    CMNode.Topics.Sub.Ext2CM.Sub         = node->subscribe(sbuf, CMNode.Cfg.QueueSub, cmnode_HelloCM_CB_TpcIn);
    CMNode.Topics.Sub.Ext2CM.Job         = CMCRJob_Create("Ext2CM_for_Sync");

    /* In this example cycle time might be updated with value of external ROS Node
     * - See CMRosIF_CMNode_TestRun_Start_atBegin() */
    CMNode.Topics.Sub.Ext2CM.CycleTime   = 15000;

    /* Services */
    strcpy(sbuf, hellocm::srv_init_name.c_str());
    LOG("  -> Service Client '%s'", sbuf);
    CMNode.Services.Init.Clnt = node->serviceClient<hellocm_msgs::Init>(sbuf);


    strcpy(sbuf, "CMRemote");
    LOG("  -> Create Service '%s'", sbuf);
    CMNode.Services.CMRemote.Srv = node->advertiseService(
	    sbuf, cmnode_HelloCM_CB_SrvCMRemote);


    /* Print general information after everything is done */
    LOG("Initialization of ROS Node finished!");
    LOG("  -> Node Name = '%s'", ros::this_node::getName().c_str());
    LOG("  -> Namespace = '%s'", ros::this_node::getNamespace().c_str());


    /* Advertised Topics */
    ros::this_node::getAdvertisedTopics(names);
    LOG("  -> Advertised Topics (%lu)", names.size());

    auto it = names.begin();
    for (; it != names.end(); ++it)
	LOG("    -> %s", (*it).c_str());


    /* Subscribed Topics */
    names.clear();
    ros::this_node::getSubscribedTopics(names);
    LOG("  -> Subscribed Topics (%lu)", names.size());
    it = names.begin();
    for (; it != names.end(); ++it)
	LOG("    -> %s",  (*it).c_str());

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Add user specific Quantities for data storage
 *   and visualization to DataDictionary
 * - Called once at program start
 * - no realtime conditions
 *
 */
void
CMRosIF_CMNode_DeclQuants (void)
{

    tDDefault *df = DDefaultCreate("CMRosIF.");

    DDefULong   (df, "CycleNoRel",         "ms", &CMNode.CycleNoRel,               DVA_None);
    DDefInt     (df, "Sync.Cycles",        "-",  &CMNode.Sync.nCycles,             DVA_None);
    DDefDouble  (df, "Sync.Time",          "s",  &CMNode.Sync.Duration,            DVA_None);
    DDefInt     (df, "Sync.CyclePrepDone", "-",  &CMNode.Sync.CyclePrepDone,       DVA_None);
    DDefInt     (df, "Sync.CycleJobDone" , "-",  &CMNode.Sync.CycleJobDone,        DVA_None);
    DDefDouble4 (df, "Sync.SynthDelay",     "s", &CMNode.Sync.SynthDelay,          DVA_IO_In);

    DDefUChar   (df, "Cfg.Mode",           "-",  (unsigned char*)&CMNode.Cfg.Mode, DVA_None);
    DDefInt     (df, "Cfg.nCyclesClock",   "ms", &CMNode.Cfg.nCyclesClock,         DVA_None);
    DDefChar    (df, "Cfg.SyncMode",       "-",  (char*)&CMNode.Cfg.SyncMode,      DVA_None);
    DDefDouble4 (df, "Cfg.SyncTimeMax",    "s",  &CMNode.Cfg.SyncTimeMax,          DVA_IO_In);

    DDefInt     (df, "Mdl.CycleNo",        "-",  &CMNode.Model.CycleNo,            DVA_None);
    DDefInt     (df, "Mdl.CycleLastOut",   "ms", &CMNode.Model.CycleLastOut,       DVA_None);
    DDefInt     (df, "Mdl.CycleLastIn",    "ms", &CMNode.Model.CycleLastIn,        DVA_None);
    DDefInt     (df, "Mdl.CycleLastFlush", "ms", &CMNode.Model.CycleLastFlush,     DVA_None);

    DDefaultDelete(df);
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when starting a new TestRun
 * - In separate Thread (no realtime conditions)
 * - After standard Infofiles are read in
 * - Return values
 *   - "rv <  0" = Error, TestRun start will be aborted
 *   - "rv >= 0" = Everything OK
 *
 * Arguments:
 * - Inf = CarMaker Infofile for CMRosIF with content after TestRun start
 *         - Please note that the Infofile provided at initialization might have been updated!
 *
 * ToDo:
 * - New Param_Get() function to read parameters from Infofile
 *
 */
int
CMRosIF_CMNode_TestRun_Start_atBegin (struct tInfos *Inf)
{

    /* Node can be disabled via Infofile */
    tCMNode_Mode     *pmode     = &CMNode.Cfg.Mode;
    tCMNode_SyncMode *psyncmode = &CMNode.Cfg.SyncMode;

    if (Inf != NULL) {
	*pmode     =     (tCMNode_Mode)iGetIntOpt(Inf, "Node.Mode",      CMNode_Mode_Disabled);
	*psyncmode = (tCMNode_SyncMode)iGetIntOpt(Inf, "Node.Sync.Mode", CMNode_SyncMode_Disabled);
    }

    if (SimCore.CycleNo == 0 || Inf == NULL || *pmode == CMNode_Mode_Disabled) {
	*pmode = CMNode_Mode_Disabled;
	LOG("CarMaker ROS Node is disabled!");
	return 0;
    }

    char sbuf[512];
    char key[256];
    char *str        = NULL;
    int rv           = 0;
    bool rvb                = false;

    int cycletime           = 0;
    int *pcycletime         = NULL;
    int cycleoff            = 0;
    tCMCRJob *job           = NULL;
    auto srv         = &CMNode.Services.Init;

    LOG("CarMaker ROS Node is enabled! Mode=%d, SyncMode=%d", *pmode, *psyncmode);
    LOG("  -> Node Name = %s", ros::this_node::getName().c_str());


    /* Update synchronization */
    if (*psyncmode != CMNode_SyncMode_Disabled && *psyncmode != CMNode_SyncMode_Tpc) {
	LogErrF(EC_Sim, "CMNode: Invalid synchronization mode '%d'!",*psyncmode);
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    CMNode.Cfg.SyncTimeMax = iGetDblOpt(Inf, "Node.Sync.TimeMax", 1.0);


    /* Reset for next cycle */
    CMNode.CycleNoRel           =  0;
    CMNode.Sync.Duration        =  0.0;
    CMNode.Sync.nCycles         = -1;
    CMNode.Sync.CycleJobDone    = -1;
    CMNode.Sync.CyclePrepDone   = -1;
    CMNode.Model.CycleNo        = -1;
    CMNode.Model.CycleLastIn    = -1;
    CMNode.Model.CycleLastOut   = -1;
    CMNode.Model.CycleLastFlush = -1;


    /* Allow an update of the clock only if it was enabled before! */
    if (CMNode.Cfg.nCyclesClock > 0) {
	if ((rv = iGetIntOpt(Inf, "Node.nCyclesClock", 1000)) > 0)
	    CMNode.Cfg.nCyclesClock = rv;
    }

    /* Necessary to ensure /clock is zeroed here?
     * ToDo: Create function? */
    if (CMNode.Cfg.nCyclesClock > 0) {
	LOG("  -> Publish /clock every %dms", CMNode.Cfg.nCyclesClock);
	CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(0.0);
	CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }


    /* Prepare external node for next simulation */
    if (!srv->Clnt.exists()) {
	// ToDo: possible to get update if external ROS Node name changes?
	LogErrF(EC_Sim, "ROS Service is not ready! Please start external ROS Node providing Service '%s'!",
		srv->Clnt.getService().c_str());
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    LOG("  -> Send Service Request");

    /* ToDo: Async?*/
    if (!srv->Clnt.call(srv->Msg)) {
	LogErrF(EC_Sim, "ROS Service error!");
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    /* Update cycle time with information of external node */
#if 1
    /* Variant 1:
     * - Receiving parameters via ROS Parameter Server
     * - Parameter may be set externally e.g. by other node or arguments to command
     * - ROS parameters are more flexible than ROS services!
     */
    strcpy(sbuf, hellocm::prm_cycletime_name.c_str());
    if ((rv = CMNode.Cfg.Ros.Node->hasParam(sbuf)) == true)
	CMNode.Cfg.Ros.Node->getParam(sbuf, rv);
#else
    /* Variant 2:
     * - Receiving parameters from external Node via Service
     * - Services might be too "static"
     * - Not recommended!
     */
    rv = srv->Msg.response.cycletime;
#endif

    pcycletime = &CMNode.Topics.Sub.Ext2CM.CycleTime;

    if (*pcycletime != rv) {
	LOG("  -> Cycle time of external node changed from %dms to %dms", *pcycletime, rv);
	*pcycletime = rv;
    }


    /* Plausibility check for Cycle Time */
    if (CMNode.Cfg.nCyclesClock > 0 && (*pcycletime < CMNode.Cfg.nCyclesClock
	    || *pcycletime%CMNode.Cfg.nCyclesClock != 0)) {
	    
	LogErrF(EC_Sim, "Ext. ROS Node has invalid cycle time! Expected multiple of %dms but got %dms",
		CMNode.Cfg.nCyclesClock, *pcycletime);
		
	*pmode = CMNode_Mode_Disabled;
	return -1;
    }

    
    
    /* Prepare Jobs for publish and subscribe
     * - Special use case:
     *   - Topic in and Topic out use same cycle time with relative shift!
     */

    /* Start to publish when simulation starts */
    job       = CMNode.Topics.Pub.CM2Ext.Job;
    cycletime = CMNode.Topics.Pub.CM2Ext.CycleTime;
    cycleoff  = CMNode.Topics.Pub.CM2Ext.CycleOffset;

    CMCRJob_Init(job, cycleoff, cycletime, CMCRJob_Mode_Default);
    

    /* Synchronization with external node
     * - external node provides cycle time (see service above)
     * - other parameterization methods (e.g. ROS parameter, ...) are possible!
     * - Expect sync Topic are delayed (communication time, ...)
     * - This example shows sync via ROS Timer in external node
     *   - Therefore "/clock" topic needs to be published by CarMaker!
     *   - Other mechanism, e.g. data triggered on external node side
     *     via publishing Topic directly inside subscription callback is also possible!
     * - time=0.0 can't be detected by external node, therefore
     *   first receive needs to start after expected cycle time
     *   of external ROS node
     */

    job       = CMNode.Topics.Sub.Ext2CM.Job;
    cycletime = CMNode.Topics.Sub.Ext2CM.CycleTime;
    cycleoff  = CMNode.Topics.Sub.Ext2CM.CycleOffset = 0; /* No offset allowed if ROS Timer is used for sync!*/


    /* Create the synchronization jobs */
    if (*psyncmode == CMNode_SyncMode_Tpc) {
	CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Ext);

	LOG("  -> Synchronize on Topic '%s' (cycletime=%d, cycleoff=%d)",
		CMNode.Topics.Sub.Ext2CM.Sub.getTopic().c_str(), cycletime, cycleoff);

    } else
	CMCRJob_Init(job, cycletime+1 , cycletime, CMCRJob_Mode_Default);


    LOG("External ROS Node is ready to simulate");

    return 1;
}



/*!
 * ToDo:
 * - Put everything to TestRun_Start_atBegin?
 *
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Repeating call for several CarMaker cycles until return value is 1
 * - May be called even previous return value was 1
 * - See "User.c:User_TestRun_RampUp()"
 *
 */
int
CMRosIF_CMNode_TestRun_RampUp (void)
{

    /* Return immediately if node is disabled */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled)
	return 1;

    /* Put your code here */
    //if (NotReady) return 0;


    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called when TestRun ends (no realtime conditions)
 * - See "User.c:User_TestRun_End()"
 *
 */
int
CMRosIF_CMNode_TestRun_End (void)
{


    /* Put your code here */

    /* Disable after simulation has finished */
    CMNode.Cfg.Mode = CMNode_Mode_Disabled;

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called at very beginning of CarMaker cycle
 * - Process all topics/services using different modes or synchronization modes
 * - See "User.c:User_In()"
 *
 * ToDo:
 * - Additional spin mechanism
 *   - e.g. for HIL
 *   - e.g. spinning in new thread, copying incoming data here, ...
 *
 */
int
CMRosIF_CMNode_In (void)
{

    int rv                   = 0;
    int rx_done              = 0;
    const char *job_name     = NULL;
    tCMCRJob *job            = NULL;
    ros::WallTime tStart     = ros::WallTime::now();
    ros::WallDuration tDelta = ros::WallDuration(0.0);
    CMNode.Sync.nCycles      = 0;
    CMNode.Sync.Duration     = 0.0;

    switch (CMNode.Cfg.Mode) {
	case CMNode_Mode_Disabled:
	    /* Comment next line if no messages/services
	     * shall be processed in disabled Node state
	     */
	    ros::spinOnce();
	    break;

	case CMNode_Mode_Default:

	    if (CMNode.Cfg.SyncMode != CMNode_SyncMode_Tpc) {
                /* Process messages in queue, but do not block */
		ros::spinOnce();

	    } else {
		/* Synchronization based on expected Topics
		 * - Blocking call (process publish and wait for answer)
		 * - Stop simulation if maximum time is exceeded
		 */
		do {
		    ros::spinOnce();

		    /* Only do anything if simulation is running */
		    if (SimCore.State != SCState_Simulate) {
			rx_done = 1;
			break;
		    }

		    rx_done = 0;

		    /* Check all jobs if preparation is done */
		    job      = CMNode.Topics.Sub.Ext2CM.Job;

		    if ((rv = CMCRJob_DoPrep(job, CMNode.CycleNoRel, 0, NULL, NULL)) < CMCRJob_RV_OK) {
			LogErrF(EC_Sim, "CMNode: Error on DoPrep for Job '%s'! rv=%s",CMCRJob_GetName(job), CMCRJob_RVStr(rv));
			rx_done = 0;
			break;
		    }

		    /* If job is not done, remember name and prevent loop to finish */
		    job_name = (rv != CMCRJob_RV_DoSomething ? NULL : CMCRJob_GetName(job));
		    rx_done  = rv == CMCRJob_RV_DoNothing ? 1 : 0;

		    if (rx_done == 1)
			break;

		    /* Wait a little that data can arrive. WallTime, NOT ROS time!!!*/
		    ros::WallDuration(0.0).sleep();
		    tDelta = ros::WallTime::now() - tStart;
		    CMNode.Sync.nCycles++;

		} while (ros::ok() && rx_done == 0 && tDelta.toSec() < CMNode.Cfg.SyncTimeMax);

		/* Final calculation to get duration including last cycle before receive */
		tDelta = ros::WallTime::now() - tStart;

		CMNode.Sync.Duration = tDelta.toSec();

		if (rx_done != 1 && CMNode.Cfg.SyncTimeMax > 0.0 && tDelta.toSec() >= CMNode.Cfg.SyncTimeMax)
		    LogErrF(EC_Sim, "CMNode: Synchronization error! tDelta=%.3f, Last invalid Job='%s'\n", tDelta.toSec(), job_name);
	    }

	    break;

	case CMNode_Mode_Threaded:
	    /* ToDo
	     * - Spinning in parallel thread started before
	     * - Lock variables!
	     * - e.g. for HIL
	     */
	    break;

	default:
	    /* Invalid!!! */;
    }

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after driving maneuver calculation
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_DrivManCalc()"
 */
int
CMRosIF_CMNode_DrivMan_Calc (double dt)
{
    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;

    /* Put your code here */

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after CMRosIF_CMNode_DrivManCalc
 * - before CMRosIF_CMNode_VehicleControl_Calc()
 * - See "User.c:User_VehicleControl_Calc()"
 */
int
CMRosIF_CMNode_VehicleControl_Calc (double dt)
{
    /* Only do anything if simulation is running */
    
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;
     /*Put your code here */
    auto in = &CMNode.Topics.Sub.Ext2CM;
    int vz = in->Msg.vz;
    if (-((*RoadSensor).Route.Deviation.Dist) < 5.0  && Vehicle.v > 5.0 && vz > 0)
		VehicleControl.Brake = 1.0;
	//else if (-((*RoadSensor).Route.Deviation.Dist) > 22.0  && Vehicle.v > 5.0 )
		//VehicleControl.Brake = 1.0;
	//if (Vehicle.v < 1.0)
		//VehicleControl.Gas = 2.0;

    //VehicleControl.Brake = in->Msg.Brake;
    VehicleControl.Steering.Ang = in->Msg.SteeringAng;
    //VehicleControl.Steering.AngVel = in->Msg.SteeringAngSpeed;
    if (-((*RoadSensor).Route.Deviation.Dist) < 5.0 && Vehicle.v > 5.0 && vz > 0 )
		DrivMan.Brake = 1.0;
	//else if (-((*RoadSensor).Route.Deviation.Dist) > 22.0 && Vehicle.v > 5.0 )
		//DrivMan.Brake = 1.0;
	//if (Vehicle.v < 1.0)
		//DrivMan.Gas = 2.0;

    //DrivMan.Brake= in->Msg.Brake;
    DrivMan.Steering.Ang = in->Msg.SteeringAng;
    //DrivMan.Steering.AngVel = in->Msg.SteeringAngSpeed;
    
    
    return 1;
    
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called after vehicle model has been calculated
 * - See "User.c:User_Calc()"
 */
int
CMRosIF_CMNode_Calc (double dt)
{

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;

    /* Put your code here
     * - Update model parameters here?
     * - Do some calculation...
     */

    /* Update model with values from external node only in specific cycle?
     * - This data handling is optionl, but necessary for deterministic behaviour
     * - if synchronization is active, incoming data remains in msg buffer until correct cycle
     */
    int rv;
    auto sync = &CMNode.Topics.Sub.Ext2CM;

    if ((rv = CMCRJob_DoJob(sync->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
	    && rv != CMCRJob_RV_DoSomething) {
	LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s", CMCRJob_GetName(sync->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
	/* Something to do in sync cycle? */
	//CMCRJob_Info(in->Job, CMNode.CycleNoRel, "CMNode: Do Something for Sync: ");

	    /* Update model parameters here? */
	CMNode.Model.CycleNo = CMNode.Topics.Sub.Ext2CM.Msg.cycleno;


	/* Remember cycle for debugging */
	CMNode.Sync.CycleJobDone    = CMNode.CycleNoRel;
	CMNode.Model.CycleLastFlush = CMNode.CycleNoRel;
    }

    /* Do some calculation... */

    return 1;
}



/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called close to end of CarMaker cycle
 * - See "User.c:User_Out()"
 */
int
CMRosIF_CMNode_Out (void)
{
    ros::WallTime wtime = ros::WallTime::now();
    tVehicleCfg VehicleCfg;

    /* Only do anything if simulation is running */
    if (CMNode.Cfg.Mode == CMNode_Mode_Disabled
	    || SimCore.State != SCState_Simulate)
	return 0;

    int rv;
    auto out = &CMNode.Topics.Pub.CM2Ext;

    /* Communicate to External ROS Node in this cycle?
     * - The job mechanism is optional and can be e.g. replaced by simple modulo on current cycle
     */
    if ((rv = CMCRJob_DoJob(out->Job, CMNode.CycleNoRel, 1, NULL, NULL)) != CMCRJob_RV_DoNothing
	    && rv != CMCRJob_RV_DoSomething) {
	LogErrF(EC_Sim, "CMNode: Error on DoJob for Job '%s'! rv=%s",CMCRJob_GetName(out->Job), CMCRJob_RVStr(rv));
    } else if (rv == CMCRJob_RV_DoSomething) {
		
    
	out->Msg.cycleno      = CMNode.CycleNoRel;
	out->Msg.time         = ros::Time(SimCore.Time);
	out->Msg.synthdelay   = CMNode.Sync.SynthDelay;
    out->Msg.velocity     = Vehicle.v;
    out->Msg.SteeringAng     = Vehicle.Steering.Ang;
    out->Msg.tRoad  = Vehicle.tRoad;
    out->Msg.Gas  = VehicleControl.Gas;
    out->Msg.Brake  = VehicleControl.Brake;
    out->Msg.DevAngToRoute  = (*RoadSensor).Path.Deviation.Ang;
    out->Msg.DevDistToRoute  = (*RoadSensor).Route.Deviation.Dist;
    out->Msg.PrevViewPointX = (*RoadSensor).Route.P_0[0];
    out->Msg.PrevViewPointY = (*RoadSensor).Route.P_0[1];
    x = (*RoadSensor).Route.P_0[0]; 
    y = (*RoadSensor).Route.P_0[1];
    x_v = Vehicle.FrX.t_0[0];
    if(ros::Time(SimCore.Time).toSec() == 0){
		x_last = (*RoadSensor).Route.P_0[0];
		y_last = (*RoadSensor).Route.P_0[1];
	}
    y_v = Vehicle.FrX.t_0[1];
    out->Msg.VehiclePosX = x_v;
    out->Msg.VehiclePosX = y_v;
    out->Msg.PrevViewPointLastX = x_last;
    out->Msg.PrevViewPointLastY = y_last;
    out->Msg.yaw = Vehicle.Yaw;
    out->Msg.pitch = Vehicle.Pitch;
    out->Msg.roll = Vehicle.Roll;
   
    RouteVector_x = x-x_last; 
    RouteVector_y = y-y_last;
    VehicleVector_x = std::cos(Vehicle.Yaw);
    VehicleVector_y = std::sin(Vehicle.Yaw);
    
    x_last = x;
    y_last = y;
    x_v_last = x_v;
    y_v_last = y_v;
    
    const double vz = boost::math::sign(RouteVector_x * VehicleVector_y - RouteVector_y * VehicleVector_x);
    const double delta_angle = vz * std::acos( (RouteVector_x * VehicleVector_x + RouteVector_y * VehicleVector_y )/ ((std::sqrt(RouteVector_x * RouteVector_x + RouteVector_y * RouteVector_y) * (VehicleVector_x * VehicleVector_x + VehicleVector_y * VehicleVector_y)+0.001)));
    
    out->Msg.DeltaAng  = delta_angle; 
    out->Msg.vz  = vz;
 
    //delta_angle = signedAngleBetween(target_direction, vehicle_frame_unit_x);
    out->Msg.WheelBase  = VehicleCfg.WheelBase;
    /*ROS_INFO("VEHICLE_SPEED",Vehicle.v);
	/* Header stamp and frame needs to be set manually! */

	/* provide system time close to data is sent */
	wtime = ros::WallTime::now();
	out->Msg.header.stamp.sec  = wtime.sec;
	out->Msg.header.stamp.nsec = wtime.nsec;

	out->Pub.publish(out->Msg);

        /* Remember cycle for debugging */
	CMNode.Model.CycleLastOut = CMNode.CycleNoRel;
    }


    /* Publish "/clock" topic after all other other topics are published
     * - Is the order of arrival in other node identical? */
    if (CMNode.Cfg.nCyclesClock > 0 && CMNode.CycleNoRel%CMNode.Cfg.nCyclesClock == 0) {
	CMNode.Topics.Pub.Clock.Msg.clock = ros::Time(SimCore.Time);
	CMNode.Topics.Pub.Clock.Pub.publish(CMNode.Topics.Pub.Clock.Msg);
    }

    /* ToDo: When increase? */
    CMNode.CycleNoRel++;

    return 1;
}




/*!
 * Important:
 * - DO NOT CHANGE FUNCTION NAME !!!
 * - Automatically called by CMRosIF extension
 *
 * Description:
 * - Called one Time when CarMaker ends
 * - See "User.c:User_End()"
 */
int
CMRosIF_CMNode_End (void)
{

    LOG("%s: End", __func__);

    if (ros::isInitialized()) {

	/* Needs to be called before program exists, otherwise
	 * "boost" error due to shared library and default deconstructor */
	CMNode.Cfg.Ros.Node->shutdown();

	/* ToDo:
	 * - Blocking call? Wait until shutdown has finished?
	 * - Optional? */
	ros::shutdown();
    }

    return 1;
}



/*!
 * Important:
 * - NOT automatically called by CMRosIF extension
 *
 * Description:
 * - Example of user generated function
 * - Can be accessed in other sources, e.g. User.c
 * - Use "CMRosIF_GetSymbol()" to get symbol (see "lib/CMRosIF.h")
 *
 */
int
CMRosIF_CMNode_MyFunc (char *LogMsg)
{

    LOG("%s: %s",  __func__, LogMsg);
    return 1;
}

#ifdef __cplusplus
}
#endif


