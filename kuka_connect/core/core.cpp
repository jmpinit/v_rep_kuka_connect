#define LUA_LIB

#include <stdlib.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <string.h>

#include <tinyxml.h>

#include "lua.hpp"
#include "lauxlib.h"

#include "udp_server.h"

#define DEBUG

typedef struct RobotPose {
  double a1;
  double a2;
  double a3;
  double a4;
  double a5;
  double a6;
} RobotPose;

typedef struct ConnectionOptions {
  char* ipAddress;
  int port;
} ConnectionOptions;

static const double RAD2DEG = 57.295779513082323;
static const double DEG2RAD = 0.017453292519943295;

RobotPose currentPose = { 0, -90, 90, 0, 90, 0 };
pthread_mutex_t lockCurrentPose;
RobotPose sensedPose = { 0, -90, 90, 0, 90, 0 };
pthread_mutex_t lockSensePose;

bool rsiConnected = false;
bool haveSensedPose = false;

pthread_t threadRSI;

void die(const char *msg) {
  perror(msg);
  exit(1);
}

void *thread_rsi_fn(void *args) {
  ConnectionOptions* connectionOpts = (ConnectionOptions*)args;

  // Create a UDP connection to talk to the robot
  UDPServer udpServer(connectionOpts->ipAddress, connectionOpts->port);

  // Interpolation cycle count value has to match the query message from the
  // robot or RSI will stop, so we keep track of it
  unsigned long long ipoc;

  std::string reply;
  int numBytesRead = udpServer.recv(reply);

  // FIXME: is this necessary?
  if (numBytesRead < 100) {
    numBytesRead = udpServer.recv(reply);
  }

#ifdef DEBUG
  printf("Initial RSI packet received\n");
#endif

  // Get the initial position so we can start calculating corrections
  // TODO: move the message parsing to its own function
  RobotPose initialPose = {0};
  {
    TiXmlDocument bufferdoc;
    bufferdoc.Parse(reply.c_str());

    // Rob node has the pose data
    TiXmlElement *rob = bufferdoc.FirstChildElement("Rob");

    TiXmlElement *asPosEl = rob->FirstChildElement("ASPos");
    pthread_mutex_lock(&lockSensePose);
    asPosEl->Attribute("A1", &initialPose.a1);
    asPosEl->Attribute("A2", &initialPose.a2);
    asPosEl->Attribute("A3", &initialPose.a3);
    asPosEl->Attribute("A4", &initialPose.a4);
    asPosEl->Attribute("A5", &initialPose.a5);
    asPosEl->Attribute("A6", &initialPose.a6);
    pthread_mutex_unlock(&lockSensePose);

    // Get the IPOC timestamp
    TiXmlElement *ipocEl = rob->FirstChildElement("IPOC");
    ipoc = std::stoull(ipocEl->FirstChild()->Value());
  }

  // RSI needs a live connection so a short timeout is appropriate
  udpServer.set_timeout(1000);

#ifdef DEBUG
  printf("Starting main RSI loop\n");
#endif

  while (true) {
    if (!rsiConnected) {
      break;
    }

    // Grab a snapshot of the joint state
    struct RobotPose currentPoseCopy;
    {
      pthread_mutex_lock(&lockCurrentPose);
      memcpy(&currentPoseCopy, &currentPose, sizeof(RobotPose));
      pthread_mutex_unlock(&lockCurrentPose);
    }

    // Calculate joint corrections
    RobotPose poseCorrection;
    poseCorrection.a1 = -(currentPoseCopy.a1 - initialPose.a1); 
    poseCorrection.a2 = -(currentPoseCopy.a2 - initialPose.a2); 
    poseCorrection.a3 = -(currentPoseCopy.a3 - initialPose.a3); 
    poseCorrection.a4 = -(currentPoseCopy.a4 - initialPose.a4); 
    poseCorrection.a5 = -(currentPoseCopy.a5 - initialPose.a5); 
    poseCorrection.a6 = -(currentPoseCopy.a6 - initialPose.a6); 

    //printf("initial: [%f, %f, %f, %f, %f, %f]\n", initialPose.a1, initialPose.a2, initialPose.a3, initialPose.a4, initialPose.a5, initialPose.a6);
    //printf("current: [%f, %f, %f, %f, %f, %f]\n", currentPoseCopy.a1, currentPoseCopy.a2, currentPoseCopy.a3, currentPoseCopy.a4, currentPoseCopy.a5, currentPoseCopy.a6);
    //printf("correction: [%f, %f, %f, %f, %f, %f]\n", poseCorrection.a1, poseCorrection.a2, poseCorrection.a3, poseCorrection.a4, poseCorrection.a5, poseCorrection.a6);

    // Construct XML string with RSI update
    std::string xmlString;
    {
      TiXmlDocument xmlDoc;

      TiXmlElement *root = new TiXmlElement("Sen");
      root->SetAttribute("Type", "ImFree");

      TiXmlElement *el;
      
      // Set the correction values for each joint
      el = new TiXmlElement("AK");
      el->SetAttribute("A1", std::to_string(poseCorrection.a1));
      el->SetAttribute("A2", std::to_string(poseCorrection.a2));
      el->SetAttribute("A3", std::to_string(poseCorrection.a3));
      el->SetAttribute("A4", std::to_string(poseCorrection.a4));
      el->SetAttribute("A5", std::to_string(poseCorrection.a5));
      el->SetAttribute("A6", std::to_string(poseCorrection.a6));
      root->LinkEndChild(el);

      // Set the interpolation cycle count value
      el = new TiXmlElement("IPOC");
      el->LinkEndChild(new TiXmlText(std::to_string(ipoc)));
      root->LinkEndChild(el);

      xmlDoc.LinkEndChild(root);

      TiXmlPrinter printer;
      printer.SetStreamPrinting();
      xmlDoc.Accept(&printer);

      xmlString = printer.Str();
    }

    // Send the update to the robot
    udpServer.send(xmlString);

    // Read a message from the client
    udpServer.recv(reply);

    // Parse message from robot
    {
      TiXmlDocument bufferdoc;
      bufferdoc.Parse(reply.c_str());

      // Rob node has the pose data
      TiXmlElement *rob = bufferdoc.FirstChildElement("Rob");

      // Get the real (measured by sensors on robot) axis specific positions
      TiXmlElement *aiPosEl = rob->FirstChildElement("AIPos");
      pthread_mutex_lock(&lockSensePose);
      aiPosEl->Attribute("A1", &sensedPose.a1);
      aiPosEl->Attribute("A2", &sensedPose.a2);
      aiPosEl->Attribute("A3", &sensedPose.a3);
      aiPosEl->Attribute("A4", &sensedPose.a4);
      aiPosEl->Attribute("A5", &sensedPose.a5);
      aiPosEl->Attribute("A6", &sensedPose.a6);
      pthread_mutex_unlock(&lockSensePose);

      haveSensedPose = true;

      /*
      TiXmlElement *asPosEl = rob->FirstChildElement("ASPos");
      asPosEl->Attribute("A1", &initial_positions[0]);
      asPosEl->Attribute("A2", &initial_positions[1]);
      asPosEl->Attribute("A3", &initial_positions[2]);
      asPosEl->Attribute("A4", &initial_positions[3]);
      asPosEl->Attribute("A5", &initial_positions[4]);
      asPosEl->Attribute("A6", &initial_positions[5]);

      // Get the real cartesian tool position
      TiXmlElement *riStEl = rob->FirstChildElement("RIst");
      riStEl->Attribute("X", &cart_position[0]);
      riStEl->Attribute("Y", &cart_position[1]);
      riStEl->Attribute("Z", &cart_position[2]);
      riStEl->Attribute("A", &cart_position[3]);
      riStEl->Attribute("B", &cart_position[4]);
      riStEl->Attribute("C", &cart_position[5]);

      // Extract cartesian actual position
      TiXmlElement *rSolEl = rob->FirstChildElement("RSol");
      rSolEl->Attribute("X", &initial_cart_position[0]);
      rSolEl->Attribute("Y", &initial_cart_position[1]);
      rSolEl->Attribute("Z", &initial_cart_position[2]);
      rSolEl->Attribute("A", &initial_cart_position[3]);
      rSolEl->Attribute("B", &initial_cart_position[4]);
      rSolEl->Attribute("C", &initial_cart_position[5]);
      */

      // Get the IPOC timestamp
      TiXmlElement *ipocEl = rob->FirstChildElement("IPOC");
      ipoc = std::stoull(ipocEl->FirstChild()->Value());
    }

    //printf("reported: [%f, %f, %f, %f, %f, %f]\n", sensedPose.a1, sensedPose.a2, sensedPose.a3, sensedPose.a4, sensedPose.a5, sensedPose.a6);
  }

  return NULL;
}

static int kuka_connect_core_connect(lua_State *L) {
  // IP address
  if (!lua_isstring(L, 1)) {
    luaL_argerror(L, 1, "string");
    return 0;
  }

  // Port
  if (!lua_isnumber(L, 2)) {
    luaL_argerror(L, 2, "number");
    return 0;
  }

  int port = lua_tonumber(L, 2);
  const char* ipAddress = lua_tostring(L, 1);
  lua_pop(L, 2);

  ConnectionOptions *connectionOptions = (ConnectionOptions*)malloc(sizeof *connectionOptions);
  connectionOptions->ipAddress = (char*)calloc(1, strlen(ipAddress) + 1);
  strcpy(connectionOptions->ipAddress, ipAddress);
  connectionOptions->port = port;

  if (pthread_create(&threadRSI, NULL, thread_rsi_fn, connectionOptions)) {
    lua_pushstring(L, "Unable to start RSI thread");
    lua_error(L);
    return 0;
  }

  rsiConnected = true;

#ifdef DEBUG
  printf("RSI thread created\n");
#endif

  return 0;
}

static int kuka_connect_core_disconnect(lua_State *L) {
  if (!rsiConnected) {
    // Already disconnected
    return 0;
  }

#ifdef DEBUG
  printf("Disconnecting from robot\n");
#endif

  haveSensedPose = false;
  rsiConnected = false;
  pthread_join(threadRSI, NULL);

  return 0;
}

static int kuka_connect_core_get_pose(lua_State *L) {
  if (!haveSensedPose) {
    // No pose to report yet
    lua_pushnil(L);
    return 1;
  }

  pthread_mutex_lock(&lockSensePose);
  double values[6] = {
    sensedPose.a1 * DEG2RAD,
    sensedPose.a2 * DEG2RAD,
    sensedPose.a3 * DEG2RAD,
    sensedPose.a4 * DEG2RAD,
    sensedPose.a5 * DEG2RAD,
    sensedPose.a6 * DEG2RAD,
  };
  pthread_mutex_unlock(&lockSensePose);

  for (int i = 0; i < 6; i++) {
    lua_pushnumber(L, values[i]);
  }

  // # of return values: 6 joint angles
  return 6;
}

static int kuka_connect_core_update_pose(lua_State *L) {
  if (!lua_istable(L, 1)) {
    luaL_argerror(L, 1, "table");
    return 0;
  }

  if (lua_objlen(L, 1) != 6) {
    lua_pushstring(L, "Expected 6 values");
    lua_error(L);
    return 0;
  }

  // Read the joint angle table

  float values[6] = {0};

  for (int i = 0; i < 6; i++) {
    lua_pushnumber(L, i + 1); // Key
    lua_gettable(L, 1); // Pops key, pushes value
    values[i] = lua_tonumber(L, -1);
    lua_pop(L, 1); // Pop value
  }

  // Pop arguments
  lua_pop(L, 1);

  pthread_mutex_lock(&lockCurrentPose);
  currentPose.a1 = values[0] * RAD2DEG;
  currentPose.a2 = values[1] * RAD2DEG;
  currentPose.a3 = values[2] * RAD2DEG;
  currentPose.a4 = values[3] * RAD2DEG;
  currentPose.a5 = values[4] * RAD2DEG;
  currentPose.a6 = values[5] * RAD2DEG;
  pthread_mutex_unlock(&lockCurrentPose);

  return 0;
}

static int kuka_connect_core_get_assigned_ips(lua_State *L) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);

  struct ifreq ifr;
  ifr.ifr_addr.sa_family = AF_INET;
  strncpy(ifr.ifr_name, "en0", IFNAMSIZ - 1);

  ioctl(fd, SIOCGIFADDR, &ifr);

  close(fd);

  const char* ipAddr = inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr);
  lua_pushstring(L, ipAddr);

#ifdef DEBUG
  printf("get_assigned_ips called and returned %s\n", ipAddr);
#endif

  return 1;
}

static const struct luaL_Reg kuka_connect_core_funcs[] = {
  { "connect", kuka_connect_core_connect },
  { "disconnect", kuka_connect_core_disconnect },
  { "get_pose", kuka_connect_core_get_pose },
  { "update_pose", kuka_connect_core_update_pose },
  { "get_assigned_ips", kuka_connect_core_get_assigned_ips },
  { NULL, NULL },
};

static int hook_gc(lua_State *L) {
#ifdef DEBUG
  printf("GC hook called\n");
#endif
  rsiConnected = false;
  pthread_join(threadRSI, NULL);
  return 0;
}

static void stackDump(lua_State *L) {
  int i;
  int top = lua_gettop(L);
  for (i = 1; i <= top; i++) { /* repeat for each level */
    int t = lua_type(L, i);
    switch (t) {
      case LUA_TSTRING: /* strings */
        printf("\"%s\"", lua_tostring(L, i));
        break;

      case LUA_TBOOLEAN: /* booleans */
        printf(lua_toboolean(L, i) ? "true" : "false");
        break;

      case LUA_TNUMBER: /* numbers */
        printf("%g", lua_tonumber(L, i));
        break;

      default: /* other values */
        printf("%s", lua_typename(L, t));
        break;
    }
    printf("  "); /* put a separator */
  }
  printf("\n"); /* end the listing */
}

extern "C" {
  LUALIB_API int luaopen_kuka_connect_core(lua_State *L) {
    luaL_register(L, "kuka_connect_core", kuka_connect_core_funcs); // pushes new table with lib functions

    lua_pushstring(L, "gc_hook");
    lua_newuserdata(L, 1);

    luaL_newmetatable(L, "gc_hook_meta"); // pushes new table
    lua_pushstring(L, "__gc");
    lua_pushcfunction(L, hook_gc);
    lua_settable(L, -3); // Sets __gc for metatable. Pops key and value off stack
    lua_setmetatable(L, -2); // Sets metatable for userdata on stack. Pops table from the stack

    // Sets gc_hook key to userdata value on library table
    lua_settable(L, -3); // pops key and value off stack

  #ifdef DEBUG
    printf("Configured GC hook\n");
  #endif
    stackDump(L);

    return 1;
  }
}
